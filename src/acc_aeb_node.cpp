//==============================================================================
//  acc_aeb_node.cpp  —  ROS hardware interface for the ACC/AEB controller.
//                       NPUST Heavy Bus Platform — v15.3
//
//  THREADING MODEL
//    [ctrl spinner]   dedicated single-thread CallbackQueue: only the control
//                     timer fires here — non-re-entrant by construction.
//    [sensor spinner] ros::AsyncSpinner(N) on the global queue: writes only
//                     perc_mutex_-guarded perception buffers and lock-free atomics.
//    [watchdog]       std::thread on steady_clock: shares only lock-free atomics
//                     with the control loop; publishes via its OWN pre-built msgs.
//    [main]           installs SIGINT/SIGTERM, idles on WallRate, on shutdown
//                     quiesces both spinners + watchdog and asserts final safe-stop.
//
//  WATCHDOG SCOPE: catches a hung control thread while other threads run.
//  Cannot catch a whole-process freeze / power loss — the downstream CAN
//  gateway must enforce its own /control_value staleness timeout independently.
//
//  All periodic outbound messages are pre-allocated members reused in place;
//  diagnostic values written via snprintf into pre-reserved string capacity.
//==============================================================================
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>

#include <atomic>
#include <mutex>
#include <thread>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cmath>
#include <memory>
#include <algorithm>

#include "pro_can/control_data.h"
#include "pro_can/VCU.h"
#include <npust_bus_msgs/TrackedObjectArray.h>
#include <npust_bus_msgs/LanePolynomial.h>

#include "acc_aeb_controller/types.h"
#include "acc_aeb_controller/math_utils.h"
#include "acc_aeb_controller/params.h"
#include "acc_aeb_controller/kinematics.h"
#include "acc_aeb_controller/state_machine.h"

namespace acc_aeb {

// Shared cross-thread state must be genuine lock-free atomics (a locked atomic
// would be unsafe to touch from a real-time / signal-adjacent context). Fail the
// build loudly on an exotic target rather than silently degrade.
static_assert(std::atomic<bool>::is_always_lock_free,     "atomic<bool> must be lock-free on this target");
static_assert(std::atomic<int64_t>::is_always_lock_free,  "atomic<int64_t> must be lock-free on this target");
static_assert(std::atomic<uint32_t>::is_always_lock_free, "atomic<uint32_t> must be lock-free on this target");
static_assert(std::atomic<double>::is_always_lock_free,   "atomic<double> must be lock-free on this target");

// Set ONLY by the SIGINT/SIGTERM handler (async-signal-safe: one lock-free
// atomic store, no allocation, no ROS calls). Polled by main().
std::atomic<bool> g_shutdown_requested{false};
extern "C" void onShutdownSignal(int) { g_shutdown_requested.store(true, std::memory_order_relaxed); }

// Monotonic ns timestamp — used for the watchdog so NTP steps / sim-time edits
// cannot mask a real freeze.
[[nodiscard]] static inline int64_t steadyNowNs() noexcept {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch()).count();
}

// Why the controller is (or would be) in FAULT — for operator-facing diagnostics.
// PERCEPTION = both camera AND radar stale (single-sensor loss is degraded, not fault).
enum class FaultSrc : uint8_t { NONE, PERCEPTION, VCU, BOTH };
[[nodiscard]] static inline const char* faultSrcStr(FaultSrc f) noexcept {
    switch (f) {
        case FaultSrc::NONE:       return "none";
        case FaultSrc::PERCEPTION: return "perception_stale";
        case FaultSrc::VCU:        return "vcu_stale";
        case FaultSrc::BOTH:       return "perception+vcu_stale";
    }
    return "unknown";
}

class AccAebNode {
public:
    AccAebNode(ros::NodeHandle& nh, ros::NodeHandle& nh_priv)
        : nh_(nh), nh_priv_(nh_priv), p_(loadParams(nh_priv)),
          tracker_(p_), sm_(p_)
    {
        validateParams(p_);

        safe_brake_g_          = std::abs(p_.a_fault_brake) * INV_G_MPS2;
        overrun_threshold_ns_  = static_cast<int64_t>(consts::LOOP_OVERRUN_FACTOR * p_.dt_s * 1e9);

        // TEST-ONLY one-shot stall to exercise the watchdog -> safe-stop path on
        // the bench (stalls the control thread only; the watchdog still runs).
        nh_priv_.param("debug_stall_once_s", debug_stall_once_s_, 0.0);
        if (debug_stall_once_s_ > 0.0) {
            ROS_WARN("[ACC/AEB] *** debug_stall_once_s=%.3f is ENABLED (test hook) — do not use on the vehicle ***",
                     debug_stall_once_s_);
        }

        if (!nh_priv_.hasParam("vx_is_relative")) {
            ROS_WARN("[ACC/AEB] *** 'vx_is_relative' not set in launch file ***");
            ROS_WARN("[ACC/AEB] Defaulting to FALSE (world-frame absolute velocity).");
            ROS_WARN("[ACC/AEB] Verify against your tracker before road testing.");
        }

        dynamic_target_speed_mps_.store(p_.target_speed_mps);

        initControlMsgs();
        initDiagMsgs();
        setupPubSub();   // creates the control timer on the dedicated queue

        // Seed the heartbeat BEFORE the watchdog starts; loop_started_ is the
        // real first-tick guard.
        last_loop_steady_ns_.store(steadyNowNs(), std::memory_order_relaxed);
        watchdog_active_.store(true, std::memory_order_relaxed);
        watchdog_thread_ = std::thread(&AccAebNode::watchdogLoop, this);

        // Bring the control loop live on its own isolated thread.
        ctrl_spinner_ = std::make_unique<ros::AsyncSpinner>(1, &ctrl_queue_);
        ctrl_spinner_->start();

        ROS_INFO("[ACC/AEB] v16.0 ready | %.0f kg | cam_vx=%s rad_vx=%s | coast=%.2f/%.2f s | spinners=%d | wd=%.0f ms | vcu_to=%.0f ms",
                 p_.mass_kg,
                 p_.vx_is_relative       ? "relative" : "world",
                 p_.radar_vx_is_relative ? "relative" : "world",
                 p_.min_coast_s_normal, p_.min_coast_s_warn,
                 p_.spinner_threads, p_.watchdog_timeout_s * 1e3, p_.vcu_timeout_s * 1e3);
    }

    ~AccAebNode() {
        stopControlSpinner();
        stopWatchdog();
    }

    [[nodiscard]] const Params& params() const noexcept { return p_; }

    // Called by main() on graceful shutdown, AFTER the sensor spinner is stopped
    // and BEFORE ros::shutdown() (transport still alive). Quiesces every internal
    // publisher (control loop + watchdog), then repeatedly asserts a hardware
    // safe-stop to maximize the chance it reaches the CAN gateway before teardown.
    void engageShutdownSafeStop() {
        stopControlSpinner();
        stopWatchdog();
        ROS_WARN("[ACC/AEB] Shutdown requested - asserting final hardware safe-stop.");
        ros::WallRate r(100.0);
        for (int i = 0; i < 5 && ros::ok(); ++i) {   // ~50 ms of repeated assertion
            publishLoopSafeStop(ros::Time::now());
            r.sleep();
        }
    }

private:
    //--------------------------------------------------------------------------
    //  Pre-allocated diagnostic key layouts.
    //--------------------------------------------------------------------------
    enum DiagIdx {
        D_TARGET_KPH = 0, D_EGO_KPH, D_MIO_X, D_CLOSING, D_TTC,
        D_CMD, D_LANE_L, D_LANE_R, D_DYN_W, D_WD_TRIPS, D_FAULT_SRC, D_LOOP_OVERRUNS,
        D_CAM_OK, D_RAD_OK, D_OVERRIDE,
        D_CAM_CONTENT,   // 1 = camera sending non-empty msgs recently; 0 = possible silent failure
        D_ADJ_MIO_X,     // closest adjacent-lane object distance (m); -1 if none
        D_ADJ_TTC,       // adjacent-lane object TTC (s); capped at 99.9
        D_COUNT
    };
    enum WdDiagIdx { WD_FROZEN_S = 0, WD_TRIP_COUNT, WD_COUNT };

    void initControlMsgs() {
        ctrl_msg_.steering_wheel_command_value = 0.0;   // longitudinal node: steering is constant zero

        // Constant-payload safe-stop messages (only header.stamp varies). Each is
        // owned by exactly one thread (loop_stop_msg_ -> control loop / main;
        // wd_stop_msg_ -> watchdog), so reuse is race-free.
        loop_stop_msg_.acc_command_value            = 0.0;
        loop_stop_msg_.brake_command_value          = safe_brake_g_;
        loop_stop_msg_.steering_wheel_command_value = 0.0;

        wd_stop_msg_.acc_command_value            = 0.0;
        wd_stop_msg_.brake_command_value          = safe_brake_g_;
        wd_stop_msg_.steering_wheel_command_value = 0.0;
    }

    void initDiagMsgs() {
        diag_msg_.status.resize(1);
        auto& st = diag_msg_.status[0];
        st.name        = "acc_aeb_controller";
        st.hardware_id = "npust_bus";
        st.values.resize(D_COUNT);
        static const char* const kKeys[D_COUNT] = {
            "target_kph", "ego_kph", "mio_x_m", "closing_mps", "ttc_s",
            "cmd_mps2", "lane_l_valid", "lane_r_valid", "dynamic_half_w",
            "watchdog_trips", "fault_source", "loop_overruns",
            "cam_ok", "rad_ok", "driver_override",
            "cam_content_ok", "adj_mio_x_m", "adj_ttc_s"
        };
        for (int i = 0; i < D_COUNT; ++i) {
            st.values[i].key = kKeys[i];
            st.values[i].value.reserve(24);     // one-time alloc; assigns never realloc
        }
        st.message.reserve(16);

        // Separate watchdog status so a frozen-loop hard-stop is distinguishable
        // from a sensor FAULT at a glance, and visible even while the loop is dead.
        wd_diag_msg_.status.resize(1);
        auto& wst = wd_diag_msg_.status[0];
        wst.name        = "acc_aeb_watchdog";
        wst.hardware_id = "npust_bus";
        wst.level       = diagnostic_msgs::DiagnosticStatus::ERROR;
        wst.message.reserve(64);
        wst.message     = "Control loop frozen - hardware safe-stop engaged";
        wst.values.resize(WD_COUNT);
        static const char* const kWdKeys[WD_COUNT] = { "frozen_for_s", "trip_count" };
        for (int i = 0; i < WD_COUNT; ++i) {
            wst.values[i].key = kWdKeys[i];
            wst.values[i].value.reserve(24);
        }
    }

    void setupPubSub() {
        // Sensor subscriptions live on the GLOBAL callback queue (serviced by the
        // sensor spinner started in main()).
        sub_vcu_           = nh_.subscribe("/VCU_Data", 1, &AccAebNode::vcuCb, this);
        sub_objects_       = nh_.subscribe("/perception/camera/tracked_objects", 1, &AccAebNode::objectsCb, this);
        sub_radar_         = nh_.subscribe("/perception/radar/tracked_objects",  1, &AccAebNode::radarCb, this);
        sub_lane_l_        = nh_.subscribe("/perception/lanes/left", 1, &AccAebNode::laneLeftCb, this);
        sub_lane_r_        = nh_.subscribe("/perception/lanes/right", 1, &AccAebNode::laneRightCb, this);
        sub_planner_speed_ = nh_.subscribe("/planner/target_speed", 1, &AccAebNode::plannerSpeedCb, this);
        sub_override_      = nh_.subscribe("/driver_override", 1, &AccAebNode::overrideCb, this);

        pub_ctrl_ = nh_.advertise<pro_can::control_data>("/control_value", 1);
        pub_diag_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1);

        // Control timer on its OWN queue -> isolated, non-re-entrant control path.
        // ros::NodeHandle::createTimer(TimerOptions&) requires a named lvalue (non-const ref).
        ros::TimerOptions ctrl_timer_opts(
            ros::Duration(p_.dt_s),
            [this](const ros::TimerEvent& e) { controlLoop(e); },
            &ctrl_queue_);
        ctrl_timer_ = nh_.createTimer(ctrl_timer_opts);
    }

    //--------------------------------------------------------------------------
    //  Sensor callbacks (run on sensor-spinner threads).
    //--------------------------------------------------------------------------
    void vcuCb(const pro_can::VCU::ConstPtr& msg) {
        const double v_kph = static_cast<double>(msg->MotorVelocity);
        if (!std::isfinite(v_kph)) {
            // Reject and do NOT refresh the freshness stamp: a persistently bad
            // VCU then trips the staleness timeout -> FAULT (controlled stop),
            // rather than us silently trusting a junk own-speed.
            ROS_ERROR_THROTTLE(1.0, "[ACC/AEB] Non-finite VCU MotorVelocity. Ignoring.");
            return;
        }
        ego_v_mps_.store(std::max(0.0, v_kph * KPH_2_MPS), std::memory_order_relaxed);
        vcu_last_arrival_sec_.store(ros::Time::now().toSec(), std::memory_order_relaxed);
        vcu_ever_received_.store(true, std::memory_order_relaxed);
    }

    void plannerSpeedCb(const std_msgs::Float32::ConstPtr& msg) {
        if (!std::isfinite(msg->data)) {
            ROS_ERROR_THROTTLE(1.0, "[ACC/AEB] NaN received from planner. Ignoring.");
            return;
        }
        const double capped = std::min(static_cast<double>(msg->data), p_.target_speed_mps);
        dynamic_target_speed_mps_.store(std::max(0.0, capped), std::memory_order_relaxed);
    }

    void objectsCb(const npust_bus_msgs::TrackedObjectArray::ConstPtr& msg) {
        std::lock_guard<std::mutex> lk(perc_mutex_);
        latest_objects_         = msg;
        last_obj_arrival_stamp_ = ros::Time::now();
        obj_ever_received_      = true;
        if (!msg->objects.empty()) {
            last_cam_nonempty_stamp_ = last_obj_arrival_stamp_;
            cam_nonempty_ever_       = true;
        }
    }

    void radarCb(const npust_bus_msgs::TrackedObjectArray::ConstPtr& msg) {
        std::lock_guard<std::mutex> lk(perc_mutex_);
        latest_radar_objects_      = msg;
        last_radar_arrival_stamp_  = ros::Time::now();
        radar_ever_received_       = true;
    }

    void overrideCb(const std_msgs::Bool::ConstPtr& msg) {
        driver_override_active_.store(msg->data, std::memory_order_relaxed);
    }

    void laneLeftCb(const npust_bus_msgs::LanePolynomial::ConstPtr& msg) {
        std::lock_guard<std::mutex> lk(perc_mutex_);
        lane_l_raw_                = *msg;
        lane_l_valid_              = isLaneValid(*msg, p_);
        last_lane_l_arrival_stamp_ = ros::Time::now();
    }

    void laneRightCb(const npust_bus_msgs::LanePolynomial::ConstPtr& msg) {
        std::lock_guard<std::mutex> lk(perc_mutex_);
        lane_r_raw_                = *msg;
        lane_r_valid_              = isLaneValid(*msg, p_);
        last_lane_r_arrival_stamp_ = ros::Time::now();
    }

    // Snapshot under perc_mutex_: shared_ptr copy keeps the array alive for the
    // full tick so selectMIO() reads it lock-free. "ever received" prevents a
    // startup false-fresh window when sim-time origin differs from wall-clock.
    [[nodiscard]] PerceptionSnapshot snapshot(const ros::Time& now, double ego_v) {
        std::lock_guard<std::mutex> lk(perc_mutex_);
        PerceptionSnapshot s;
        s.objects      = latest_objects_;
        s.cam_timeout  = !obj_ever_received_ ||
                         ((now - last_obj_arrival_stamp_).toSec() > p_.sensor_timeout_s);
        s.radar_objects = latest_radar_objects_;
        s.radar_timeout = !radar_ever_received_ ||
                          ((now - last_radar_arrival_stamp_).toSec() > p_.radar_timeout_s);
        s.cam_nonempty_sec  = last_cam_nonempty_stamp_.toSec();
        s.cam_nonempty_ever = cam_nonempty_ever_;

        const bool l_current = lane_l_valid_ && ((now - last_lane_l_arrival_stamp_).toSec() <= p_.sensor_timeout_s);
        const bool r_current = lane_r_valid_ && ((now - last_lane_r_arrival_stamp_).toSec() <= p_.sensor_timeout_s);
        if (l_current) s.lane_l = {lane_l_raw_.a, lane_l_raw_.b, lane_l_raw_.c, true};
        else           s.lane_l = {0.0, 0.0, 0.0, false};
        if (r_current) s.lane_r = {lane_r_raw_.a, lane_r_raw_.b, lane_r_raw_.c, true};
        else           s.lane_r = {0.0, 0.0, 0.0, false};

        s.dynamic_half_w = std::max(p_.min_static_half_w_m,
                                    p_.static_half_w_m - (ego_v * p_.lane_narrow_rate));
        return s;
    }

    [[nodiscard]] bool vcuStale(const ros::Time& now) const {
        if (!vcu_ever_received_.load(std::memory_order_relaxed)) return true;
        return (now.toSec() - vcu_last_arrival_sec_.load(std::memory_order_relaxed)) > p_.vcu_timeout_s;
    }

    //--------------------------------------------------------------------------
    //  Main control tick (dedicated single-thread queue -> non-re-entrant).
    //--------------------------------------------------------------------------
    void controlLoop(const ros::TimerEvent& evt) {
        // TEST-ONLY one-shot stall, BEFORE the heartbeat refresh so the watchdog
        // observes a genuine freeze. Zero cost when disabled.
        if (debug_stall_once_s_ > 0.0) {
            const double s = debug_stall_once_s_;
            debug_stall_once_s_ = 0.0;
            ROS_WARN("[ACC/AEB] TEST: injecting %.3f s control-loop stall to exercise the watchdog.", s);
            ros::WallDuration(s).sleep();
        }

        double actual_dt = p_.dt_s;
        if (!evt.last_real.isZero()) {
            actual_dt = clampVal((evt.current_real - evt.last_real).toSec(),
                                 p_.dt_s * 0.5, p_.dt_s * 3.0);
        }
        const ros::Time now = evt.current_real;

        // Heartbeat (monotonic, lock-free) + wall-clock loop-overrun accounting.
        const int64_t now_steady = steadyNowNs();
        const int64_t prev_steady = last_loop_steady_ns_.exchange(now_steady, std::memory_order_relaxed);
        loop_started_.store(true, std::memory_order_relaxed);
        if (!first_control_tick_ && (now_steady - prev_steady) > overrun_threshold_ns_) {
            ++loop_overrun_count_;
        }
        first_control_tick_ = false;

        // Consume a pending watchdog trip: the watchdog already published a safe-
        // stop, so do a FULL command-pipeline reset to resume coherently.
        if (watchdog_tripped_.exchange(false, std::memory_order_acq_rel)) {
            ROS_WARN("[ACC/AEB] Resuming after watchdog safe-stop - full command reset.");
            sm_.onSafeStopEngaged(p_.a_fault_brake, safe_brake_g_);
        }

        const double ego_v = ego_v_mps_.load(std::memory_order_relaxed);
        if (!std::isfinite(ego_v)) {   // defensive: vcuCb already rejects NaN
            ROS_ERROR_THROTTLE(1.0, "[ACC/AEB] FATAL: Invalid ego speed (NaN). Forcing safe stop.");
            publishLoopSafeStop(now);
            sm_.onSafeStopEngaged(p_.a_fault_brake, safe_brake_g_);
            return;
        }

        const PerceptionSnapshot snap = snapshot(now, ego_v);

        // Perception fault only when BOTH camera AND radar are stale.
        // A single-sensor loss is degraded mode (selectMIO uses whichever source is fresh).
        const bool     cam_stale = snap.cam_timeout;
        const bool     rad_stale = snap.radar_timeout;
        const bool     obj_stale = cam_stale && rad_stale;
        const bool     vcu_stale = vcuStale(now);
        const bool     sensor_ok = !obj_stale && !vcu_stale;
        const FaultSrc fsrc = (obj_stale && vcu_stale) ? FaultSrc::BOTH
                            :  obj_stale               ? FaultSrc::PERCEPTION
                            :  vcu_stale               ? FaultSrc::VCU
                            :                            FaultSrc::NONE;

        if (cam_stale && !rad_stale)
            ROS_WARN_THROTTLE(5.0, "[ACC/AEB] Camera stale — running on radar only (degraded).");
        else if (rad_stale && !cam_stale)
            ROS_INFO_THROTTLE(5.0, "[ACC/AEB] Radar stale — running on camera only.");

        const MioResult cam_raw = (!snap.cam_timeout && snap.objects)
                                  ? selectMIO(snap, ego_v, p_, SensorSource::CAMERA_ONLY)
                                  : MioResult{};
        const MioResult rad_raw = (!snap.radar_timeout && snap.radar_objects)
                                  ? selectMIO(snap, ego_v, p_, SensorSource::RADAR_ONLY)
                                  : MioResult{};

        // Adjacent MIO is computed before the tracker so an imminent cut-in can be
        // escalated to the FSM before the vehicle finishes crossing the lane line.
        //
        // DIRECT AEB PROMOTION only (no pre-warm):
        //   When adj_kin.ttc < ttc_aeb_s we bypass the tracker and route the raw
        //   adjacent MIO straight to the FSM.  Pre-warming the tracker at the wider
        //   ttc_warn_s threshold was removed: longitudinal TTC alone cannot distinguish
        //   a genuine cut-in from ego overtaking a slower adjacent vehicle, and that
        //   misidentification would trigger false WARN braking on a bus route with
        //   mixed-speed traffic.
        const MioResult adj_mio = !obj_stale ? selectAdjacentMIO(snap, ego_v, p_) : MioResult{};
        const KinResult adj_kin = adj_mio.valid ? computeKinematics(ego_v, adj_mio, p_) : KinResult{};

        if (adj_mio.valid) {
            ROS_INFO_THROTTLE(2.0,
                "[ACC/AEB] Adjacent-lane object: x=%.1fm TTC=%.1fs — monitoring for cut-in.",
                adj_mio.x, std::min(adj_kin.ttc, consts::DIAG_TTC_DISPLAY_CAP));
        }

        const MioResult mio = tracker_.track(cam_raw, rad_raw, ego_v, actual_dt);
        const KinResult kin = (mio.valid || mio.stale) ? computeKinematics(ego_v, mio, p_) : KinResult{};

        // Direct AEB promotion: bypass the tracker when the adjacent object is at
        // AEB-level TTC and is more urgent than whatever the tracker currently holds.
        MioResult fsm_mio = mio;
        KinResult fsm_kin = kin;
        if (adj_mio.valid && adj_kin.ttc < p_.ttc_aeb_s &&
            (!mio.valid || adj_kin.ttc < kin.ttc)) {
            fsm_mio = adj_mio;
            fsm_kin = adj_kin;
            ROS_WARN_THROTTLE(0.5,
                "[ACC/AEB] CUT-IN AEB: adjacent x=%.1fm TTC=%.1fs "
                "— direct AEB trigger (vehicle may not yet be fully in ego lane).",
                adj_mio.x, std::min(adj_kin.ttc, consts::DIAG_TTC_DISPLAY_CAP));
        }

        // Camera content health: camera can be alive (fresh messages) but broken
        // (ZED sending all-NaN depth → YOLO publishes empty arrays continuously).
        // We cannot FAULT here (a clear road with no objects is valid), but we warn
        // the operator so they can inspect the sensor.
        const bool cam_content_ok =
            snap.cam_timeout ||          // cam stale → staleness FAULT handles it; skip this check
            !snap.cam_nonempty_ever ||   // never received objects: clear road / startup
            ego_v <= p_.ego_v_moving_thres ||  // stationary: objects may legitimately be absent
            (now.toSec() - snap.cam_nonempty_sec) <= p_.cam_health_timeout_s;
        if (!cam_content_ok) {
            ROS_WARN_THROTTLE(10.0,
                "[ACC/AEB] Camera delivering empty arrays for >%.0f s while moving "
                "— possible sensor silent failure. Verify ZED hardware.",
                p_.cam_health_timeout_s);
        }

        const double target_speed  = dynamic_target_speed_mps_.load(std::memory_order_relaxed);
        const bool override_active = driver_override_active_.load(std::memory_order_relaxed);
        // When driver override is active, shield the FSM from the sensor-stale signal
        // so it never enters FAULT and never applies the fault brake against the driver.
        // Stale sensors while override is active = degraded manual driving, not a fault.
        // The warning block below tells the operator that AEB is non-functional.
        const bool sm_sensor_ok = sensor_ok || override_active;
        const ControlResult cr = sm_.step(fsm_mio, fsm_kin, ego_v, actual_dt, sm_sensor_ok, target_speed);

        if (!std::isfinite(cr.accel_cmd)) {
            ROS_ERROR_THROTTLE(1.0, "[ACC/AEB] FATAL: NaN in longitudinal command. Forcing safe stop.");
            publishLoopSafeStop(now);
            sm_.onSafeStopEngaged(p_.a_fault_brake, safe_brake_g_);
            return;
        }

        if      (cr.event == SmEvent::ENTERED_FAULT)   ROS_WARN("[ACC/AEB] FAULT entered (%s).", faultSrcStr(fsrc));
        else if (cr.event == SmEvent::FAULT_RECOVERED) ROS_WARN("[ACC/AEB] Sensors recovered - resuming ops.");

        // Log every state transition so WARN and AEB entries appear in the ros log.
        if (cr.state != last_state_) {
            ROS_WARN("[ACC/AEB] State: %s -> %s", toString(last_state_), toString(cr.state));
            last_state_ = cr.state;
        }

        if (override_active != prev_override_state_) {
            if (override_active) ROS_WARN("[ACC/AEB] Driver override ACTIVE — ACC suspended.");
            else                 ROS_WARN("[ACC/AEB] Driver override released — ACC resuming.");
            prev_override_state_ = override_active;
        }

        // Sensors are actually stale even though the FSM doesn't know (sm_sensor_ok masked it).
        // Warn loudly so the operator knows AEB cannot activate until sensors recover.
        if (override_active && !sensor_ok) {
            ROS_WARN_THROTTLE(5.0,
                "[ACC/AEB] DRIVER OVERRIDE + SENSOR FAULT (%s): "
                "AEB / ADAS NON-FUNCTIONAL — driver has full authority. "
                "AEB CANNOT protect occupants until sensors recover. "
                "Restore sensor feed before relying on AEB.",
                faultSrcStr(fsrc));
        }

        // Driver override suspends ACC (CRUISE/FOLLOW) but AEB/WARN remain armed.
        // Because sm_sensor_ok keeps the FSM in CRUISE when sensors are stale + override active,
        // suppress_acc already covers all override cases — no separate suppress_fault needed.
        const bool suppress_acc = override_active &&
                                  (cr.state == State::CRUISE || cr.state == State::FOLLOW);
        if (suppress_acc) sm_.onDriverOverride();

        ctrl_msg_.header.stamp        = now;
        ctrl_msg_.acc_command_value   = suppress_acc ? 0.0 : cr.act.torque_nm;
        ctrl_msg_.brake_command_value = suppress_acc ? 0.0 : cr.act.brake_g;
        // steering already zeroed in initControlMsgs()
        pub_ctrl_.publish(ctrl_msg_);

        publishDiag(now, ego_v, snap, fsm_mio, fsm_kin, cr.accel_cmd, cr.act, fsrc,
                    adj_mio, adj_kin, cam_content_ok);
    }

    //--------------------------------------------------------------------------
    //  Watchdog thread: steady-clock freeze detection, single-shot + bounded
    //  re-assert safe-stop, and its own distinct diagnostic.
    //--------------------------------------------------------------------------
    void watchdogLoop() {
        const auto    poll        = std::chrono::milliseconds(consts::WATCHDOG_POLL_PERIOD_MS);
        const int64_t timeout_ns  = static_cast<int64_t>(p_.watchdog_timeout_s * 1e9);
        const int64_t refresh_ns  = static_cast<int64_t>(p_.dt_s * 1e9);   // re-assert at control rate
        bool          latched     = false;
        int64_t       last_pub_ns = 0;

        while (watchdog_active_.load(std::memory_order_relaxed)) {
            if (!loop_started_.load(std::memory_order_relaxed)) {
                std::this_thread::sleep_for(poll);   // no false trip before the loop's first tick
                continue;
            }

            const int64_t now_ns    = steadyNowNs();
            const int64_t frozen_ns  = now_ns - last_loop_steady_ns_.load(std::memory_order_relaxed);

            if (frozen_ns > timeout_ns) {
                const double frozen_s = static_cast<double>(frozen_ns) * 1e-9;
                if (!latched) {
                    latched = true;
                    watchdog_tripped_.store(true, std::memory_order_release);
                    const uint32_t n = watchdog_trip_count_.fetch_add(1, std::memory_order_relaxed) + 1;
                    ROS_FATAL("[ACC/AEB] WATCHDOG TIMEOUT (%.3f s): control loop frozen. "
                              "Engaging hardware safe-stop (trip #%u).", frozen_s, n);
                    publishWatchdogStop(frozen_s);
                    last_pub_ns = now_ns;
                } else if (now_ns - last_pub_ns >= refresh_ns) {
                    publishWatchdogStop(frozen_s);   // re-assert at control rate, not 50 Hz
                    last_pub_ns = now_ns;
                }
            } else if (latched) {
                latched = false;
                ROS_WARN("[ACC/AEB] WATCHDOG: control loop resumed after freeze.");
                publishWatchdogClear();
            }

            std::this_thread::sleep_for(poll);
        }
    }

    void stopWatchdog() noexcept {
        watchdog_active_.store(false, std::memory_order_relaxed);
        if (watchdog_thread_.joinable()) watchdog_thread_.join();
    }

    void stopControlSpinner() noexcept {
        if (ctrl_spinner_) ctrl_spinner_->stop();   // idempotent
    }

    //--------------------------------------------------------------------------
    //  Safe-stop publishers (each uses a thread-private pre-built message).
    //--------------------------------------------------------------------------
    void publishLoopSafeStop(const ros::Time& now) {
        loop_stop_msg_.header.stamp = now;
        pub_ctrl_.publish(loop_stop_msg_);
    }

    void publishWatchdogStop(double frozen_s) {
        const ros::Time stamp = ros::Time::now();
        wd_stop_msg_.header.stamp = stamp;
        pub_ctrl_.publish(wd_stop_msg_);

        auto& st = wd_diag_msg_.status[0];
        wd_diag_msg_.header.stamp = stamp;
        st.level   = diagnostic_msgs::DiagnosticStatus::ERROR;
        st.message = "Control loop frozen - hardware safe-stop engaged";
        setDiagValue(st.values[WD_FROZEN_S],   frozen_s);
        setDiagValue(st.values[WD_TRIP_COUNT], static_cast<double>(watchdog_trip_count_.load(std::memory_order_relaxed)));
        pub_diag_.publish(wd_diag_msg_);
    }

    void publishWatchdogClear() {
        auto& st = wd_diag_msg_.status[0];
        wd_diag_msg_.header.stamp = ros::Time::now();
        st.level   = diagnostic_msgs::DiagnosticStatus::OK;
        st.message = "Control loop resumed after freeze";
        setDiagValue(st.values[WD_FROZEN_S],   0.0);
        setDiagValue(st.values[WD_TRIP_COUNT], static_cast<double>(watchdog_trip_count_.load(std::memory_order_relaxed)));
        pub_diag_.publish(wd_diag_msg_);
    }

    //--------------------------------------------------------------------------
    //  Diagnostics — reuses the pre-allocated message; zero hot-path heap.
    //--------------------------------------------------------------------------
    static void setDiagValue(diagnostic_msgs::KeyValue& kv, double v) noexcept {
        char buf[24];
        std::snprintf(buf, sizeof(buf), "%.6f", v);   // matches std::to_string(double)
        kv.value.assign(buf);                         // reuses reserved capacity, no realloc
    }

    void publishDiag(const ros::Time& now, double ego_v, const PerceptionSnapshot& snap,
                     const MioResult& mio, const KinResult& kin, double cmd, const ActCmd& act,
                     FaultSrc fsrc,
                     const MioResult& adj_mio, const KinResult& adj_kin, bool cam_content_ok) {
        const State  s        = sm_.state();
        const bool   has_mio  = (mio.valid || mio.stale);
        const double ttc_disp = std::min(kin.ttc, consts::DIAG_TTC_DISPLAY_CAP);

        ROS_DEBUG_THROTTLE(1.0, "[%s] v=%.1fkm/h x=%.1fm close=%.1fm/s TTC=%.1fs cmd=%.2f T=%.0fNm B=%.2fG",
            toString(s), ego_v * MPS_2_KPH, has_mio ? mio.x : 0.0,
            kin.closing_spd, ttc_disp, cmd, act.torque_nm, act.brake_g);

        auto& st = diag_msg_.status[0];
        diag_msg_.header.stamp = now;
        st.message = toString(s);
        st.level = (s == State::FAULT || s == State::AEB) ? diagnostic_msgs::DiagnosticStatus::ERROR
                 : (s == State::WARN)                     ? diagnostic_msgs::DiagnosticStatus::WARN
                                                          : diagnostic_msgs::DiagnosticStatus::OK;

        setDiagValue(st.values[D_TARGET_KPH], dynamic_target_speed_mps_.load(std::memory_order_relaxed) * MPS_2_KPH);
        setDiagValue(st.values[D_EGO_KPH],    ego_v * MPS_2_KPH);
        setDiagValue(st.values[D_MIO_X],      has_mio ? mio.x : -1.0);
        setDiagValue(st.values[D_CLOSING],    kin.closing_spd);
        setDiagValue(st.values[D_TTC],        ttc_disp);
        setDiagValue(st.values[D_CMD],        cmd);
        setDiagValue(st.values[D_LANE_L],     snap.lane_l.valid ? 1.0 : 0.0);
        setDiagValue(st.values[D_LANE_R],     snap.lane_r.valid ? 1.0 : 0.0);
        setDiagValue(st.values[D_DYN_W],      snap.dynamic_half_w);
        setDiagValue(st.values[D_WD_TRIPS],   static_cast<double>(watchdog_trip_count_.load(std::memory_order_relaxed)));
        st.values[D_FAULT_SRC].value.assign(faultSrcStr(fsrc));
        setDiagValue(st.values[D_LOOP_OVERRUNS], static_cast<double>(loop_overrun_count_));
        setDiagValue(st.values[D_CAM_OK],      snap.cam_timeout   ? 0.0 : 1.0);
        setDiagValue(st.values[D_RAD_OK],      snap.radar_timeout ? 0.0 : 1.0);
        setDiagValue(st.values[D_OVERRIDE],    driver_override_active_.load(std::memory_order_relaxed) ? 1.0 : 0.0);
        setDiagValue(st.values[D_CAM_CONTENT], cam_content_ok ? 1.0 : 0.0);
        setDiagValue(st.values[D_ADJ_MIO_X],  adj_mio.valid ? adj_mio.x : -1.0);
        setDiagValue(st.values[D_ADJ_TTC],    adj_mio.valid
                                                ? std::min(adj_kin.ttc, consts::DIAG_TTC_DISPLAY_CAP)
                                                : consts::DIAG_TTC_DISPLAY_CAP);

        pub_diag_.publish(diag_msg_);
    }

    //--------------------------------------------------------------------------
    //  Members. (Declaration order: p_ precedes tracker_/sm_; ctrl_queue_
    //  precedes ctrl_timer_ precedes ctrl_spinner_ so teardown is spinner ->
    //  timer -> queue.)
    //--------------------------------------------------------------------------
    ros::NodeHandle& nh_;
    ros::NodeHandle& nh_priv_;
    const Params     p_;

    MioTracker   tracker_;
    StateMachine sm_;

    ros::Subscriber sub_vcu_, sub_objects_, sub_radar_, sub_lane_l_, sub_lane_r_,
                    sub_planner_speed_, sub_override_;
    ros::Publisher  pub_ctrl_, pub_diag_;

    // Dedicated, isolated control path.
    ros::CallbackQueue                  ctrl_queue_;
    ros::Timer                          ctrl_timer_;
    std::unique_ptr<ros::AsyncSpinner>  ctrl_spinner_;

    // Lock-free state shared with the watchdog thread.
    std::thread           watchdog_thread_;
    std::atomic<bool>     watchdog_active_{false};
    std::atomic<bool>     loop_started_{false};
    std::atomic<bool>     watchdog_tripped_{false};
    std::atomic<int64_t>  last_loop_steady_ns_{0};
    std::atomic<uint32_t> watchdog_trip_count_{0};
    double                safe_brake_g_       = 0.0;
    int64_t               overrun_threshold_ns_ = 0;

    // Control-loop-thread-only (no synchronization needed).
    double   debug_stall_once_s_  = 0.0;
    uint32_t loop_overrun_count_  = 0;
    bool     first_control_tick_  = true;
    bool     prev_override_state_ = false;
    State    last_state_          = State::FAULT;   // start FAULT; first real tick logs the transition

    // Perception buffers (perc_mutex_-guarded; written by sensor callbacks).
    std::mutex perc_mutex_;
    npust_bus_msgs::TrackedObjectArray::ConstPtr latest_objects_;
    npust_bus_msgs::TrackedObjectArray::ConstPtr latest_radar_objects_;
    npust_bus_msgs::LanePolynomial lane_l_raw_, lane_r_raw_;
    ros::Time last_obj_arrival_stamp_{ros::Time(0)};
    ros::Time last_radar_arrival_stamp_{ros::Time(0)};
    ros::Time last_lane_l_arrival_stamp_{ros::Time(0)};
    ros::Time last_lane_r_arrival_stamp_{ros::Time(0)};
    ros::Time last_cam_nonempty_stamp_{ros::Time(0)};  // wall time of last non-empty camera msg
    bool      obj_ever_received_{false};
    bool      radar_ever_received_{false};
    bool      lane_l_valid_{false}, lane_r_valid_{false};
    bool      cam_nonempty_ever_{false};               // true once camera sent ≥1 object

    // Atomics fed by callbacks, read by the control loop.
    std::atomic<double> ego_v_mps_{0.0};
    std::atomic<double> dynamic_target_speed_mps_{0.0};
    std::atomic<double> vcu_last_arrival_sec_{0.0};
    std::atomic<bool>   vcu_ever_received_{false};
    std::atomic<bool>   driver_override_active_{false};

    // Pre-allocated outbound messages (reused every publish -> no hot-path heap).
    pro_can::control_data            ctrl_msg_;        // control-loop thread
    pro_can::control_data            loop_stop_msg_;   // control-loop thread / main (shutdown)
    pro_can::control_data            wd_stop_msg_;     // watchdog thread
    diagnostic_msgs::DiagnosticArray diag_msg_;        // control-loop thread
    diagnostic_msgs::DiagnosticArray wd_diag_msg_;     // watchdog thread
};

} // namespace acc_aeb

int main(int argc, char** argv) {
    // Own SIGINT so we can assert a final safe-stop before tearing down.
    ros::init(argc, argv, "acc_aeb_controller", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh, nh_priv("~");

    acc_aeb::AccAebNode node(nh, nh_priv);   // also starts the control loop + watchdog

    std::signal(SIGINT,  acc_aeb::onShutdownSignal);
    std::signal(SIGTERM, acc_aeb::onShutdownSignal);

    // Sensor spinner on the GLOBAL queue (control loop has its own queue/thread).
    ros::AsyncSpinner sensor_spinner(static_cast<uint32_t>(std::max(1, node.params().spinner_threads)));
    sensor_spinner.start();

    // Idle on a WALL clock so shutdown works even if sim time is paused/stopped.
    ros::WallRate idle(50.0);
    while (ros::ok() && !acc_aeb::g_shutdown_requested.load(std::memory_order_relaxed)) {
        idle.sleep();
    }

    sensor_spinner.stop();             // quiesce sensor callbacks
    node.engageShutdownSafeStop();     // quiesce control loop + watchdog, assert final safe-stop
    ros::shutdown();
    return 0;
}
