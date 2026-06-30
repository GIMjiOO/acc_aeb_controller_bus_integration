#pragma once
//==============================================================================
//  types.h  —  Core POD data types and tuning constants (ROS-free).
//
//  No roscpp dependency — kinematics and state_machine can be unit-tested
//  without a ROS master. Perception-coupled aggregates live in kinematics.h.
//  Members are declared widest-first (doubles → int → bool): zero padding.
//==============================================================================
#include <cstdint>
#include <limits>

namespace acc_aeb {

// --- Physical / unit constants -----------------------------------------------
constexpr double G_MPS2     = 9.80665;
constexpr double KPH_2_MPS  = 1.0 / 3.6;
constexpr double MPS_2_KPH  = 3.6;
constexpr double INV_G_MPS2 = 1.0 / G_MPS2;                    // a/g  ->  a * INV_G_MPS2
constexpr double INF_D      = std::numeric_limits<double>::max();

// --- Fixed algorithm constants (not ROS-tunable) -----------------------------
namespace consts {
    constexpr double KF_A_REL_CLAMP_MPS2   = 15.0;   // |a_rel| ceiling after KF update
    constexpr double GRACE_A_CLOSING_CLAMP =  2.0;   // |a| ceiling during dead-reckon coast
    constexpr double KIN_MIN_A_REL_MPS2    =  0.1;   // below this, skip the quadratic TTC path
    constexpr double MIO_SCORE_EPS         =  0.1;   // denominator floor in the threat score
    constexpr double BACKSTOP_RANGE_TIME_S =  4.0;   // stationary-target backstop look-ahead
    constexpr double TTC_CLOSING_EPS       =  0.01;  // min closing speed for a finite TTC
    constexpr double SAFE_DIV_EPS          =  1e-6;  // guard for the generic safeDiv()
    constexpr double DIAG_TTC_DISPLAY_CAP  = 99.9;   // cosmetic cap for reported TTC
    constexpr int    WATCHDOG_POLL_PERIOD_MS = 20;   // watchdog steady-clock poll period (50 Hz)
    constexpr double LOOP_OVERRUN_FACTOR   =  1.5;   // wall period > factor*dt_s counts as an overrun

    // Variance fallbacks used when a publisher leaves x_var/vx_var at 0.0.
    // Zero variance means infinite confidence — breaks the KF weight calculation.
    constexpr double CAM_X_VAR_DEFAULT  = 1.00;   // camera depth is noisy (m²)
    constexpr double CAM_VX_VAR_DEFAULT = 2.00;   // camera velocity is optical-flow derived (m²/s²)
    constexpr double RAD_X_VAR_DEFAULT  = 0.10;   // radar range is accurate (m²)
    constexpr double RAD_VX_VAR_DEFAULT = 0.05;   // radar Doppler is very accurate (m²/s²)
}

// --- ROS parameter defaults ---------------------------------------------------
namespace defaults {
    constexpr double DT_S                 = 0.05;
    constexpr double TARGET_KPH           = 25.0;
    constexpr double TIME_GAP_S           = 2.0;
    constexpr double SAFE_BASE_M          = 5.0;
    constexpr double SYSTEM_LATENCY_S     = 0.5;
    constexpr bool   VX_IS_RELATIVE       = false;
    constexpr double AEB_BACKSTOP_BASE_M  = 20.0;
    constexpr double TTC_AEB_S            = 1.5;
    constexpr double TTC_WARN_S           = 3.0;
    constexpr double TTC_WARN_EXIT_S      = 3.8;
    constexpr double AEB_DISTANCE_M       = 8.0;
    constexpr double AEB_IMMEDIATE_DIST_M = 4.0;
    constexpr double A_ACC_MAX            =  1.0;
    constexpr double A_SOFT_BRAKE         = -1.5;
    constexpr double A_WARN_BRAKE         = -2.5;
    constexpr double A_FAULT_BRAKE        = -2.5;
    constexpr double A_AEB_MAX            = -5.5;
    constexpr double JERK_NORMAL          =  2.0;
    constexpr double JERK_LOW_SPEED       =  1.0;
    constexpr double LOW_SPEED_KPH        = 20.0;
    constexpr double JERK_WARN            =  4.0;
    constexpr double THROTTLE_ENTER_MPS2  =  0.15;
    constexpr double THROTTLE_DEAD_MPS2   =  0.05;
    constexpr double BRAKE_ENTER_MPS2     = -0.15;
    constexpr double BRAKE_DEAD_MPS2      = -0.05;
    constexpr double MIN_COAST_S_NORMAL   = 0.30;
    constexpr double MIN_COAST_S_WARN     = 0.10;
    constexpr double SENSOR_TIMEOUT_S     = 0.30;
    constexpr double VCU_TIMEOUT_S        = 0.30;    // ego-speed (own-velocity) staleness limit
    constexpr double FAULT_RECOVERY_S     = 1.0;
    constexpr double AEB_HOLD_S           = 2.0;
    constexpr double STOP_SPEED_MPS       = 0.30;
    constexpr double RESUME_DIST_M        = 12.0;
    constexpr double LOSS_HOLD_DECEL      = -0.5;
    constexpr int    CONFIRM_FRAMES       = 3;
    constexpr int    AEB_CONFIRM_FRAMES   = 2;



    constexpr double Q_X = 0.5;    // process noise scalar (m²/s⁵)
    constexpr double Q_V = 0.15;   // process noise scalar for velocity (m²/s³)
    constexpr double Q_A = 0.05;   // process noise scalar for acceleration (m²/s)
    constexpr double FUSION_GATE_M = 3.0; // max cam/rad range difference to allow joint KF update (m)
    constexpr double MIO_WEIGHT_DIST      = 10.0;
    constexpr double MIO_WEIGHT_TTC       = 5.0;
    constexpr double VEHICLE_MASS_KG      = 10000.0;
    constexpr double WHEEL_RADIUS_M       = 0.478;
    constexpr double GEAR_RATIO           = 6.5;
    constexpr double DRIVETRAIN_EFF       = 0.92;
    constexpr double MOTOR_TORQUE_MAX     = 1200.0;
    constexpr double BRAKE_G_MAX          = 0.60;

    constexpr double TORQUE_RATE_LIMIT_NM_S = 2000.0;
    constexpr double BRAKE_RATE_LIMIT_G_S   = 2.0;
    constexpr double KP_FOLLOW_DIST       =  0.12;
    constexpr double KD_FOLLOW_VREL       =  0.45;
    constexpr double KI_FOLLOW            =  0.005;
    constexpr double KF_FOLLOW_AREL       =  0.20;  // acceleration feedforward
    constexpr double FOLLOW_ICLAMP        =  5.0;
    constexpr double KP_CRUISE            =  0.50;
    constexpr double KI_CRUISE            =  0.01;
    constexpr double CRUISE_ICLAMP        =  3.0;
    constexpr double MAX_RANGE_M          = 120.0;
    constexpr double STATIC_HALF_W_M      =   1.75;
    constexpr double MAX_LANE_WIDTH_M     =   5.00;
    constexpr double MAX_LANE_CURVE_A     =   0.02;
    constexpr double MAX_LANE_HEADING_B   =   0.35;
    constexpr double TARGET_LOSS_TIMEOUT_S = 1.0;
    constexpr double MIN_VALID_X_M        = 0.5;
    constexpr double STATIONARY_V_ABS_MAX = 0.5;    // strictly parked objects
    constexpr double MIN_CLOSING_SPEED_MPS = 0.01;
    constexpr double LANE_NARROW_RATE     = 0.02;
    constexpr double MIN_STATIC_HALF_W_M  = 1.0;

    constexpr double FOLLOW_ENTER_RATIO   = 1.5;
    constexpr double FOLLOW_EXIT_RATIO    = 1.8;
    constexpr double MIN_STALE_X_M        = 1.0;
    constexpr double V_REL_PLAUSIBLE_MAX  = 70.0;
    constexpr double EGO_V_MOVING_THRES   = 0.1;

    constexpr double WATCHDOG_TIMEOUT_S    = 0.20;

    constexpr double RADAR_TIMEOUT_S       = 0.30;  // radar staleness → degraded (cam still OK)
    constexpr double RADAR_FUSION_DEDUP_M  = 2.0;   // objects within this XY dist = same obstacle
    constexpr bool   RADAR_VX_IS_RELATIVE  = true;  // Doppler vx is ego-relative

    constexpr int    SPINNER_THREADS      = 2;      // ros::AsyncSpinner worker count

    // Camera content-health: warn if camera sends empty arrays while ego is moving
    // for longer than this. Not a FAULT (clear road is valid); diagnostic only.
    constexpr double CAM_HEALTH_TIMEOUT_S  = 30.0;

    // Adjacent-lane cut-in zone: objects between 1× and this factor × dynamic_half_w
    // are tracked as potential cut-in threats and exposed in diagnostics.
    constexpr double CUT_IN_LATERAL_FACTOR = 2.0;
}

// --- Enumerations -------------------------------------------------------------
enum class ActMode : uint8_t { THROTTLE, COAST, BRAKE };
enum class State   : uint8_t { CRUISE, FOLLOW, WARN, AEB, FAULT };

[[nodiscard]] inline const char* toString(State s) noexcept {
    switch (s) {
        case State::CRUISE: return "CRUISE";
        case State::FOLLOW: return "FOLLOW";
        case State::WARN:   return "WARN";
        case State::AEB:    return "AEB";
        case State::FAULT:  return "FAULT";
    }
    return "UNKNOWN";
}

//==============================================================================
//  Parameter block. All doubles first, then ints, then the single bool so the
//  struct carries no padding (was ~8 bytes of holes around the mid-struct bool
//  and int pair in the v15.0 layout).
//==============================================================================
struct Params {
    double dt_s, target_speed_mps;
    double time_gap_s, safe_base_m, latency_s;
    double aeb_backstop_base_m;
    double ttc_aeb_s, ttc_warn_s, ttc_warn_exit_s, aeb_dist_m, aeb_immediate_dist_m;
    double a_acc_max, a_soft_brake, a_warn_brake, a_fault_brake, a_aeb_max;
    double jerk_normal, jerk_low_speed, low_speed_threshold_mps, jerk_warn;
    double throttle_enter, throttle_dead, brake_enter, brake_dead;
    double torque_rate_limit, brake_rate_limit;
    double min_coast_s_normal, min_coast_s_warn;
    double sensor_timeout_s, vcu_timeout_s, fault_recovery_s;
    double aeb_hold_s, stop_speed_mps, resume_dist_m;
    double loss_hold_decel;

    double mio_weight_dist, mio_weight_ttc;
    double mass_kg, wheel_r_m, gear_ratio, drv_eff, torque_max, brake_g_max;
    double kp_follow_dist, kd_follow_vrel, ki_follow, kf_follow_arel, follow_iclamp;
    double kp_cruise, ki_cruise, cruise_iclamp;
    double max_range_m, static_half_w_m, max_lane_width_m, max_lane_curve_a, max_lane_heading_b;
    double target_loss_timeout_s;
    double min_valid_x_m, stationary_v_abs_max, min_closing_speed_mps, lane_narrow_rate, min_static_half_w_m;
    double follow_enter_ratio, follow_exit_ratio, min_stale_x_m, v_rel_plausible_max, ego_v_moving_thres;
    double watchdog_timeout_s;
    double radar_timeout_s, radar_fusion_dedup_m;
    double cam_health_timeout_s;   // warn threshold: camera empty while moving
    double cut_in_lateral_factor;  // adjacent-zone outer edge = factor × dynamic_half_w
    double q_x;                    // KF process noise scalar (m²/s⁵), tune 0.1–2.0
    double q_v;                    // KF process noise scalar for velocity (m²/s³)
    double q_a;                    // KF process noise scalar for acceleration (m²/s)
    double fusion_gate_m;          // max |cam.x - rad.x| to allow joint KF update (m)

    int    confirm_frames, aeb_confirm_frames, spinner_threads;

    bool   vx_is_relative;
    bool   radar_vx_is_relative;
};

//==============================================================================
//  Most-Important-Object result. Doubles, then the 4-byte id, then the bools.
//==============================================================================
struct MioResult {
    double  x            = INF_D;
    double  v_abs        = 0.0;
    double  v_rel        = 0.0;
    double  a_rel        = 0.0;
    double  threat_score = 0.0;
    double  x_var        = 1.0;   // measurement variance on x  (m²)
    double  vx_var       = 1.0;   // measurement variance on vx (m²/s²)
    int32_t id           = -1;
    bool    valid        = false;
    bool    stale        = false;
};

//==============================================================================
//  Kalman filter state for one tracked object (longitudinal axis only).
//  P is stored as upper-triangular: P[0]=Pxx P[1]=Pxv P[2]=Pxa
//                                   P[3]=Pvv P[4]=Pva P[5]=Paa
//==============================================================================
struct KFState {
    double x = 0.0;   // range (m)
    double v = 0.0;   // relative velocity (m/s)
    double a = 0.0;   // relative acceleration (m/s²)
    double P[6] = {10.0, 0.0, 0.0,   // initial covariance — high uncertainty
                    5.0, 0.0,
                    2.0};
    bool   initialised = false;
};

struct KinResult {
    double ttc         = INF_D;
    double target_dist = 0.0;
    double closing_spd = 0.0;
};

struct ActCmd {
    double torque_nm = 0.0;
    double brake_g   = 0.0;
};

} // namespace acc_aeb
