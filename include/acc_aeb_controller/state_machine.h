#pragma once
//==============================================================================
//  state_machine.h  —  Longitudinal decision + command generation.
//
//  Encapsulates everything between "here is the tracked threat" and "here is
//  the actuator command": the CRUISE/FOLLOW/WARN/AEB/FAULT state machine, the
//  PID + feedforward control law, jerk limiting, and the accel->torque/brake
//  actuator mapping with rate limits. ROS-free and self-contained — all of its
//  state lives here, so the node never reaches into control internals.
//==============================================================================
#include "acc_aeb_controller/types.h"
#include "acc_aeb_controller/math_utils.h"

namespace acc_aeb {

// Notable transitions surfaced to the node for logging, so this module stays
// free of ROS console calls.
enum class SmEvent : uint8_t { NONE, ENTERED_FAULT, FAULT_RECOVERED };

struct ControlResult {
    ActCmd  act{};                      // actuator command (valid only if accel_cmd is finite)
    double  accel_cmd     = 0.0;        // jerk-limited longitudinal accel (NaN => node safe-stops)
    double  target_speed  = 0.0;        // m/s — integrated from accel_cmd; published to /control_value
    State   state         = State::CRUISE;
    SmEvent event         = SmEvent::NONE;
};

class StateMachine {
public:
    explicit StateMachine(const Params& p);

    // One control tick. `sensor_ok` is false when the object tracker is stale;
    // `target_speed` is the (already capped) cruise setpoint in m/s.
    [[nodiscard]] ControlResult step(const MioResult& mio, const KinResult& kin,
                                     double ego_v, double dt, bool sensor_ok, double target_speed);

    // Force the command pipeline into a state consistent with an EXTERNALLY
    // commanded full safe-stop (the node has just published 0 torque + `brake_g`
    // braking at `decel_mps2`). This is a COMPLETE reset of the longitudinal
    // command pipeline, not just the rate-limit baseline:
    //   * jerk baseline (last_acc_cmd_) <- decel_mps2  -- so the next tick is NOT
    //     jerk-limited upward away from braking (this was a latent bug: a stale
    //     positive last_acc_cmd_ could delay post-fault braking by ~1 s),
    //   * integrators                   <- 0           -- no wind-up carried over,
    //   * actuator mode                 <- BRAKE       -- resume braking, no coast,
    //   * coast timer                   <- 0,
    //   * rate-limit baselines          <- (0 torque, brake_g).
    // It intentionally does NOT touch the FSM state_; the next step() re-evaluates
    // the situation from fresh perception. (Renamed/expanded from onActuatorReset.)
    void onSafeStopEngaged(double decel_mps2, double brake_g) noexcept;

    // Soft reset when the driver takes manual control (ACC suspended, AEB remains
    // armed). Clears integrators and rate-limit baselines without entering BRAKE
    // mode so the next ACC tick resumes smoothly from zero.
    void onDriverOverride() noexcept;

    void  reset() noexcept;
    [[nodiscard]] State state() const noexcept { return state_; }

private:
    [[nodiscard]] State  evaluateOperationalState(State cur, const MioResult& mio, const KinResult& kin);
    [[nodiscard]] State  updateStateMachine(State cur, const MioResult& mio, const KinResult& kin,
                                            double ego_v, double dt, bool sensor_ok);
    [[nodiscard]] double computeCommand(State s, State prev, double ego_v,
                                        const MioResult& mio, const KinResult& kin,
                                        double dt, double target_speed);
    [[nodiscard]] double applyJerkLimit(double cmd, State s, double ego_v, double dt);
    void                 updateActuatorMode(double cmd, State s, double dt);
    [[nodiscard]] ActCmd mapToActuators(double cmd, State s, double dt);

    const Params& p_;

    // Derived constant: accel -> wheel torque gain, folded once so the hot path
    // is a single multiply instead of mul+mul+div every cycle.
    const double torque_per_accel_;     // (mass * wheel_r) / (gear_ratio * drv_eff)

    // --- control state ---
    State   state_                = State::CRUISE;
    ActMode act_mode_             = ActMode::COAST;
    double  last_acc_cmd_         = 0.0;
    double  integral_cruise_      = 0.0;
    double  integral_follow_      = 0.0;
    double  aeb_hold_timer_       = 0.0;
    double  fault_recovery_timer_ = 0.0;
    double  coast_timer_          = 0.0;
    double  last_pub_torque_      = 0.0;
    double  last_pub_brake_       = 0.0;
    int     aeb_trigger_count_    = 0;
    bool    aeb_hold_active_      = false;
    bool    recovered_this_step_  = false;   // set by updateStateMachine on FAULT->ops recovery
};

} // namespace acc_aeb
