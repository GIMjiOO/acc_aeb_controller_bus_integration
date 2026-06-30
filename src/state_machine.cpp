//==============================================================================
//  state_machine.cpp  —  FSM, control law, jerk limiting, actuator mapping.
//==============================================================================
#include "acc_aeb_controller/state_machine.h"

#include <algorithm>
#include <cmath>

namespace acc_aeb {

StateMachine::StateMachine(const Params& p)
    : p_(p),
      torque_per_accel_((p.mass_kg * p.wheel_r_m) / (p.gear_ratio * p.drv_eff))
{}

void StateMachine::onSafeStopEngaged(double decel_mps2, double brake_g) noexcept {
    // Full command-pipeline reset (see header for rationale). The node calls
    // this immediately after it publishes a hardware safe-stop so that the very
    // next control tick resumes from a braking posture instead of a stale one.
    last_acc_cmd_    = decel_mps2;       // jerk baseline reflects the commanded braking
    integral_cruise_ = 0.0;              // clear PI wind-up
    integral_follow_ = 0.0;
    act_mode_        = ActMode::BRAKE;   // we are braking; do not coast on resume
    coast_timer_     = 0.0;
    last_pub_torque_ = 0.0;              // rate-limit baselines match what was published
    last_pub_brake_  = brake_g;
}

void StateMachine::onDriverOverride() noexcept {
    integral_cruise_ = 0.0;
    integral_follow_ = 0.0;
    act_mode_        = ActMode::COAST;
    coast_timer_     = 0.0;
    last_pub_torque_ = 0.0;
    last_pub_brake_  = 0.0;
    // last_acc_cmd_ intentionally untouched: jerk-limit from current level on resume
}

void StateMachine::reset() noexcept {
    state_                = State::CRUISE;
    act_mode_             = ActMode::COAST;
    last_acc_cmd_         = 0.0;
    integral_cruise_      = 0.0;
    integral_follow_      = 0.0;
    aeb_hold_timer_       = 0.0;
    fault_recovery_timer_ = 0.0;
    coast_timer_          = 0.0;
    last_pub_torque_      = 0.0;
    last_pub_brake_       = 0.0;
    aeb_trigger_count_    = 0;
    aeb_hold_active_      = false;
    recovered_this_step_  = false;
}

ControlResult StateMachine::step(const MioResult& mio, const KinResult& kin,
                                 double ego_v, double dt, bool sensor_ok, double target_speed) {
    ControlResult r;
    recovered_this_step_ = false;
    SmEvent ev = SmEvent::NONE;

    // Sensor-loss fault entry (was done node-side before the FSM update in v15.0).
    if (!sensor_ok && state_ != State::FAULT) {
        state_                = State::FAULT;
        fault_recovery_timer_ = 0.0;
        ev = SmEvent::ENTERED_FAULT;
    }

    // prev_state is captured AFTER the fault-entry mutation, matching v15.0 so
    // the integral-reset-on-transition logic in computeCommand() is identical.
    const State prev_state = state_;
    state_ = updateStateMachine(state_, mio, kin, ego_v, dt, sensor_ok);
    if (ev == SmEvent::NONE && recovered_this_step_) ev = SmEvent::FAULT_RECOVERED;

    const double cmd_raw  = computeCommand(state_, prev_state, ego_v, mio, kin, dt, target_speed);
    const double cmd_jerk = applyJerkLimit(cmd_raw, state_, ego_v, dt);

    r.state     = state_;
    r.accel_cmd = cmd_jerk;
    r.event     = ev;

    // NaN guard mirrors v15.0 ordering: never feed a bad command into the
    // actuator rate-limit baseline. The node detects the non-finite accel and
    // issues a safe stop instead of publishing r.act.
    if (!std::isfinite(cmd_jerk)) return r;

    r.act          = mapToActuators(cmd_jerk, state_, dt);
    r.target_speed = std::max(0.0, ego_v + cmd_jerk * dt);
    return r;
}

State StateMachine::evaluateOperationalState(State current_state, const MioResult& mio, const KinResult& kin) {
    if (mio.stale) {
        aeb_trigger_count_ = 0;
        return current_state;
    }
    if (!mio.valid) {
        aeb_trigger_count_ = 0;
        return State::CRUISE;
    }
    if (mio.x < p_.aeb_immediate_dist_m) {
        aeb_trigger_count_ = p_.aeb_confirm_frames;
        return State::AEB;
    }
    if (kin.ttc < p_.ttc_aeb_s || mio.x < p_.aeb_dist_m) {
        aeb_trigger_count_++;
        if (aeb_trigger_count_ >= p_.aeb_confirm_frames) return State::AEB;
        return State::WARN;
    }

    aeb_trigger_count_ = 0;
    if (kin.ttc < p_.ttc_warn_s) return State::WARN;
    if (current_state == State::WARN && kin.ttc < p_.ttc_warn_exit_s) return State::WARN;

    if (mio.x <= kin.target_dist * p_.follow_enter_ratio) return State::FOLLOW;
    if (current_state == State::FOLLOW && mio.x <= kin.target_dist * p_.follow_exit_ratio) return State::FOLLOW;

    return State::CRUISE;
}

State StateMachine::updateStateMachine(State cur, const MioResult& mio, const KinResult& kin,
                                       double ego_v, double dt, bool sensor_ok) {
    // FAULT must precede AEB hold: sensors failing while aeb_hold_active_=true
    // would otherwise cause FAULT<->AEB oscillation (hold returns AEB, step()
    // resets to FAULT, repeats). Clearing the hold here breaks the cycle.
    if (cur == State::FAULT) {
        aeb_hold_active_   = false;   // sensor loss overrides any pending AEB hold
        aeb_trigger_count_ = 0;
        if (sensor_ok) {
            fault_recovery_timer_ += dt;
            if (fault_recovery_timer_ >= p_.fault_recovery_s) {
                fault_recovery_timer_ = 0.0;
                recovered_this_step_  = true;     // node logs "Sensor recovered"
                return evaluateOperationalState(cur, mio, kin);
            }
        } else {
            fault_recovery_timer_ = 0.0;
        }
        return State::FAULT;
    }

    if (aeb_hold_active_) {
        aeb_trigger_count_ = 0;
        aeb_hold_timer_ -= dt;
        if (aeb_hold_timer_ <= 0.0 && (!mio.valid || mio.x > p_.resume_dist_m)) {
            aeb_hold_active_ = false;
            return evaluateOperationalState(cur, mio, kin);   // bypass re-arm block
        } else {
            return State::AEB;
        }
    }

    if (cur == State::AEB) {
        if (ego_v < p_.stop_speed_mps) {
            aeb_trigger_count_ = 0;
            aeb_hold_active_   = true;
            aeb_hold_timer_    = p_.aeb_hold_s;
        }
        return State::AEB;
    }

    return evaluateOperationalState(cur, mio, kin);
}

double StateMachine::computeCommand(State s, State prev_state, double ego_v,
                                    const MioResult& mio, const KinResult& kin,
                                    double dt, double target_speed) {
    if (s != prev_state) {
        integral_cruise_ = 0.0;
        integral_follow_ = 0.0;
    }
    if (!std::isfinite(target_speed)) target_speed = 0.0;

    double cmd = 0.0;
    switch (s) {
    case State::FAULT:
        cmd = p_.a_fault_brake;
        break;
    case State::AEB:
        cmd = p_.a_aeb_max;
        break;
    case State::WARN:
        cmd = p_.a_warn_brake;
        break;
    case State::FOLLOW: {
        if (mio.stale) {
            cmd = std::min(p_.loss_hold_decel, 0.0);
            break;
        }
        const double dist_err = mio.x - kin.target_dist;
        const double p_term = p_.kp_follow_dist * dist_err
                            + p_.kd_follow_vrel * mio.v_rel
                            + p_.kf_follow_arel * mio.a_rel;
        const double raw_cmd = p_term + p_.ki_follow * integral_follow_;

        const bool saturated_accel = (raw_cmd >= p_.a_acc_max    && dist_err > 0.0);
        const bool saturated_brake = (raw_cmd <= p_.a_soft_brake && dist_err < 0.0);
        if (!saturated_accel && !saturated_brake) {
            integral_follow_ += dist_err * dt;
            integral_follow_  = clampVal(integral_follow_, -p_.follow_iclamp, p_.follow_iclamp);
        }
        cmd = clampVal(p_term + p_.ki_follow * integral_follow_, p_.a_soft_brake, p_.a_acc_max);
        break;
    }
    case State::CRUISE: {
        const double v_err = target_speed - ego_v;
        const double p_term = p_.kp_cruise * v_err;
        const double raw_cmd = p_term + p_.ki_cruise * integral_cruise_;

        const bool saturated_accel = (raw_cmd >= p_.a_acc_max    && v_err > 0.0);
        const bool saturated_brake = (raw_cmd <= p_.a_soft_brake && v_err < 0.0);
        if (!saturated_accel && !saturated_brake) {
            integral_cruise_ += v_err * dt;
            integral_cruise_  = clampVal(integral_cruise_, -p_.cruise_iclamp, p_.cruise_iclamp);
        }
        cmd = clampVal(p_term + p_.ki_cruise * integral_cruise_, p_.a_soft_brake, p_.a_acc_max);
        break;
    }
    }
    return cmd;
}

double StateMachine::applyJerkLimit(double cmd, State s, double ego_v, double dt) {
    if (s == State::AEB) {
        last_acc_cmd_ = cmd;
        return cmd;
    }
    const double comfort_jerk = (ego_v < p_.low_speed_threshold_mps) ? p_.jerk_low_speed : p_.jerk_normal;
    const double jerk_decel   = (s == State::WARN || s == State::FAULT) ? p_.jerk_warn : comfort_jerk;

    cmd = clampVal(cmd, last_acc_cmd_ - jerk_decel * dt, last_acc_cmd_ + comfort_jerk * dt);
    last_acc_cmd_ = cmd;
    return cmd;
}

void StateMachine::updateActuatorMode(double cmd, State s, double dt) {
    if (s == State::AEB) {
        act_mode_    = ActMode::BRAKE;
        coast_timer_ = 0.0;
        return;
    }
    const double min_coast = (s == State::WARN || s == State::FAULT) ? p_.min_coast_s_warn
                                                                     : p_.min_coast_s_normal;
    switch (act_mode_) {
    case ActMode::THROTTLE:
        if (cmd < p_.throttle_dead) { act_mode_ = ActMode::COAST; coast_timer_ = 0.0; }
        break;
    case ActMode::COAST:
        coast_timer_ += dt;
        if (coast_timer_ >= min_coast) {
            if      (cmd >= p_.throttle_enter) act_mode_ = ActMode::THROTTLE;
            else if (cmd <= p_.brake_enter)    act_mode_ = ActMode::BRAKE;
        }
        break;
    case ActMode::BRAKE:
        if (cmd > p_.brake_dead) { act_mode_ = ActMode::COAST; coast_timer_ = 0.0; }
        break;
    }
}

ActCmd StateMachine::mapToActuators(double cmd, State s, double dt) {
    updateActuatorMode(cmd, s, dt);

    ActCmd out;
    if (act_mode_ == ActMode::THROTTLE) {
        out.torque_nm = clampVal(torque_per_accel_ * cmd, 0.0, p_.torque_max);
    } else if (act_mode_ == ActMode::BRAKE) {
        out.brake_g = clampVal(std::abs(cmd) * INV_G_MPS2, 0.0, p_.brake_g_max);
    }

    if (s != State::AEB) {
        const double max_torque_step = p_.torque_rate_limit * dt;
        const double max_brake_step  = p_.brake_rate_limit * dt;
        out.torque_nm = clampVal(out.torque_nm, last_pub_torque_ - max_torque_step, last_pub_torque_ + max_torque_step);
        out.brake_g   = clampVal(out.brake_g,   last_pub_brake_  - max_brake_step,  last_pub_brake_  + max_brake_step);
    }

    last_pub_torque_ = out.torque_nm;
    last_pub_brake_  = out.brake_g;
    return out;
}

} // namespace acc_aeb
