//==============================================================================
//  params.cpp  —  Implementation of parameter loading + validation.
//==============================================================================
#include "acc_aeb_controller/params.h"

#include <cmath>
#include <cstdlib>
#include <string>

namespace acc_aeb {

Params loadParams(ros::NodeHandle& nh) {
    Params p;
    auto g  = [&](const std::string& k, double d) -> double { double v; nh.param(k, v, d); return v; };
    auto gb = [&](const std::string& k, bool   d) -> bool   { bool   v; nh.param(k, v, d); return v; };
    auto gi = [&](const std::string& k, int    d) -> int    { int    v; nh.param(k, v, d); return v; };
    using namespace defaults;

    p.dt_s                    = g("dt_s",                  DT_S);
    p.target_speed_mps        = g("target_kph",            TARGET_KPH) * KPH_2_MPS;
    p.time_gap_s              = g("time_gap_s",            TIME_GAP_S);
    p.safe_base_m             = g("safe_base_m",           SAFE_BASE_M);
    p.latency_s               = g("latency_s",             SYSTEM_LATENCY_S);
    p.vx_is_relative          = gb("vx_is_relative",       VX_IS_RELATIVE);
    p.aeb_backstop_base_m     = g("aeb_backstop_base_m",   AEB_BACKSTOP_BASE_M);
    p.ttc_aeb_s               = g("ttc_aeb_s",             TTC_AEB_S);
    p.ttc_warn_s              = g("ttc_warn_s",            TTC_WARN_S);
    p.ttc_warn_exit_s         = g("ttc_warn_exit_s",       TTC_WARN_EXIT_S);
    p.aeb_dist_m              = g("aeb_dist_m",            AEB_DISTANCE_M);
    p.aeb_immediate_dist_m    = g("aeb_immediate_dist_m",  AEB_IMMEDIATE_DIST_M);
    p.a_acc_max               = g("a_acc_max",             A_ACC_MAX);
    p.a_soft_brake            = g("a_soft_brake",          A_SOFT_BRAKE);
    p.a_warn_brake            = g("a_warn_brake",          A_WARN_BRAKE);
    p.a_fault_brake           = g("a_fault_brake",         A_FAULT_BRAKE);
    p.a_aeb_max               = g("a_aeb_max",             A_AEB_MAX);
    p.jerk_normal             = g("jerk_normal",           JERK_NORMAL);
    p.jerk_low_speed          = g("jerk_low_speed",        JERK_LOW_SPEED);
    p.low_speed_threshold_mps = g("low_speed_kph",         LOW_SPEED_KPH) * KPH_2_MPS;
    p.jerk_warn               = g("jerk_warn",             JERK_WARN);
    p.throttle_enter          = g("throttle_enter_mps2",   THROTTLE_ENTER_MPS2);
    p.throttle_dead           = g("throttle_dead_mps2",    THROTTLE_DEAD_MPS2);
    p.brake_enter             = g("brake_enter_mps2",      BRAKE_ENTER_MPS2);
    p.brake_dead              = g("brake_dead_mps2",       BRAKE_DEAD_MPS2);
    p.torque_rate_limit       = g("torque_rate_limit",     TORQUE_RATE_LIMIT_NM_S);
    p.brake_rate_limit        = g("brake_rate_limit",      BRAKE_RATE_LIMIT_G_S);
    p.min_coast_s_normal      = g("min_coast_s_normal",    MIN_COAST_S_NORMAL);
    p.min_coast_s_warn        = g("min_coast_s_warn",      MIN_COAST_S_WARN);
    p.sensor_timeout_s        = g("sensor_timeout_s",      SENSOR_TIMEOUT_S);
    p.vcu_timeout_s           = g("vcu_timeout_s",         VCU_TIMEOUT_S);
    p.fault_recovery_s        = g("fault_recovery_s",      FAULT_RECOVERY_S);
    p.aeb_hold_s              = g("aeb_hold_s",            AEB_HOLD_S);
    p.stop_speed_mps          = g("stop_speed_mps",        STOP_SPEED_MPS);
    p.resume_dist_m           = g("resume_dist_m",         RESUME_DIST_M);
    p.loss_hold_decel         = g("loss_hold_decel",       LOSS_HOLD_DECEL);
    p.confirm_frames          = gi("confirm_frames",       CONFIRM_FRAMES);
    p.aeb_confirm_frames      = gi("aeb_confirm_frames",   AEB_CONFIRM_FRAMES);
    p.spinner_threads         = gi("spinner_threads",      SPINNER_THREADS);

    p.mio_weight_dist         = g("mio_weight_dist",       MIO_WEIGHT_DIST);
    p.mio_weight_ttc          = g("mio_weight_ttc",        MIO_WEIGHT_TTC);
    p.mass_kg                 = g("mass_kg",               VEHICLE_MASS_KG);
    p.wheel_r_m               = g("wheel_r_m",             WHEEL_RADIUS_M);
    p.gear_ratio              = g("gear_ratio",            GEAR_RATIO);
    p.drv_eff                 = g("drivetrain_eff",        DRIVETRAIN_EFF);
    p.torque_max              = g("motor_torque_max",      MOTOR_TORQUE_MAX);
    p.brake_g_max             = g("brake_g_max",           BRAKE_G_MAX);
    p.kp_follow_dist          = g("kp_follow_dist",        KP_FOLLOW_DIST);
    p.kd_follow_vrel          = g("kd_follow_vrel",        KD_FOLLOW_VREL);
    p.ki_follow               = g("ki_follow",             KI_FOLLOW);
    p.kf_follow_arel          = g("kf_follow_arel",        KF_FOLLOW_AREL);
    p.follow_iclamp           = g("follow_iclamp",         FOLLOW_ICLAMP);
    p.kp_cruise               = g("kp_cruise",             KP_CRUISE);
    p.ki_cruise               = g("ki_cruise",             KI_CRUISE);
    p.cruise_iclamp           = g("cruise_iclamp",         CRUISE_ICLAMP);
    p.max_range_m             = g("max_range_m",           MAX_RANGE_M);
    p.static_half_w_m         = g("static_half_w_m",       STATIC_HALF_W_M);
    p.max_lane_width_m        = g("max_lane_width_m",      MAX_LANE_WIDTH_M);
    p.max_lane_curve_a        = g("max_lane_curve_a",      MAX_LANE_CURVE_A);
    p.max_lane_heading_b      = g("max_lane_heading_b",    MAX_LANE_HEADING_B);
    p.target_loss_timeout_s   = g("target_loss_timeout_s", TARGET_LOSS_TIMEOUT_S);
    p.min_valid_x_m           = g("min_valid_x_m",         MIN_VALID_X_M);
    p.stationary_v_abs_max    = g("stationary_v_abs_max",  STATIONARY_V_ABS_MAX);
    p.min_closing_speed_mps   = g("min_closing_speed_mps", MIN_CLOSING_SPEED_MPS);
    p.lane_narrow_rate        = g("lane_narrow_rate",      LANE_NARROW_RATE);
    p.min_static_half_w_m     = g("min_static_half_w_m",   MIN_STATIC_HALF_W_M);
    p.follow_enter_ratio      = g("follow_enter_ratio",    FOLLOW_ENTER_RATIO);
    p.follow_exit_ratio       = g("follow_exit_ratio",     FOLLOW_EXIT_RATIO);
    p.min_stale_x_m           = g("min_stale_x_m",         MIN_STALE_X_M);
    p.v_rel_plausible_max     = g("v_rel_plausible_max",   V_REL_PLAUSIBLE_MAX);
    p.ego_v_moving_thres      = g("ego_v_moving_thres",    EGO_V_MOVING_THRES);
    p.watchdog_timeout_s      = g("watchdog_timeout_s",      WATCHDOG_TIMEOUT_S);
    p.radar_timeout_s         = g("radar_timeout_s",         RADAR_TIMEOUT_S);
    p.radar_fusion_dedup_m    = g("radar_fusion_dedup_m",    RADAR_FUSION_DEDUP_M);
    p.radar_vx_is_relative    = gb("radar_vx_is_relative",   RADAR_VX_IS_RELATIVE);
    p.cam_health_timeout_s    = g("cam_health_timeout_s",    CAM_HEALTH_TIMEOUT_S);
    p.cut_in_lateral_factor   = g("cut_in_lateral_factor",   CUT_IN_LATERAL_FACTOR);
    p.q_x                     = g("q_x",                     Q_X);
    p.q_v                     = g("q_v",                     Q_V);
    p.q_a                     = g("q_a",                     Q_A);
    p.fusion_gate_m           = g("fusion_gate_m",           FUSION_GATE_M);
    return p;
}

void validateParams(const Params& p) {
    bool ok = true;
    auto requireCond = [&](bool condition, const char* msg) {
        if (!condition) { ROS_FATAL("[ACC/AEB] PARAM ERROR: %s", msg); ok = false; }
    };

    // dt_s must be strictly positive: the KF predict uses dt, dt², dt³… so a
    // non-positive value would produce nonsense covariance growth.
    requireCond(p.dt_s > 0.0, "dt_s must be strictly positive");

    requireCond(p.a_aeb_max     < 0.0, "a_aeb_max must be strictly negative");
    requireCond(p.a_warn_brake  < 0.0, "a_warn_brake must be strictly negative");
    requireCond(p.a_fault_brake < 0.0, "a_fault_brake must be strictly negative");
    requireCond(p.a_soft_brake  < 0.0, "a_soft_brake must be strictly negative");
    requireCond(p.a_acc_max     > 0.0, "a_acc_max must be strictly positive");

    requireCond(p.brake_enter    < 0.0, "brake_enter_mps2 must be strictly negative");
    requireCond(p.brake_dead     < 0.0, "brake_dead_mps2 must be strictly negative");
    requireCond(p.throttle_enter > 0.0, "throttle_enter_mps2 must be strictly positive");
    requireCond(p.throttle_dead  > 0.0, "throttle_dead_mps2 must be strictly positive");
    requireCond(p.throttle_dead < p.throttle_enter,
                "throttle_dead must be strictly less than throttle_enter (hysteresis boundary)");
    requireCond(std::abs(p.brake_dead) < std::abs(p.brake_enter),
                "magnitude of brake_dead must be strictly less than brake_enter");

    requireCond(p.ttc_aeb_s  > 0.0,         "ttc_aeb_s must be strictly positive");
    requireCond(p.ttc_warn_s > p.ttc_aeb_s, "ttc_warn_s must be > ttc_aeb_s");
    requireCond(p.ttc_warn_exit_s > p.ttc_warn_s, "ttc_warn_exit_s must be > ttc_warn_s");
    requireCond(p.aeb_dist_m > 0.0,         "aeb_dist_m must be strictly positive");
    requireCond(p.aeb_immediate_dist_m < p.aeb_dist_m,
                "aeb_immediate_dist_m must be strictly less than aeb_dist_m");
    requireCond(p.resume_dist_m > p.aeb_dist_m, "resume_dist_m must be > aeb_dist_m");
    requireCond(p.safe_base_m >= 0.0,       "safe_base_m must be >= 0");

    requireCond(p.torque_rate_limit > 0.0, "torque_rate_limit must be strictly positive");
    requireCond(p.brake_rate_limit  > 0.0, "brake_rate_limit must be strictly positive");

    requireCond(p.target_speed_mps > 0.0, "target_kph must be strictly positive");
    requireCond(p.time_gap_s > 0.0,       "time_gap_s must be strictly positive");
    requireCond(p.min_coast_s_normal >= 0.0, "min_coast_s_normal must be >= 0");
    requireCond(p.min_coast_s_warn   >= 0.0, "min_coast_s_warn must be >= 0");
    requireCond(p.target_loss_timeout_s > 0.0, "target_loss_timeout_s must be strictly positive");
    requireCond(p.confirm_frames > 0,      "confirm_frames must be strictly positive");
    requireCond(p.aeb_confirm_frames >= 1, "aeb_confirm_frames must be >= 1");
    requireCond(p.spinner_threads >= 1,    "spinner_threads must be >= 1");
    requireCond(p.fault_recovery_s > 0.0,  "fault_recovery_s must be strictly positive");
    requireCond(p.sensor_timeout_s > 0.0,  "sensor_timeout_s must be strictly positive");
    requireCond(p.vcu_timeout_s    > 0.0,  "vcu_timeout_s must be strictly positive");
    requireCond(p.max_lane_width_m > 0.0,  "max_lane_width_m must be strictly positive");
    requireCond(p.radar_timeout_s     > 0.0, "radar_timeout_s must be strictly positive");
    requireCond(p.radar_fusion_dedup_m > 0.0, "radar_fusion_dedup_m must be strictly positive");
    requireCond(p.cam_health_timeout_s > 0.0, "cam_health_timeout_s must be strictly positive");
    requireCond(p.cut_in_lateral_factor > 1.0, "cut_in_lateral_factor must be > 1.0 (must extend beyond ego lane)");
    requireCond(p.q_x > 0.0, "q_x must be strictly positive");
    requireCond(p.q_v > 0.0, "q_v must be strictly positive");
    requireCond(p.q_a > 0.0, "q_a must be strictly positive");
    requireCond(p.fusion_gate_m > 0.0, "fusion_gate_m must be strictly positive");

    requireCond(p.follow_enter_ratio > 1.0, "follow_enter_ratio must be > 1.0");
    requireCond(p.follow_exit_ratio > p.follow_enter_ratio,
                "follow_exit_ratio must be > follow_enter_ratio");


    requireCond(p.mass_kg    > 0.0, "mass_kg must be strictly positive");
    requireCond(p.wheel_r_m  > 0.0, "wheel_r_m must be strictly positive");
    requireCond(p.gear_ratio > 0.0, "gear_ratio must be strictly positive");
    requireCond(p.drv_eff > 0.0 && p.drv_eff <= 1.0, "drivetrain_eff must be in (0,1]");
    requireCond(p.torque_max  > 0.0, "motor_torque_max must be strictly positive");
    requireCond(p.brake_g_max > 0.0, "brake_g_max must be strictly positive");

    if (!ok) {
        ROS_FATAL("[ACC/AEB] Validation failed — fix launch parameters and restart.");
        ros::shutdown();
        std::exit(1);
    } else {
        ROS_INFO("[ACC/AEB] All parameters validated OK.");
    }
}

} // namespace acc_aeb
