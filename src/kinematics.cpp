//==============================================================================
//  kinematics.cpp  —  MIO association, Kalman tracking, and TTC math.
//==============================================================================
#include "acc_aeb_controller/kinematics.h"

#include <algorithm>
#include <cmath>
#include <ros/ros.h>

namespace acc_aeb {

bool isLaneValid(const npust_bus_msgs::LanePolynomial& msg, const Params& p) {
    if (!msg.is_valid) return false;
    if (std::abs(msg.a) > p.max_lane_curve_a) return false;
    if (std::abs(msg.b) > p.max_lane_heading_b) return false;
    return true;
}

MioResult selectMIO(const PerceptionSnapshot& snap, double ego_v, const Params& p,
                    SensorSource src) {
    MioResult best;
    double    max_threat = -1.0;

    // Score one object; updates best/max_threat if it is the highest threat so far.
    auto score = [&](double ox, double oy, double ovx,
                     double ox_var, double ovx_var,
                     int32_t oid, bool vx_rel) {
        if (ox <= p.min_valid_x_m || ox > p.max_range_m) return;
        if (!std::isfinite(ox) || !std::isfinite(oy) || !std::isfinite(ovx)) return;

        const double v_rel = vx_rel ? ovx : (ovx - ego_v);
        if (std::abs(v_rel) > p.v_rel_plausible_max) return;

        bool in_lane = false;
        if (snap.lane_l.valid && snap.lane_r.valid) {
            const double x2  = ox * ox;
            const double y_l = snap.lane_l.a * x2 + snap.lane_l.b * ox + snap.lane_l.c;
            const double y_r = snap.lane_r.a * x2 + snap.lane_r.b * ox + snap.lane_r.c;
            if (std::abs(y_l - y_r) <= p.max_lane_width_m)
                in_lane = (oy <= std::max(y_l, y_r) && oy >= std::min(y_l, y_r));
            else
                in_lane = (std::abs(oy) <= snap.dynamic_half_w);
        } else {
            in_lane = (std::abs(oy) <= snap.dynamic_half_w);
        }
        if (!in_lane) return;

        const double raw_ttc = computeTTC(ox, -v_rel);
        const double threat  = (p.mio_weight_dist / std::max(consts::MIO_SCORE_EPS, ox)) +
                               (p.mio_weight_ttc  / std::max(consts::MIO_SCORE_EPS, raw_ttc));

        if (threat > max_threat) {
            max_threat        = threat;
            best.x            = ox;
            best.id           = oid;
            best.valid        = true;
            best.threat_score = threat;
            best.v_rel        = v_rel;
            best.v_abs        = vx_rel ? (ego_v + ovx) : ovx;
            best.x_var        = ox_var;
            best.vx_var       = ovx_var;
        }
    };

    // Apply per-sensor variance defaults when the publisher leaves fields at 0.
    // Zero variance gives the KF infinite sensor confidence — corrupts the blend.
    // Warnings are throttled to 10 s each and fire inline on first zero hit —
    // no separate pre-scan loop needed.
    auto camVar = [](double v, double def) noexcept -> double {
        if (v > 0.0) return v;
        ROS_WARN_THROTTLE(10.0, "[selectMIO] Camera objects have zero variance — "
            "using fallback (x_var=%.2f vx_var=%.2f). Fix camera node.",
            consts::CAM_X_VAR_DEFAULT, consts::CAM_VX_VAR_DEFAULT);
        return def;
    };
    auto radVar = [](double v, double def) noexcept -> double {
        if (v > 0.0) return v;
        ROS_WARN_THROTTLE(10.0, "[selectMIO] Radar objects have zero variance — "
            "using fallback (x_var=%.2f vx_var=%.2f). Fix radar node.",
            consts::RAD_X_VAR_DEFAULT, consts::RAD_VX_VAR_DEFAULT);
        return def;
    };

    // Camera objects (primary: better lateral accuracy).
    if (src != SensorSource::RADAR_ONLY && !snap.cam_timeout && snap.objects) {
        for (const auto& obj : snap.objects->objects)
            score(obj.x, obj.y, obj.vx,
                  camVar(obj.x_var,  consts::CAM_X_VAR_DEFAULT),
                  camVar(obj.vx_var, consts::CAM_VX_VAR_DEFAULT),
                  obj.id, p.vx_is_relative);
    }

    if (src == SensorSource::RADAR_ONLY) {
        // Radar-only: no dedup — camera not used, KF handles the fusion separately.
        if (!snap.radar_timeout && snap.radar_objects) {
            for (const auto& robj : snap.radar_objects->objects)
                score(robj.x, robj.y, robj.vx,
                      radVar(robj.x_var,  consts::RAD_X_VAR_DEFAULT),
                      radVar(robj.vx_var, consts::RAD_VX_VAR_DEFAULT),
                      robj.id, p.radar_vx_is_relative);
        }
    } else if (src != SensorSource::CAMERA_ONLY) {
        // ALL: radar objects with dedup against in-lane camera returns.
        // Only suppress a radar object when the matching camera object is itself
        // in-lane — an out-of-lane camera false detection must not block a valid
        // radar detection.
        if (!snap.radar_timeout && snap.radar_objects) {
            const double dedup2 = p.radar_fusion_dedup_m * p.radar_fusion_dedup_m;
            for (const auto& robj : snap.radar_objects->objects) {
                bool covered = false;
                if (!snap.cam_timeout && snap.objects) {
                    for (const auto& cobj : snap.objects->objects) {
                        if (cobj.x <= p.min_valid_x_m || cobj.x > p.max_range_m) continue;
                        if (!std::isfinite(cobj.x) || !std::isfinite(cobj.y)) continue;

                        bool cam_in_lane = false;
                        if (snap.lane_l.valid && snap.lane_r.valid) {
                            const double x2  = cobj.x * cobj.x;
                            const double y_l = snap.lane_l.a * x2 + snap.lane_l.b * cobj.x + snap.lane_l.c;
                            const double y_r = snap.lane_r.a * x2 + snap.lane_r.b * cobj.x + snap.lane_r.c;
                            if (std::abs(y_l - y_r) <= p.max_lane_width_m)
                                cam_in_lane = (cobj.y <= std::max(y_l, y_r) && cobj.y >= std::min(y_l, y_r));
                            else
                                cam_in_lane = (std::abs(cobj.y) <= snap.dynamic_half_w);
                        } else {
                            cam_in_lane = (std::abs(cobj.y) <= snap.dynamic_half_w);
                        }
                        if (!cam_in_lane) continue;

                        const double dx = robj.x - cobj.x;
                        const double dy = robj.y - cobj.y;
                        if (dx * dx + dy * dy < dedup2) { covered = true; break; }
                    }
                }
                if (!covered)
                    score(robj.x, robj.y, robj.vx,
                          radVar(robj.x_var,  consts::RAD_X_VAR_DEFAULT),
                          radVar(robj.vx_var, consts::RAD_VX_VAR_DEFAULT),
                          robj.id, p.radar_vx_is_relative);
            }
        }
    }

    return best;
}

MioResult selectAdjacentMIO(const PerceptionSnapshot& snap, double ego_v, const Params& p) {
    MioResult best;
    double    max_threat = -1.0;

    // Returns true if `oy` is in the adjacent cut-in zone at longitudinal dist `ox`:
    // outside the ego-lane boundary but within cut_in_lateral_factor × dynamic_half_w.
    auto in_adjacent = [&](double ox, double oy) -> bool {
        const double outer_w = snap.dynamic_half_w * p.cut_in_lateral_factor;

        bool in_ego = false;
        if (snap.lane_l.valid && snap.lane_r.valid) {
            const double x2  = ox * ox;
            const double y_l = snap.lane_l.a * x2 + snap.lane_l.b * ox + snap.lane_l.c;
            const double y_r = snap.lane_r.a * x2 + snap.lane_r.b * ox + snap.lane_r.c;
            if (std::abs(y_l - y_r) <= p.max_lane_width_m)
                in_ego = (oy <= std::max(y_l, y_r) && oy >= std::min(y_l, y_r));
            else
                in_ego = (std::abs(oy) <= snap.dynamic_half_w);
        } else {
            in_ego = (std::abs(oy) <= snap.dynamic_half_w);
        }
        return !in_ego && (std::abs(oy) <= outer_w);
    };

    auto score = [&](double ox, double oy, double ovx, int32_t oid, bool vx_rel) {
        if (ox <= p.min_valid_x_m || ox > p.max_range_m) return;
        if (!std::isfinite(ox) || !std::isfinite(oy) || !std::isfinite(ovx)) return;

        const double v_rel = vx_rel ? ovx : (ovx - ego_v);
        if (std::abs(v_rel) > p.v_rel_plausible_max) return;
        if (!in_adjacent(ox, oy)) return;

        const double raw_ttc = computeTTC(ox, -v_rel);
        const double threat  = (p.mio_weight_dist / std::max(consts::MIO_SCORE_EPS, ox)) +
                               (p.mio_weight_ttc  / std::max(consts::MIO_SCORE_EPS, raw_ttc));

        if (threat > max_threat) {
            max_threat        = threat;
            best.x            = ox;
            best.id           = oid;
            best.valid        = true;
            best.threat_score = threat;
            best.v_rel        = v_rel;
            best.v_abs        = vx_rel ? (ego_v + ovx) : ovx;
        }
    };

    // Camera objects in the adjacent zone.
    if (!snap.cam_timeout && snap.objects) {
        for (const auto& obj : snap.objects->objects)
            score(obj.x, obj.y, obj.vx, obj.id, p.vx_is_relative);
    }

    // Radar objects — dedup against any nearby camera object (same logic as selectMIO).
    if (!snap.radar_timeout && snap.radar_objects) {
        const double dedup2 = p.radar_fusion_dedup_m * p.radar_fusion_dedup_m;
        for (const auto& robj : snap.radar_objects->objects) {
            bool covered = false;
            if (!snap.cam_timeout && snap.objects) {
                for (const auto& cobj : snap.objects->objects) {
                    if (cobj.x <= p.min_valid_x_m || cobj.x > p.max_range_m) continue;
                    if (!std::isfinite(cobj.x) || !std::isfinite(cobj.y)) continue;
                    const double dx = robj.x - cobj.x;
                    const double dy = robj.y - cobj.y;
                    if (dx * dx + dy * dy < dedup2) { covered = true; break; }
                }
            }
            if (!covered)
                score(robj.x, robj.y, robj.vx, robj.id, p.radar_vx_is_relative);
        }
    }

    return best;
}

MioResult MioTracker::toMioResult(double ego_v) const {
    MioResult r;
    r.x     = kf_.x;
    r.v_rel = kf_.v;
    r.a_rel = kf_.a;
    r.v_abs = ego_v + kf_.v;
    r.valid = true;
    r.stale = false;
    r.x_var  = kf_.P[0];
    r.vx_var = kf_.P[3];
    return r;
}

MioResult MioTracker::propagateGrace(double dt, double ego_v) {
    if (!kf_.initialised || !last_valid_.valid) return {};
    kfPredict(kf_, dt, p_.q_x, p_.q_v, p_.q_a);
    MioResult g = toMioResult(ego_v);
    g.stale = true;
    last_valid_ = g;
    return g;
}

MioResult MioTracker::track(const MioResult& cam, const MioResult& rad,
                             double ego_v, double dt) {
    const bool have_cam = cam.valid;
    const bool have_rad = rad.valid;

    // ── 1. No detection from either sensor: coast then give up ───────────────
    if (!have_cam && !have_rad) {
        stale_timer_ += dt;
        confirm_count_ = 0;
        if (stale_timer_ >= p_.target_loss_timeout_s) {
            reset();
            return {};
        }
        return propagateGrace(dt, ego_v);
    }

    stale_timer_ = 0.0;

    // Use camera ID as primary (better lateral); fall back to radar.
    const int32_t primary_id = have_cam ? cam.id : rad.id;

    // Bypass confirm gate for imminent threats.
    const double x_ref    = have_cam ? cam.x : rad.x;
    const double vrel_ref = have_cam ? cam.v_rel : rad.v_rel;
    const double ttc_ref  = computeTTC(x_ref, -vrel_ref);
    const bool bypass = (x_ref < p_.aeb_immediate_dist_m) ||
                        (-vrel_ref > p_.min_closing_speed_mps && ttc_ref < p_.ttc_aeb_s);

    if (primary_id != prev_cam_id_ && !bypass) {
        if (primary_id != candidate_id_) {
            candidate_id_  = primary_id;
            confirm_count_ = 0;
        }
        ++confirm_count_;
        if (confirm_count_ < p_.confirm_frames) {
            return propagateGrace(dt, ego_v);
        }
    }

    prev_cam_id_   = primary_id;
    confirm_count_ = p_.confirm_frames;

    // ── 2. Predict ────────────────────────────────────────────────────────────
    if (!kf_.initialised) {
        const MioResult& src = have_cam ? cam : rad;
        kfInit(kf_, src.x, src.v_rel, src.x_var, src.vx_var);
    } else {
        kfPredict(kf_, dt, p_.q_x, p_.q_v, p_.q_a);
    }

    // ── 3. Update — camera first, then radar ──────────────────────────────────
    if (have_cam) {
        kfUpdate(kf_, cam.x, cam.v_rel, cam.x_var, cam.vx_var);
    }
    if (have_rad) {
        // Fusion gate: only use radar if it agrees with camera within fusion_gate_m.
        // Without this check, camera tracking a car at 25 m and radar seeing a
        // guardrail at 8 m would corrupt the fused estimate toward 8 m.
        const bool agree = !have_cam ||
                           (std::abs(cam.x - rad.x) < p_.fusion_gate_m);
        if (agree) {
            kfUpdate(kf_, rad.x, rad.v_rel, rad.x_var, rad.vx_var);
        } else {
            ROS_WARN_THROTTLE(2.0,
                "[MioTracker] cam/rad mismatch cam=%.1fm rad=%.1fm (gate=%.1fm) "
                "— radar update skipped this cycle",
                cam.x, rad.x, p_.fusion_gate_m);
        }
    }

    // ── 4. Build output ───────────────────────────────────────────────────────
    MioResult out = toMioResult(ego_v);
    out.threat_score = have_cam ? cam.threat_score : rad.threat_score;
    out.id           = primary_id;

    last_valid_ = out;
    return out;
}

void MioTracker::reset() noexcept {
    kf_            = KFState{};
    stale_timer_   = 0.0;
    prev_cam_id_   = -1;
    candidate_id_  = -1;
    confirm_count_ = 0;
    last_valid_    = {};
}

KinResult computeKinematics(double ego_v, const MioResult& mio, const Params& p) {
    KinResult k;
    k.closing_spd = -mio.v_rel;

    double ttc_measured = INF_D;
    const double a_rel = mio.a_rel;
    const double v_rel = mio.v_rel;

    // Constant-acceleration TTC: solve 0 = x + v_rel*t + 0.5*a_rel*t^2.
    if (std::abs(a_rel) > consts::KIN_MIN_A_REL_MPS2) {
        const double discriminant = v_rel * v_rel - 2.0 * a_rel * mio.x;
        if (discriminant >= 0.0) {
            const double sqrt_disc = std::sqrt(discriminant);
            const double inv_a     = 1.0 / a_rel;     // |a_rel| > 0.1 -> reciprocal is safe
            const double root1 = (-v_rel + sqrt_disc) * inv_a;
            const double root2 = (-v_rel - sqrt_disc) * inv_a;

            const bool r1_ok = std::isfinite(root1) && root1 > 0.0;
            const bool r2_ok = std::isfinite(root2) && root2 > 0.0;
            if      (r1_ok && r2_ok) ttc_measured = std::min(root1, root2);
            else if (r1_ok)          ttc_measured = root1;
            else if (r2_ok)          ttc_measured = root2;
        }
    }

    if (ttc_measured == INF_D && k.closing_spd > p.min_closing_speed_mps) {
        ttc_measured = computeTTC(mio.x, k.closing_spd);
    }

    // Backstop: a (near-)stationary object dead-ahead while we are moving.
    double ttc_backstop = INF_D;
    if (std::abs(mio.v_abs) < p.stationary_v_abs_max && ego_v > p.ego_v_moving_thres) {
        const double backstop_range = std::max(p.aeb_backstop_base_m,
                                                ego_v * consts::BACKSTOP_RANGE_TIME_S);
        if (mio.x < backstop_range) {
            ttc_backstop = computeTTC(mio.x, ego_v);
        }
    }

    k.ttc         = std::min(ttc_measured, ttc_backstop);
    k.target_dist = ego_v * (p.time_gap_s + p.latency_s) + p.safe_base_m;  // factored: 1 mul + 1 add
    return k;
}

} // namespace acc_aeb
