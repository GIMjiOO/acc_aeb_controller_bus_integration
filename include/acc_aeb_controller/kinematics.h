#pragma once
//==============================================================================
//  kinematics.h  —  Perception association + pure kinematic math.
//
//  Owns the "where is the threat and how is it moving" pipeline:
//    PerceptionSnapshot  (immutable per-cycle view of the perception buffers)
//    isLaneValid()       (lane-polynomial sanity gate)
//    selectMIO()         (Most-Important-Object association, pure)
//    MioTracker          (stateful multi-sensor Kalman filter + grace coasting)
//    computeKinematics() (TTC + desired gap, pure)
//
//  No roscpp here — only the perception message types — so this layer stays
//  unit-testable in isolation.
//==============================================================================
#include "acc_aeb_controller/types.h"
#include "acc_aeb_controller/math_utils.h"

#include <npust_bus_msgs/TrackedObjectArray.h>
#include <npust_bus_msgs/LanePolynomial.h>

namespace acc_aeb {

// Immutable snapshot the control loop builds once per cycle under the
// perception mutex, then passes around by const-ref (no further locking).
struct PerceptionSnapshot {
    npust_bus_msgs::TrackedObjectArray::ConstPtr objects;       // camera
    npust_bus_msgs::TrackedObjectArray::ConstPtr radar_objects; // radar (may be null)
    struct Lane {
        double a = 0.0, b = 0.0, c = 0.0;
        bool   valid = false;
    } lane_l, lane_r;
    double dynamic_half_w    = 0.0;
    bool   cam_timeout       = false;  // camera objects stale
    bool   radar_timeout     = false;  // radar objects stale
    // Camera content health: seconds timestamp of last non-empty camera message
    // (wall time as toSec(); 0.0 = never received).
    double cam_nonempty_sec  = 0.0;
    bool   cam_nonempty_ever = false;
};

[[nodiscard]] bool isLaneValid(const npust_bus_msgs::LanePolynomial& msg, const Params& p);

enum class SensorSource { CAMERA_ONLY, RADAR_ONLY, ALL };

// Pure association: pick the highest-threat in-lane object. No internal state.
// src controls which sensor(s) to consider; defaults to ALL (original behaviour).
[[nodiscard]] MioResult selectMIO(const PerceptionSnapshot& snap, double ego_v, const Params& p,
                                  SensorSource src = SensorSource::ALL);

// Pick the highest-threat object in the ADJACENT zone (outside ego lane but
// within cut_in_lateral_factor × dynamic_half_w). Used for cut-in awareness
// diagnostics only — does NOT feed the AEB/state-machine pipeline.
[[nodiscard]] MioResult selectAdjacentMIO(const PerceptionSnapshot& snap, double ego_v, const Params& p);

// Pure kinematics for the (already tracked) MIO: time-to-collision and the
// desired following distance.
[[nodiscard]] KinResult computeKinematics(double ego_v, const MioResult& mio, const Params& p);

//------------------------------------------------------------------------------
//  Stateful multi-sensor Kalman tracker with confirm/grace logic.
//------------------------------------------------------------------------------
class MioTracker {
public:
    explicit MioTracker(const Params& p) : p_(p) {}

    // Two-step multi-sensor Kalman update. Call with camera MIO first, then
    // radar MIO (both may be invalid). Returns the fused smoothed MIO, a
    // KF-predicted "grace" MIO during brief dropouts, or invalid once lost.
    [[nodiscard]] MioResult track(const MioResult& camera_raw,
                                  const MioResult& radar_raw,
                                  double ego_v, double dt);

    void reset() noexcept;

private:
    [[nodiscard]] MioResult toMioResult(double ego_v) const;
    [[nodiscard]] MioResult propagateGrace(double dt, double ego_v);

    const Params& p_;
    KFState   kf_{};
    double    stale_timer_   = 0.0;
    int32_t   prev_cam_id_   = -1;
    int32_t   candidate_id_  = -1;
    int       confirm_count_ = 0;
    MioResult last_valid_{};   // kept for grace propagation
};

} // namespace acc_aeb
