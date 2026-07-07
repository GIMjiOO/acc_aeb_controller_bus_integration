# acc_aeb_controller (v16.0)

Adaptive Cruise Control / Autonomous Emergency Braking longitudinal controller for the **NPUST Heavy Bus Platform**.

- **Platform:** Jetson AGX Orin · aarch64 · Ubuntu 20.04
- **Runtime:** ROS 1 Noetic · C++17
- **Control rate:** 20 Hz (configurable via `dt_s`)

---

## Table of Contents

1. [Overview](#1-overview)
2. [System Architecture](#2-system-architecture)
3. [Threading Model](#3-threading-model)
4. [Data Flow Per Tick](#4-data-flow-per-tick)
5. [State Machine](#5-state-machine)
6. [Perception Pipeline — MIO Selection](#6-perception-pipeline--mio-selection)
7. [Sensor Fusion — Kalman Tracker](#7-sensor-fusion--kalman-tracker)
8. [Kinematics](#8-kinematics)
9. [Control Law](#9-control-law)
10. [Actuator Mapping](#10-actuator-mapping)
11. [Watchdog](#11-watchdog)
12. [Topics](#12-topics)
13. [Parameters](#13-parameters)
14. [Diagnostics](#14-diagnostics)
15. [Build and Run](#15-build-and-run)
16. [Test Suite](#16-test-suite)
17. [Developer Tools](#17-developer-tools)
18. [Safety Notes](#18-safety-notes)
19. [Known Gaps](#19-known-gaps)
20. [Message Field Assumptions](#20-message-field-assumptions)
21. [Module Dependency Graph](#21-module-dependency-graph)

---

## 1. Overview

The controller implements a full longitudinal ADAS stack on a 10-tonne electric bus:

| Function | Description |
|---|---|
| **ACC (Adaptive Cruise Control)** | Maintains a configurable target speed when the road ahead is clear; follows a leading vehicle at a safe, speed-dependent time gap. |
| **WARN** | Pre-brakes softly when TTC drops below a warning threshold. |
| **AEB (Autonomous Emergency Braking)** | Commands maximum safe deceleration when an imminent collision is detected; bypasses the jerk limiter for immediate response. |
| **FAULT / Safe-stop** | Applies a controlled deceleration whenever perception or ego-speed data become stale; recovers automatically once sensors are healthy for `fault_recovery_s`. |

The node is designed for **zero hot-path heap allocation**: all ROS messages are pre-allocated at startup and reused every tick. Control atomics are lock-free. The perception buffer uses a single mutex, held only for a snapshot copy at the start of each tick.

---

## 2. System Architecture

```
External sensors / systems
══════════════════════════════════════════════════════════════════
  /VCU_Data              →  ego_v_mps (atomic<double>)
  /planner/target_speed  →  dynamic_target_speed_mps (atomic<double>)
  /driver_override       →  driver_override_active (atomic<bool>)

  /perception/camera/tracked_objects ─┐
  /perception/radar/tracked_objects   ├─ perc_mutex_ buffer
  /perception/lanes/left              │  (written by sensor-spinner threads)
  /perception/lanes/right             ┘

ACC/AEB Controller node
══════════════════════════════════════════════════════════════════
  ┌─────────────────────────────────────────────────────────────┐
  │  Per-tick (20 Hz, control-spinner thread):                  │
  │                                                             │
  │  snapshot()          Lock once, copy pointers + lane data   │
  │       │                                                     │
  │       ▼                                                     │
  │  selectMIO()         Score in-lane objects (cam + radar)    │
  │       │                                                     │
  │       ▼                                                     │
  │  MioTracker::track() 3-state Kalman filter; grace coast     │
  │       │              on brief dropouts; confirm gate        │
  │       ▼                                                     │
  │  computeKinematics() TTC (quadratic + backstop), gap        │
  │       │                                                     │
  │       ▼                                                     │
  │  StateMachine::step() FSM + PI/PID control law + jerk limit │
  │       │                                                     │
  │       ▼                                                     │
  │  mapToActuators()    THROTTLE/COAST/BRAKE + rate limits     │
  │       │                                                     │
  │       ▼                                                     │
  │  publish → /control_value  (target_speed_value, mode_value) │
  │          → /diagnostics                                     │
  └─────────────────────────────────────────────────────────────┘

  Watchdog thread (steady_clock, 50 Hz poll):
  ═══════════════════════════════════════════
  Detects frozen control thread → publishes safe-stop on /control_value
  and a separate ERROR status on /diagnostics.

Downstream
══════════════════════════════════════════════════════════════════
  /control_value → bus integration layer → target-speed tracking
```

**Output note:** `/control_value` publishes only `target_speed_value` (m/s, integrated from the jerk-limited accel command) and `mode_value` (current FSM state as a string, inspect-only). `torque_nm`/`brake_g` are still computed internally every tick via `mapToActuators()` (used for jerk-limited rate limiting and bench logging) but are **not** published — this is a deliberate replacement per the bus team's integration requirements, not an addition. See [§10 Actuator Mapping](#10-actuator-mapping) and [§12 Topics](#12-topics).

### Package layout

```
acc_aeb_controller/
├── CMakeLists.txt
├── package.xml
├── README.md                          ← this file
├── launch/
│   ├── acc_aeb_controller.launch      main deployment launch
│   └── live_test.launch               real camera + simulated ego dynamics
├── scripts/
│   ├── fake_vcu.py                    interactive ego-speed publisher (bench)
│   ├── monitor.py                     live terminal display of sensor + KF state
│   └── vehicle_plant.py              integrates /control_value → /VCU_Data
├── include/acc_aeb_controller/
│   ├── types.h                        POD structs, enums, all defaults (ROS-free)
│   ├── math_utils.h                   clampVal, safeDiv, computeTTC, KF math
│   ├── params.h                       parameter loading + validation (header)
│   ├── kinematics.h                   selectMIO, MioTracker, computeKinematics
│   └── state_machine.h                FSM, control law, actuator mapping
└── src/
    ├── acc_aeb_node.cpp               ROS I/O, watchdog, diagnostics, main()
    ├── kinematics.cpp
    ├── params.cpp
    └── state_machine.cpp
```

---

## 3. Threading Model

Four concurrent threads with carefully bounded sharing:

| Thread | Mechanism | Shared state |
|---|---|---|
| **ctrl-spinner** | `ros::AsyncSpinner(1, &ctrl_queue_)` — dedicated queue, never re-entrant | Reads atomics; reads/writes `perc_mutex_`-snapshot; writes `last_loop_steady_ns_` |
| **sensor-spinner** | `ros::AsyncSpinner(N)` on global queue | Writes `perc_mutex_`-guarded buffers; writes lock-free atomics (`ego_v_mps_`, `vcu_last_arrival_sec_`, `dynamic_target_speed_mps_`, `driver_override_active_`) |
| **watchdog** | `std::thread` on `steady_clock` | Reads `last_loop_steady_ns_` (lock-free); writes `watchdog_tripped_` (lock-free); publishes via own pre-built `wd_stop_msg_` |
| **main** | `ros::WallRate` idle, SIGINT/SIGTERM handler | Quiesces spinners; calls `engageShutdownSafeStop()` |

**Lock-free guarantee:** All cross-thread atomics (`std::atomic<double>`, `std::atomic<bool>`, `std::atomic<int64_t>`, `std::atomic<uint32_t>`) are verified always-lock-free via `static_assert` at compile time. A non-lock-free target fails the build loudly rather than silently degrading.

**Perception mutex:** Held only for the brief `snapshot()` call at the top of each control tick. No ROS calls are made while holding it.

**Pre-allocated messages:** `ctrl_msg_`, `loop_stop_msg_`, `wd_stop_msg_`, `diag_msg_`, and `wd_diag_msg_` are allocated at startup. Only `header.stamp` and value strings are updated per tick. String values use `reserve(24)` so `assign()` never reallocates.

---

## 4. Data Flow Per Tick

Every `dt_s` seconds (default 50 ms, 20 Hz), the control-spinner thread executes:

```
1. Heartbeat          Update last_loop_steady_ns_ (monotonic steady_clock).
                       Consume watchdog_tripped_ if set → full command-pipeline reset.

2. snapshot()         Under perc_mutex_: copy latest_objects_, latest_radar_objects_,
                       lane polys, freshness flags → PerceptionSnapshot (immutable).

3. Staleness check    cam_stale  = camera message age > sensor_timeout_s
                       rad_stale  = radar  message age > radar_timeout_s
                       obj_stale  = cam_stale AND rad_stale   (FAULT if true)
                       vcu_stale  = VCU message age > vcu_timeout_s  (FAULT if true)

4. selectMIO(CAMERA)  Score all in-lane camera objects; pick highest threat.
   selectMIO(RADAR)   Score all in-lane radar objects not covered by camera.

5. MioTracker::track  Sequential Kalman update: predict → camera update → radar update.
                       Grace-coast if both sensors momentarily absent.
                       Confirm gate: require N frames for a new track ID before trusting.

6. computeKinematics  TTC (quadratic constant-acceleration or backstop), target gap.

7. selectAdjacentMIO  Cut-in awareness zone (diagnostics only, not fed to FSM).

8. Camera health      Warn if camera sends empty arrays while ego is moving > cam_health_timeout_s.

9. StateMachine::step FSM transition + PI/PID control + jerk limit → accel_cmd,
                       target_speed (integrated from accel_cmd), and actuator cmd
                       (torque_nm/brake_g — computed for rate limiting + logging).

10. Driver override   If override active in CRUISE/FOLLOW: publish target_speed_value = ego_v
                       (hold current speed, no ACC-commanded acceleration).
                       AEB/WARN/FAULT remain fully armed.

11. NaN guard         If accel_cmd is non-finite → publishLoopSafeStop() + pipeline reset.

12. Publish           /control_value  (target_speed_value, mode_value)
                       /diagnostics    (18 key-value fields + state)
```

---

## 5. State Machine

Five operational states. The node starts in **FAULT**; the first real tick transitions out once VCU and at least one sensor have each been received.

```
          ┌────────────────────────────────────┐
          │              FAULT                 │◄── sensor_ok = false
          │  cmd = a_fault_brake               │    (cam_stale AND rad_stale) OR vcu_stale
          │  jerk = jerk_warn                  │
          └────────────────────────────────────┘
               ▲  fault_recovery_s of clean sensors  │
               │◄──────────────────────────────────►▼
          ┌────────────────────────────────────────────────────┐
          │                    CRUISE                          │
          │  cmd = PI(v_err)   a ∈ [a_soft_brake, a_acc_max]  │
          └────────────────────────────────────────────────────┘
               │  mio.x ≤ target_dist × follow_enter_ratio
               ▼
          ┌────────────────────────────────────────────────────┐
          │                    FOLLOW                          │
          │  cmd = PID(dist_err, v_rel) + FF(a_rel)           │
          └────────────────────────────────────────────────────┘
               │  TTC < ttc_warn_s  OR  mio.x < aeb_dist_m
               ▼
          ┌────────────────────────────────────────────────────┐
          │                     WARN                           │
          │  cmd = a_warn_brake  (jerk-limited)                │
          └────────────────────────────────────────────────────┘
               │  TTC < ttc_aeb_s  AND  N=aeb_confirm_frames,
               │  OR mio.x < aeb_immediate_dist_m (no confirm)
               ▼
          ┌────────────────────────────────────────────────────┐
          │                      AEB                           │
          │  cmd = a_aeb_max  (NO jerk limit)                  │
          │  Latches until ego_v < stop_speed_mps,             │
          │  then holds aeb_hold_s; clears if mio.x > resume_dist_m
          └────────────────────────────────────────────────────┘
```

### State transition details

| From → To | Condition |
|---|---|
| Any → FAULT | `obj_stale && vcu_stale` — both perception sensors stale, OR VCU stale |
| FAULT → ops | `fault_recovery_s` of clean sensors elapsed |
| CRUISE → FOLLOW | `mio.x ≤ target_dist × follow_enter_ratio` (1.5× gap by default) |
| FOLLOW → CRUISE | `mio.x > target_dist × follow_exit_ratio` (1.8× gap by default) |
| any → WARN | `TTC < ttc_warn_s` OR `mio.x < aeb_dist_m` |
| WARN → CRUISE/FOLLOW | `TTC > ttc_warn_exit_s` (hysteresis) |
| any → AEB | `TTC < ttc_aeb_s` (N confirm frames) OR `mio.x < aeb_immediate_dist_m` (0 frames) |
| AEB latched | `ego_v < stop_speed_mps` → hold `aeb_hold_s` seconds |
| AEB clear | Hold timer expired AND `mio.x > resume_dist_m` |

### Driver override interaction

| State | Override active? | Output |
|---|---|---|
| CRUISE / FOLLOW | Yes | `target_speed_value = ego_v` (hold current speed — driver has authority) |
| WARN / AEB | Yes | Full brake command **always active** (target speed integrates the fault/AEB decel) — collision protection overrides the driver |
| FAULT (sensor stale) | Yes | FSM is shielded from the stale-sensor signal and stays in CRUISE, so `target_speed_value = ego_v` — driver can drive the bus freely; WARNING logged every 5 s that AEB is non-functional |
| FAULT (sensor stale) | No | Fault brake (`a_fault_brake`) applied as normal — `target_speed_value` decays toward 0 |

When sensors recover while override is still active, the controller exits FAULT automatically after `fault_recovery_s` and AEB/WARN re-arm — no driver action required.

### Target distance (following gap)

```
target_dist = ego_v × (time_gap_s + latency_s) + safe_base_m
```

This grows linearly with speed, ensuring a safe time headway at any velocity plus a constant minimum standstill gap.

---

## 6. Perception Pipeline — MIO Selection

`selectMIO()` in [kinematics.cpp](src/kinematics.cpp) picks the **Most Important Object** — the single in-lane threat with the highest composite score.

### Object scoring

For each object at `(x, y, vx)`:

1. **Range gate:** `min_valid_x_m < x ≤ max_range_m` (default 0.5 m – 120 m).
2. **Finite check:** `x`, `y`, `vx` must all be finite.
3. **Velocity plausibility:** `|v_rel| ≤ v_rel_plausible_max` (default 70 m/s).
4. **Lane association:**
   - If both lane polynomials are valid: object is in-lane if its `y` falls between the left and right lane curves at range `x` (quadratic: `y = a·x² + b·x + c`). If the computed lane width exceeds `max_lane_width_m`, falls back to the dynamic half-width.
   - Otherwise: `|y| ≤ dynamic_half_w`.
5. **Threat score:** `score = mio_weight_dist / x + mio_weight_ttc / TTC` (higher = more dangerous).

The object with the **highest threat score** is returned as the MIO.

### Dynamic lane width

```
dynamic_half_w = max(min_static_half_w_m,  static_half_w_m - ego_v × lane_narrow_rate)
```

The effective corridor narrows at speed to exclude slow roadside objects that would only be a risk at low speed.

### Sensor redundancy

| Camera | Radar | Mode | Controller |
|---|---|---|---|
| Fresh | Fresh | Full fusion | Both updates applied to KF |
| Fresh | Stale | Camera only | `INFO_THROTTLE` log |
| Stale | Fresh | Radar only | `WARN_THROTTLE` log |
| Stale | Stale | No perception | **FAULT** — safe-stop |

Camera is treated as primary (better lateral accuracy). Radar objects are included only if they are **not covered** by a nearby camera detection (within `radar_fusion_dedup_m` XY distance) that is itself in-lane. This prevents a camera false detection outside the lane from shadowing a valid radar return inside it.

### Adjacent-lane cut-in zone

`selectAdjacentMIO()` separately tracks the highest-threat object in the zone between `dynamic_half_w` and `cut_in_lateral_factor × dynamic_half_w`. The result feeds two active cut-in protection behaviours **before** it is exposed in `/diagnostics`:

| Behaviour | Condition | What happens |
|---|---|---|
| **Tracker pre-warm** | No in-lane detection AND `adj_kin.ttc < ttc_warn_s` | Adjacent MIO is fed to `MioTracker` as a camera input. The confirm gate counts down while the vehicle is still in the adjacent zone; when it crosses the lane line `cam_raw` picks up the same ID — already confirmed — so WARN/AEB fires with no extra 3-frame delay. |
| **Direct AEB promotion** | `adj_kin.ttc < ttc_aeb_s` AND more urgent than in-lane track | Adjacent MIO is routed directly to the FSM, bypassing the tracker. At sub-AEB TTC a cutting vehicle may collide before finishing the lane crossing; the brake fires immediately. Log: `CUT-IN AEB: adjacent x=…m TTC=…s`. |

Results are also exposed in `/diagnostics` as `adj_mio_x_m` and `adj_ttc_s` for operator awareness regardless of whether a promotion occurred.

---

## 7. Sensor Fusion — Kalman Tracker

`MioTracker` in [kinematics.cpp](src/kinematics.cpp) maintains a **3-state Kalman filter** over the longitudinal axis of the MIO.

### State vector

```
x̂ = [ x_range (m),  v_rel (m/s),  a_rel (m/s²) ]
```

### Covariance storage

Upper-triangular: `P[0]=Pxx  P[1]=Pxv  P[2]=Pxa  P[3]=Pvv  P[4]=Pva  P[5]=Paa`

### Prediction step (`kfPredict`)

Constant-acceleration kinematic model (`F` matrix implicit):

```
x_new = x + v·dt + 0.5·a·dt²
v_new = v + a·dt
a     unchanged
```

Process noise covariance added per continuous-time Wiener acceleration model (`q_x`, `q_v`, `q_a`).

### Update step (`kfUpdate`)

Measurement model: `H = [1 0 0; 0 1 0]` (observe range and relative velocity).

Noise matrix: `R = diag(x_var, vx_var)`.

Implemented in **Joseph form** (`P = (I−KH)P⁻(I−KH)ᵀ + KRKᵀ`) for numerical symmetry preservation. Diagonal terms are floor-clamped to 1e-6 to prevent negative variances from accumulating.

Relative acceleration is clamped to `±KF_A_REL_CLAMP_MPS2` (15 m/s²) after every update.

### Per-tick flow

```
1. No detections from either sensor?
   → stale_timer += dt
   → if stale_timer ≥ target_loss_timeout_s: reset KF
   → else: propagateGrace (predict-only, flagged `stale=true`)

2. New ID differs from previous AND not an imminent threat (bypass=true)?
   → confirm gate: require confirm_frames consecutive frames
   → during confirmation: propagateGrace from last valid

3. KF not yet initialised?
   → kfInit from first detection (camera preferred)

4. Predict: kfPredict(dt, q_x, q_v, q_a)

5. Camera update: kfUpdate(cam.x, cam.v_rel, cam.x_var, cam.vx_var)

6. Radar update (if available AND |cam.x − rad.x| < fusion_gate_m):
   kfUpdate(rad.x, rad.v_rel, rad.x_var, rad.vx_var)
   (If mismatch exceeds gate: log WARN_THROTTLE, skip radar this cycle)
```

### Variance defaults

Publishers that leave `x_var` / `vx_var` at 0.0 produce infinite KF confidence, which corrupts the blended estimate. Fallback defaults are applied silently with a throttled warning:

| Sensor | `x_var` default | `vx_var` default |
|---|---|---|
| Camera | 1.00 m² | 2.00 m²/s² |
| Radar  | 0.10 m² | 0.05 m²/s² |

---

## 8. Kinematics

`computeKinematics()` in [kinematics.cpp](src/kinematics.cpp) computes TTC and the desired following distance from the fused MIO state.

### Time-to-collision (TTC)

**Quadratic path** (used when `|a_rel| > KIN_MIN_A_REL_MPS2 = 0.1 m/s²`):

Solve `x + v_rel·t + 0.5·a_rel·t² = 0` for the smallest positive `t`:

```
discriminant = v_rel² − 2·a_rel·x
ttc = min positive root of the quadratic
```

**Linear fallback** (used when `|a_rel|` is small or quadratic has no real root):

```
ttc = x / closing_speed    (where closing_speed = −v_rel > min_closing_speed_mps)
```

**Stationary-object backstop:**

If `|v_abs| < stationary_v_abs_max` (object nearly parked) AND ego is moving (`ego_v > ego_v_moving_thres`) AND `x < max(aeb_backstop_base_m, ego_v × 4.0 s)`:

```
ttc_backstop = x / ego_v
```

Final TTC is the minimum of the quadratic/linear path and the backstop path.

### Target distance

```
target_dist = ego_v × (time_gap_s + latency_s) + safe_base_m
```

`latency_s` accounts for actuator and perception pipeline delay so the gap is already opening before the braking command reaches the wheels.

---

## 9. Control Law

`StateMachine::computeCommand()` in [state_machine.cpp](src/state_machine.cpp) selects the desired longitudinal acceleration:

### CRUISE (speed hold)

```
v_err    = target_speed − ego_v
accel    = kp_cruise × v_err + ki_cruise × integral_cruise
integral += v_err × dt   (anti-windup: frozen when saturated AND error pushes further)
accel    = clamp(accel, a_soft_brake, a_acc_max)
```

### FOLLOW (gap hold)

```
dist_err = mio.x − target_dist
accel    = kp_follow_dist × dist_err
         + kd_follow_vrel  × mio.v_rel
         + ki_follow       × integral_follow
         + kf_follow_arel  × mio.a_rel
integral += dist_err × dt   (anti-windup clamped to ±follow_iclamp)
accel    = clamp(accel, a_soft_brake, a_acc_max)
```

When the tracker is in grace (stale MIO): `accel = min(loss_hold_decel, 0)` — gentle deceleration to stay safe.

### WARN

```
accel = a_warn_brake   (fixed, jerk-limited at jerk_warn rate)
```

### AEB

```
accel = a_aeb_max   (fixed, NO jerk limiting — immediate full braking)
```

### FAULT

```
accel = a_fault_brake   (fixed, jerk-limited at jerk_warn rate)
```

### Jerk limiting

Applied to all states except AEB:

```
comfort_jerk = (ego_v < low_speed_threshold_mps) ? jerk_low_speed : jerk_normal
jerk_decel   = (WARN or FAULT) ? jerk_warn : comfort_jerk

accel_out = clamp(accel_desired,
                  last_accel − jerk_decel × dt,    // decel headroom (fast)
                  last_accel + comfort_jerk × dt)  // accel headroom (slow)
```

State transitions reset integrators (`integral_cruise_`, `integral_follow_`) to zero.

---

## 10. Actuator Mapping

`StateMachine::mapToActuators()` converts the jerk-limited acceleration command into torque (Nm) and brake (g). **These values are computed every tick but are not published to `/control_value`** (see [§12 Topics](#12-topics)) — they exist to drive the COAST/THROTTLE/BRAKE hysteresis and rate limiting below, and are logged to the console for bench comparison against the published `target_speed_value`.

### Actuator mode hysteresis

```
COAST → THROTTLE  if  cmd ≥ throttle_enter  AND  coast timer ≥ min_coast_s
COAST → BRAKE     if  cmd ≤ brake_enter     AND  coast timer ≥ min_coast_s
THROTTLE → COAST  if  cmd < throttle_dead
BRAKE → COAST     if  cmd > brake_dead
```

The coast dwell (`min_coast_s_normal` / `min_coast_s_warn`) prevents rapid THROTTLE↔BRAKE switching. AEB always forces BRAKE mode with no dwell.

### Throttle command

```
torque_nm = clamp(torque_per_accel × accel_cmd, 0, torque_max)
```

where `torque_per_accel = (mass_kg × wheel_r_m) / (gear_ratio × drivetrain_eff)`

### Brake command

```
brake_g = clamp(|accel_cmd| / 9.80665, 0, brake_g_max)
```

### Rate limits (non-AEB only)

```
torque_nm = clamp(torque_nm, last_pub_torque − torque_rate_limit × dt,
                              last_pub_torque + torque_rate_limit × dt)
brake_g   = clamp(brake_g,   last_pub_brake  − brake_rate_limit × dt,
                              last_pub_brake  + brake_rate_limit × dt)
```

---

## 11. Watchdog

The watchdog thread polls `steady_clock` every 20 ms (50 Hz). It monitors `last_loop_steady_ns_`, which is updated at the top of every control tick.

### Trip sequence

```
frozen_ns = now_ns − last_loop_steady_ns_

if frozen_ns > watchdog_timeout_ns:
    if not latched:
        Set watchdog_tripped_ = true    (control loop reads this on resume)
        Increment watchdog_trip_count_
        Publish wd_stop_msg_ on /control_value   (target_speed_value=0, mode_value="FAULT")
        Publish ERROR on /diagnostics (acc_aeb_watchdog)
        latch = true
    else if (now_ns − last_pub_ns) ≥ refresh_ns:
        Re-publish safe-stop at control rate (not 50 Hz)

else if latched:
    latch = false
    Publish OK on /diagnostics (acc_aeb_watchdog)
```

### Resume after watchdog trip

When the control loop resumes (after a latched trip), `watchdog_tripped_` is consumed and `onSafeStopEngaged()` is called:

- `last_acc_cmd_` ← `a_fault_brake` (jerk baseline reflects actual braking; prevents a stale positive baseline from delaying post-fault braking)
- `integral_cruise_`, `integral_follow_` ← 0
- `act_mode_` ← BRAKE
- `last_pub_torque_` ← 0, `last_pub_brake_` ← safe_brake_g

### Watchdog scope

The watchdog detects a **hung control thread while other threads still run**. It cannot detect:
- A whole-process freeze or kernel panic
- Power loss
- Silent hardware failure

**The CAN gateway must independently time out `/control_value`** and command a safe state if the topic goes stale. The node's shutdown safe-stop is defence-in-depth, not the primary safety net.

---

## 12. Topics

| Direction | Topic | Type | Notes |
|---|---|---|---|
| Subscribe | `/VCU_Data` | `pro_can/VCU` | `MotorVelocity` field (float, km/h). Non-finite values rejected; VCU freshness clock not refreshed on reject. |
| Subscribe | `/perception/camera/tracked_objects` | `npust_bus_msgs/TrackedObjectArray` | Camera object list. Stale after `sensor_timeout_s`. |
| Subscribe | `/perception/radar/tracked_objects` | `npust_bus_msgs/TrackedObjectArray` | Radar object list. Stale after `radar_timeout_s`. Single sensor loss = degraded, not FAULT. |
| Subscribe | `/perception/lanes/left` | `npust_bus_msgs/LanePolynomial` | Left lane boundary polynomial. Validated: `|a| ≤ max_lane_curve_a`, `|b| ≤ max_lane_heading_b`. |
| Subscribe | `/perception/lanes/right` | `npust_bus_msgs/LanePolynomial` | Right lane boundary polynomial. Same validation. |
| Subscribe | `/planner/target_speed` | `std_msgs/Float32` | Target speed in **m/s**, capped to `target_speed_mps`. Non-finite values rejected. |
| Subscribe | `/driver_override` | `std_msgs/Bool` | `true` = driver has manual control. Suspends ACC (CRUISE/FOLLOW → neutral output). AEB/WARN remain armed. **FAULT braking is suppressed when the fault is sensor-driven** (stale perception or VCU) so the driver can operate the bus without autonomous braking fighting them — a WARNING is logged every 5 s that AEB is non-functional until sensors recover. AEB re-arms automatically as soon as sensors return healthy. |
| Publish | `/control_value` | `pro_can::control_data` | `target_speed_value` (m/s, integrated from the jerk-limited accel command) and `mode_value` (current FSM state name, e.g. `"CRUISE"`/`"AEB"` — inspect-only, not load-bearing on the bus side). Replaces the earlier torque/brake output per the bus team's integration requirements; `control_data.msg` no longer has `acc_command_value`/`brake_command_value`/`steering_wheel_command_value` fields. |
| Publish | `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Two status entries: `acc_aeb_controller` (main) and `acc_aeb_watchdog` (watchdog-only, separate so it is visible even when the control loop is dead). |

Use `<remap>` tags in the launch file to adapt to different topic names in your stack.

---

## 13. Parameters

All parameters live in the private namespace (`~`). Defaults are in `types.h::defaults`.

### Control timing

| Parameter | Default | Unit | Description |
|---|---|---|---|
| `dt_s` | 0.05 | s | Control loop period. Sets 20 Hz. Must be strictly positive. |
| `spinner_threads` | 2 | — | `ros::AsyncSpinner` worker count for sensor callbacks. |

### Speed and following

| Parameter | Default | Unit | Description |
|---|---|---|---|
| `target_kph` | 25.0 | km/h | Maximum cruise speed. Capped by `/planner/target_speed` dynamically. |
| `time_gap_s` | 2.0 | s | Desired time headway. `target_dist = ego_v × (time_gap_s + latency_s) + safe_base_m` |
| `safe_base_m` | 5.0 | m | Minimum standstill gap component. |
| `latency_s` | 0.5 | s | System latency compensation in gap formula. |

### AEB / WARN thresholds

| Parameter | Default | Unit | Description |
|---|---|---|---|
| `ttc_aeb_s` | 1.5 | s | TTC threshold to trigger AEB (with confirm gate). |
| `ttc_warn_s` | 3.0 | s | TTC threshold to enter WARN. Must be > `ttc_aeb_s`. |
| `ttc_warn_exit_s` | 3.8 | s | TTC hysteresis exit for WARN. Must be > `ttc_warn_s`. |
| `aeb_dist_m` | 8.0 | m | Range threshold to trigger AEB (N frames confirm gate). |
| `aeb_immediate_dist_m` | 4.0 | m | Range threshold for immediate AEB (no confirm gate). Must be < `aeb_dist_m`. |
| `aeb_backstop_base_m` | 20.0 | m | Minimum backstop range for stationary-object TTC. |
| `resume_dist_m` | 12.0 | m | MIO must exceed this range before AEB hold releases. Must be > `aeb_dist_m`. |
| `aeb_hold_s` | 2.0 | s | How long AEB keeps braking after ego stops. |
| `confirm_frames` | 3 | frames | New-ID confirm gate for normal tracking. |
| `aeb_confirm_frames` | 2 | frames | Confirm gate for AEB trigger (shorter = faster response). |

### Acceleration limits

| Parameter | Default | Unit | Description |
|---|---|---|---|
| `a_acc_max` | 1.0 | m/s² | Maximum commanded acceleration (must be > 0). |
| `a_soft_brake` | -1.5 | m/s² | Soft braking limit for CRUISE/FOLLOW (must be < 0). |
| `a_warn_brake` | -2.5 | m/s² | WARN fixed deceleration (must be < 0). |
| `a_fault_brake` | -2.5 | m/s² | FAULT fixed deceleration (must be < 0). |
| `a_aeb_max` | -5.5 | m/s² | AEB maximum deceleration (must be < 0). |

### Jerk limits

| Parameter | Default | Unit | Description |
|---|---|---|---|
| `jerk_normal` | 2.0 | m/s³ | Normal-speed acceleration jerk limit. |
| `jerk_low_speed` | 1.0 | m/s³ | Jerk limit below `low_speed_kph` (smoother at low speed). |
| `low_speed_kph` | 20.0 | km/h | Speed below which `jerk_low_speed` is used. |
| `jerk_warn` | 4.0 | m/s³ | Deceleration jerk limit for WARN and FAULT states (safety-critical, faster response). |

### Actuator hysteresis

| Parameter | Default | Unit | Description |
|---|---|---|---|
| `throttle_enter_mps2` | 0.15 | m/s² | Accel above this threshold → switch COAST→THROTTLE (after coast dwell). |
| `throttle_dead_mps2` | 0.05 | m/s² | Accel below this → exit THROTTLE. Must be < `throttle_enter_mps2`. |
| `brake_enter_mps2` | -0.15 | m/s² | Accel below this → switch COAST→BRAKE (after coast dwell). |
| `brake_dead_mps2` | -0.05 | m/s² | Accel above this → exit BRAKE. `|brake_dead| < |brake_enter|`. |
| `min_coast_s_normal` | 0.30 | s | Coast dwell before mode switching (CRUISE/FOLLOW). |
| `min_coast_s_warn` | 0.10 | s | Shorter coast dwell in WARN/FAULT (faster brake entry). |
| `torque_rate_limit` | 2000 | Nm/s | Maximum rate of torque change. |
| `brake_rate_limit` | 2.0 | g/s | Maximum rate of brake change. |

### Vehicle / drivetrain // recheck w/ real info****

| Parameter | Default | Unit | Description |
|---|---|---|---|
| `mass_kg` | 10000 | kg | Vehicle mass. |
| `wheel_r_m` | 0.478 | m | Wheel radius. |
| `gear_ratio` | 6.5 | — | Drivetrain gear ratio. |
| `drivetrain_eff` | 0.92 | — | Drivetrain efficiency (0–1]. |
| `motor_torque_max` | 1200 | Nm | Motor torque ceiling. |
| `brake_g_max` | 0.60 | g | Brake command ceiling. |

### Control gains

| Parameter | Default | Description |
|---|---|---|
| `kp_cruise` | 0.50 | CRUISE proportional gain on speed error (m/s² per m/s) |
| `ki_cruise` | 0.01 | CRUISE integral gain |
| `cruise_iclamp` | 3.0 | CRUISE integral clamp (m/s²) |
| `kp_follow_dist` | 0.12 | FOLLOW proportional gain on distance error (m/s² per m) |
| `kd_follow_vrel` | 0.45 | FOLLOW derivative gain on relative velocity (m/s² per m/s) |
| `ki_follow` | 0.005 | FOLLOW integral gain |
| `kf_follow_arel` | 0.20 | FOLLOW feedforward gain on relative acceleration |
| `follow_iclamp` | 5.0 | FOLLOW integral clamp (m/s²) |

### Sensor timeouts and fault logic

| Parameter | Default | Unit | Description |
|---|---|---|---|
| `sensor_timeout_s` | 0.30 | s | Camera message freshness. Stale = degraded if radar is fresh; FAULT if radar also stale. |
| `radar_timeout_s` | 0.30 | s | Radar message freshness. Stale = degraded if camera is fresh. |
| `vcu_timeout_s` | 0.30 | s | VCU (ego speed) freshness. Stale always triggers FAULT. |
| `fault_recovery_s` | 1.0 | s | Duration of clean sensors required before exiting FAULT. |
| `watchdog_timeout_s` | 0.20 | s | Control-loop freeze detection threshold. |

### Perception parameters

| Parameter | Default | Description |
|---|---|---|
| `vx_is_relative` | false | **Must match camera tracker.** `true` = `obj.vx` is already relative to ego. `false` = world-frame absolute (ego_v subtracted internally). Wrong setting silently corrupts TTC. |
| `radar_vx_is_relative` | true | **Must match radar.** Doppler velocity is relative; almost always `true`. |
| `max_range_m` | 120.0 | Objects beyond this range are ignored. |
| `min_valid_x_m` | 0.5 | Objects closer than this are ignored (sensor blind zone). |
| `static_half_w_m` | 1.75 | Ego-lane lateral half-width for object association when lane polynomials are unavailable. |
| `max_lane_width_m` | 5.0 | Maximum accepted lane width from polynomial; wider → fall back to half-width. |
| `max_lane_curve_a` | 0.02 | Maximum curvature coefficient for a lane polynomial to be considered valid. |
| `max_lane_heading_b` | 0.35 | Maximum heading offset for a lane polynomial to be considered valid. |
| `lane_narrow_rate` | 0.02 | Rate at which `dynamic_half_w` shrinks with speed (m per m/s). |
| `min_static_half_w_m` | 1.0 | Floor for `dynamic_half_w`. |
| `mio_weight_dist` | 10.0 | Threat score weight for range term. |
| `mio_weight_ttc` | 5.0 | Threat score weight for TTC term. |
| `follow_enter_ratio` | 1.5 | FOLLOW entered when `mio.x ≤ target_dist × ratio` (must be > 1). |
| `follow_exit_ratio` | 1.8 | FOLLOW exited when `mio.x > target_dist × ratio` (must be > `follow_enter_ratio`). |
| `target_loss_timeout_s` | 1.0 | Grace coast duration before KF reset on target loss. |
| `loss_hold_decel` | -0.5 | Decel applied during FOLLOW grace coast (m/s²). |
| `stop_speed_mps` | 0.30 | Ego speed below which AEB hold activates (m/s). |
| `stationary_v_abs_max` | 0.5 | Object `|v_abs|` below this is treated as parked for backstop TTC. |
| `min_closing_speed_mps` | 0.01 | Minimum closing speed for linear TTC computation. |
| `v_rel_plausible_max` | 70.0 | Objects with implausible relative velocity are rejected. |
| `ego_v_moving_thres` | 0.1 | Ego speed below which stationary-object backstop is suppressed (m/s). |
| `min_stale_x_m` | 1.0 | Minimum range for a grace-propagated stale MIO. |

### Radar fusion

| Parameter | Default | Description |
|---|---|---|
| `radar_fusion_dedup_m` | 2.0 | XY radius for camera-radar deduplication. Objects within this distance of a camera detection are not scored separately from radar. |
| `fusion_gate_m` | 3.0 | Maximum `|cam.x − rad.x|` before the radar KF update is skipped for that cycle. Prevents a guardrail detection from corrupting the car-tracking filter. |

### Kalman filter process noise

| Parameter | Default | Description |
|---|---|---|
| `q_x` | 0.5 | Process noise scalar for range (m²/s⁵). Increase if filter lags sudden braking. |
| `q_v` | 0.15 | Process noise scalar for velocity (m²/s³). |
| `q_a` | 0.05 | Process noise scalar for acceleration (m²/s). |

### Camera health and cut-in

| Parameter | Default | Description |
|---|---|---|
| `cam_health_timeout_s` | 30.0 | If camera sends empty arrays while ego is moving for longer than this, emit `ROS_WARN`. Not a FAULT (clear road is valid); diagnostic only. |
| `cut_in_lateral_factor` | 2.0 | Adjacent-zone outer edge = `factor × dynamic_half_w`. Must be > 1.0. |

### Debug / test

| Parameter | Default | Description |
|---|---|---|
| `debug_stall_once_s` | (unset) | **Test only.** Injects a one-shot control-loop stall of this duration to exercise the watchdog. Leave unset on the vehicle. |

---

## 14. Diagnostics

The node publishes `diagnostic_msgs/DiagnosticArray` on `/diagnostics` every tick with two named status entries.

### `acc_aeb_controller` status

| Level | Condition |
|---|---|
| OK | State is CRUISE or FOLLOW |
| WARN | State is WARN |
| ERROR | State is AEB or FAULT |

Key-value fields (all `double` formatted to 6 decimal places):

| Key | Description |
|---|---|
| `target_kph` | Current dynamic target speed (km/h) |
| `ego_kph` | Ego speed from VCU (km/h) |
| `mio_x_m` | Tracked MIO range (m); `-1` if no valid MIO |
| `closing_mps` | Closing speed to MIO (m/s) |
| `ttc_s` | TTC (s); capped at 99.9 for display |
| `cmd_mps2` | Jerk-limited longitudinal acceleration command (m/s²) |
| `lane_l_valid` | 1.0 if left lane polynomial is valid and fresh; 0.0 otherwise |
| `lane_r_valid` | 1.0 if right lane polynomial is valid and fresh; 0.0 otherwise |
| `dynamic_half_w` | Current ego-lane lateral half-width (m) |
| `watchdog_trips` | Total watchdog trip count since startup |
| `fault_source` | `none` / `perception_stale` / `vcu_stale` / `perception+vcu_stale` |
| `loop_overruns` | Count of ticks where wall time exceeded `1.5 × dt_s` |
| `cam_ok` | 1.0 if camera is fresh; 0.0 if stale |
| `rad_ok` | 1.0 if radar is fresh; 0.0 if stale |
| `driver_override` | 1.0 if driver override is active |
| `cam_content_ok` | 1.0 if camera content is healthy; 0.0 if empty arrays while moving > `cam_health_timeout_s` |
| `adj_mio_x_m` | Adjacent-lane MIO range (m); `-1` if none |
| `adj_ttc_s` | Adjacent-lane MIO TTC (s); 99.9 if none |

### `acc_aeb_watchdog` status

Published by the watchdog thread — visible even when the control loop is frozen.

| Level | Condition |
|---|---|
| ERROR | Control loop frozen — hardware safe-stop engaged |
| OK | Control loop resumed after freeze |

| Key | Description |
|---|---|
| `frozen_for_s` | Duration of current freeze (s) |
| `trip_count` | Total watchdog trip count |

---

## 15. Build and Run

### Prerequisites

- ROS 1 Noetic
- `pro_can` package (`control_data.msg`, `VCU.msg`) on the workspace path
- `npust_bus_msgs` package (`TrackedObjectArray.msg`, `LanePolynomial.msg`) on the workspace path

### Build

```bash
cd ~/bus_ws
catkin_make          # or: catkin build
source devel/setup.bash
```

### Run (vehicle deployment)

```bash
roslaunch acc_aeb_controller acc_aeb_controller.launch
```

### Live camera test (real YOLO objects + simulated vehicle dynamics)

Requires:
- `roscore` on 192.168.0.250
- YOLO perception team publishing `/perception/camera/tracked_objects` from 192.168.0.105

```bash
roslaunch acc_aeb_controller live_test.launch
roslaunch acc_aeb_controller live_test.launch init_speed_kph:=30.0
```

This launches the full controller plus [vehicle_plant.py](#vehicle_plantpy), which integrates `/control_value` → `/VCU_Data` and publishes static flat lane stubs. Objects come from the real camera.

---

## 16. Test Suite

### Unit tests

Located in [test/unit_test.cpp](test/unit_test.cpp). Run with:

```bash
catkin_make run_tests_acc_aeb_controller
# or:
rostest acc_aeb_controller unit_test.launch   # (if a test launch exists)
```

Tests cover:

| Test group | What is verified |
|---|---|
| `SelectMio.*` | Both sensors timed out → invalid; camera in-lane; object outside lane rejected; radar fills camera stale gap; dedup skips sub-threshold camera return |
| `MioTracker.*` | Grace v_abs consistent with v_rel; grace v_abs reflects current ego_v |
| `StateMachine.*` | FAULT reaches full brake at `jerk_warn` rate; WARN and FAULT use identical jerk rate; AEB command is immediate (no jerk limit); sensor-OK prevents immediate FAULT |
| `Kinematics.*` | Stationary-object backstop triggers AEB-level TTC; linear TTC fallback when `a_rel` is zero |
| `KalmanTest.*` | Predict closes range; update pulls toward measurement; radar-only converges in velocity; covariance diagonal stays positive |
| `FusionGate.*` | Agreement fuses both sensors; mismatch skips radar; radar-only always updates; zero-variance fallback produces no NaN |
| `SelectAdjacentMio.*` | In-lane object not reported as adjacent; object beyond cut-in zone ignored; adjacent object detected; radar fills adjacent zone when camera stale |
| `Efficiency.*` | 2000 full-pipeline iterations complete in < 5 ms average (budget << 1 ms in practice) |

Tests are **ROS-free** for the core logic layers (`types.h`, `math_utils.h`, `kinematics.*`, `state_machine.*`). Only `ros::Time::init()` is called for `ROS_WARN_THROTTLE` in `kinematics.cpp`.

### Closed-loop bench testing

There is no dedicated `acc_aeb_sim` package in this workspace. For closed-loop testing without a real VCU or CAN gateway, use [live_test.launch](#build-and-run) together with [vehicle_plant.py](#vehicle_plantpy), which integrates the published `/control_value` back into `/VCU_Data` through simple 1D vehicle dynamics — see [Developer Tools](#17-developer-tools). Drive scenarios (free cruise, following, AEB stop, sensor dropouts) by publishing synthetic camera/radar objects and lane polynomials, or by using [fake_vcu.py](#fake_vcupy) to control ego speed directly.

---

## 17. Developer Tools

### `fake_vcu.py`

Interactive ego-speed publisher for bench testing without a real VCU.

```bash
rosrun acc_aeb_controller fake_vcu.py
```

Commands:

| Input | Action |
|---|---|
| `<number>` | Set speed to N km/h (e.g. `30`) |
| `+` | Increase speed by 5 km/h |
| `-` | Decrease speed by 5 km/h |
| `0` or `stop` | Set speed to 0 |
| `q` | Quit |

Publishes at 10 Hz on `/VCU_Data`.

### `monitor.py`

Live terminal dashboard showing raw sensor inputs and the controller's KF output.

```bash
rosrun acc_aeb_controller monitor.py
```

Displays (refreshed at 4 Hz):

- Camera object list: ID, x, y, vx, x_var, y_var, vx_var (up to 8 objects)
- Radar object list: same fields
- Controller KF state: ego speed, target speed, MIO range, closing speed, TTC, accel command, adjacent MIO
- Sensor health flags: `cam_ok`, `rad_ok`, `cam_content_ok`, `driver_override`, `fault_source`

Subscribes to `/perception/camera/tracked_objects`, `/perception/radar/tracked_objects`, `/diagnostics`.

### `vehicle_plant.py`

Simple 1D vehicle dynamics model. Tracks the `target_speed_value` published on `/control_value` and integrates the result into `/VCU_Data`.

```bash
# Usually launched via live_test.launch; can also be run standalone:
rosrun acc_aeb_controller vehicle_plant.py _dt:=0.05 _init_speed_kph:=0.0
```

Physics (P-controlled speed tracking, matching the node's new `target_speed_value`/`mode_value` output):

```
speed_err = target_speed_value − ego_v
accel     = clamp(speed_err × SPEED_P_GAIN, −MAX_DECEL_MPS2, MAX_ACCEL_MPS2)
            (SPEED_P_GAIN=2.0, MAX_ACCEL_MPS2=1.6, MAX_DECEL_MPS2=5.5)

if |speed_err| < 0.05 and ego_v > 0:
    accel = −DRAG_MPS2   (0.08 m/s² — rolling resistance + drag, coast when on-target)

ego_v = max(0, ego_v + accel × dt)
```

This is a plant model standing in for the bus's own speed-tracking control loop — it no longer models torque/brake actuation, since the node no longer publishes those.

---

## 18. Safety Notes

1. **`vx_is_relative` and `radar_vx_is_relative` are safety-critical.** A wrong setting causes `v_rel` to be off by ≈ `2 × ego_v`, silently corrupting TTC. Set these to match each sensor's velocity convention and verify before any road test.

2. **Downstream failsafe is mandatory.** The CAN gateway must command a safe state if `/control_value` goes stale. The node's shutdown safe-stop is defence-in-depth only.

3. **`debug_stall_once_s` must not be set on the vehicle.** It stalls the control thread intentionally to exercise the watchdog on the bench.

4. **`min_valid_x_m` must be set to the sensor's minimum reliable range before deployment.** The default (0.5 m) may pass noise returns in very close range. Do not raise this value to mask false detections — fix the sensor instead.

5. **Node starts in FAULT.** The controller applies `a_fault_brake` deceleration until both VCU and at least one perception sensor have each published at least once, and `fault_recovery_s` has elapsed. Ensure sensors are running before the controller is expected to control motion.

6. **Driver override** (`/driver_override`, `std_msgs/Bool = true`) suspends ACC (CRUISE/FOLLOW → `target_speed_value = ego_v`, i.e. hold current speed). **AEB and WARN always remain armed.** FAULT braking is suppressed when the fault source is stale sensors — this lets the driver operate the bus without autonomous braking fighting them. A `ROS_WARN_THROTTLE(5 s)` is emitted every 5 seconds while this condition holds, stating that AEB is non-functional. AEB re-arms automatically as soon as sensors recover (no driver action required). Wire the override topic from the VCU's brake-pedal or throttle-override signal.

---

## 19. Known Gaps

| Gap | Required work | Priority |
|---|---|---|
| Object class filtering | Use `TrackedObject.type` from the perception team to filter road signs, barriers, and oncoming traffic | High |
| Yaw-rate / curve compensation | IMU subscription; adjust the lane corridor for cornering; currently only static lateral half-width adapts | Medium |
| Speed-limit awareness | Map or sign-recognition input; currently only `/planner/target_speed` caps cruise | Medium |
| VCU brake-pedal field for override | Wire `/driver_override` directly from `VCU.brake_pedal_*`; currently requires a separate Bool topic | Low |

---

## 20. Message Field Assumptions

| Message | Fields used | Notes |
|---|---|---|
| `pro_can/VCU` | `MotorVelocity` (float, km/h) | Non-finite values rejected; freshness clock not refreshed on rejection |
| `pro_can/control_data` | `header`, `target_speed_value` (float64, m/s), `mode_value` (string, FSM state name) | Message no longer carries `acc_command_value`/`brake_command_value`/`steering_wheel_command_value` — see [§12 Topics](#12-topics) |
| `npust_bus_msgs/TrackedObject` | `x`, `y`, `vx` (float64, m and m/s), `id` (int32), `x_var`, `vx_var` (float64, m² and m²/s²) | Zero variance triggers fallback default with throttled warning |
| `npust_bus_msgs/LanePolynomial` | `a`, `b`, `c` (float64, polynomial coefficients), `is_valid` (bool) | Validated: `|a| ≤ max_lane_curve_a`, `|b| ≤ max_lane_heading_b` |

---

## 21. Module Dependency Graph

```
types.h  (ROS-free: POD structs, enums, defaults, consts)
   │
   ├── math_utils.h  (ROS-free: inline helpers, clampVal, safeDiv, kfPredict/kfUpdate)
   │       │
   │       ├── kinematics.h/.cpp  (+ npust_bus_msgs headers, + roscpp for WARN_THROTTLE)
   │       │       selectMIO, selectAdjacentMIO, MioTracker, computeKinematics
   │       │
   │       └── state_machine.h/.cpp  (ROS-free)
   │               StateMachine (FSM, control law, jerk limit, actuator mapping)
   │
   └── params.h/.cpp  (+ roscpp: NodeHandle param loading, validation, shutdown)
           loadParams, validateParams

All of the above ──► acc_aeb_node.cpp  (ROS I/O, threads, watchdog, main)
```

**ROS-free layers** (`types.h`, `math_utils.h`, `state_machine.*`): fully unit-testable without a ROS master.

**Minimal ROS dependency** (`kinematics.*`): only `ros::WallRate`/`ROS_WARN_THROTTLE` — can be tested with `ros::Time::init()` only.

**Full ROS** (`params.*`, `acc_aeb_node.cpp`): require `NodeHandle` / roscore.
