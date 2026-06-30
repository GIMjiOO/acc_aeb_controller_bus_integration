#!/usr/bin/env python3
"""
monitor.py — Live display of camera raw, radar raw, and controller KF result.
Run:  rosrun acc_aeb_controller monitor.py
"""
import os, sys, threading, time
import rospy
from diagnostic_msgs.msg import DiagnosticArray
from npust_bus_msgs.msg  import TrackedObjectArray

# ── thread-safe state ──────────────────────────────────────────────────────────
_lock = threading.Lock()

_cam  = []   # list of TrackedObject
_rad  = []
_diag = {}   # key → value string

_cam_stamp  = None
_rad_stamp  = None
_diag_stamp = None

# ── subscribers ───────────────────────────────────────────────────────────────
def cb_cam(msg):
    global _cam, _cam_stamp
    with _lock:
        _cam = list(msg.objects)
        _cam_stamp = rospy.Time.now()

def cb_rad(msg):
    global _rad, _rad_stamp
    with _lock:
        _rad = list(msg.objects)
        _rad_stamp = rospy.Time.now()

def cb_diag(msg):
    global _diag, _diag_stamp
    for st in msg.status:
        if st.name in ("acc_aeb_controller", "acc_aeb_watchdog"):
            with _lock:
                _diag = {kv.key: kv.value for kv in st.values}
                _diag["__level__"]   = st.level
                _diag["__message__"] = st.message
                _diag_stamp = rospy.Time.now()
            if st.name == "acc_aeb_controller":
                return  # prefer controller over watchdog

# ── helpers ───────────────────────────────────────────────────────────────────
def age(stamp):
    if stamp is None:
        return "never"
    dt = (rospy.Time.now() - stamp).to_sec()
    return f"{dt:.2f}s ago"

def diag_f(key, default="-"):
    return _diag.get(key, default)

LEVEL_STR = {0: "OK   ", 1: "WARN ", 2: "ERROR", 3: "STALE"}

def obj_lines(objs, label):
    lines = []
    lines.append(f"  {'ID':>4}  {'x':>7}  {'y':>7}  {'vx':>7}  {'x_var':>7}  {'y_var':>7}  {'vx_var':>7}")
    lines.append("  " + "-"*64)
    if not objs:
        lines.append("  (no objects)")
    else:
        for o in objs[:8]:   # cap at 8 rows
            xv  = getattr(o, 'x_var',  float('nan'))
            yv  = getattr(o, 'y_var',  float('nan'))
            vxv = getattr(o, 'vx_var', float('nan'))
            lines.append(
                f"  {o.id:>4}  {o.x:>7.2f}  {o.y:>7.2f}  {o.vx:>7.2f}"
                f"  {xv:>7.3f}  {yv:>7.3f}  {vxv:>7.3f}"
            )
        if len(objs) > 8:
            lines.append(f"  ... (+{len(objs)-8} more)")
    return lines

# ── render ────────────────────────────────────────────────────────────────────
def render():
    with _lock:
        cam  = list(_cam)
        rad  = list(_rad)
        diag = dict(_diag)
        cs   = _cam_stamp
        rs   = _rad_stamp
        ds   = _diag_stamp

    os.system("clear")

    # ── header ──
    now = time.strftime("%H:%M:%S")
    print(f"┌─────────────────────────────── ACC/AEB MONITOR  {now} ───────────────────────────────┐")

    # ── camera ──
    print(f"│ CAMERA  ({len(cam)} objects)   last msg: {age(cs):<12}")
    for l in obj_lines(cam, "cam"):
        print(f"│ {l}")
    print("│")

    # ── radar ──
    print(f"│ RADAR   ({len(rad)} objects)   last msg: {age(rs):<12}")
    for l in obj_lines(rad, "rad"):
        print(f"│ {l}")
    print("│")

    # ── controller KF result ──
    lvl = diag.get("__level__", -1)
    state_str = diag.get("__message__", "?")
    lvl_label = LEVEL_STR.get(lvl, "?    ")

    print(f"│ CONTROLLER KF OUTPUT   last msg: {age(ds):<12}   [{lvl_label}] {state_str}")
    print("│  ┌──────────────────────────────────────────────────────────────────────┐")

    cam_ok      = diag_f("cam_ok")
    rad_ok      = diag_f("rad_ok")
    cam_content = diag_f("cam_content_ok")
    override    = diag_f("driver_override")
    fault_src   = diag_f("fault_source", "none")

    ego_kph   = diag_f("ego_kph")
    tgt_kph   = diag_f("target_kph")
    mio_x     = diag_f("mio_x_m")
    closing   = diag_f("closing_mps")
    ttc       = diag_f("ttc_s")
    cmd       = diag_f("cmd_mps2")
    adj_x     = diag_f("adj_mio_x_m")
    adj_ttc   = diag_f("adj_ttc_s")

    print(f"│  │  cam_ok={cam_ok:<4}  rad_ok={rad_ok:<4}  cam_content={cam_content:<4}  override={override:<4}  fault_src={fault_src}")
    print(f"│  │  ego={ego_kph:<7} km/h   target={tgt_kph:<7} km/h")
    print(f"│  │  MIO x={mio_x:<8} m    closing={closing:<8} m/s    TTC={ttc:<7} s")
    print(f"│  │  accel_cmd={cmd:<8} m/s²")
    print(f"│  │  adj_MIO x={adj_x:<8} m    adj_TTC={adj_ttc:<7} s")
    print("│  └──────────────────────────────────────────────────────────────────────┘")
    print("└─────────────────────────────────────────────────────────────────────────────────────────┘")
    print("  Ctrl-C to quit")

# ── main ──────────────────────────────────────────────────────────────────────
def main():
    rospy.init_node("acc_aeb_monitor", anonymous=True)

    rospy.Subscriber("/perception/camera/tracked_objects",
                     TrackedObjectArray, cb_cam,  queue_size=1)
    rospy.Subscriber("/perception/radar/tracked_objects",
                     TrackedObjectArray, cb_rad,  queue_size=1)
    rospy.Subscriber("/diagnostics",
                     DiagnosticArray,   cb_diag, queue_size=1)

    print("acc_aeb monitor started — waiting for data...")
    rate = rospy.Rate(4)   # refresh 4 Hz
    try:
        while not rospy.is_shutdown():
            render()
            rate.sleep()
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
