#!/usr/bin/env python3
"""Interactive fake VCU — publishes /VCU_Data at 10 Hz with manually adjustable speed."""
import sys
import threading
import rospy
from pro_can.msg import VCU

STEP = 5.0       # km/h per +/- step
MAX_KPH = 120.0

speed_kph = 0.0
lock = threading.Lock()
shutdown = threading.Event()


def publish_loop(pub):
    rate = rospy.Rate(10)
    msg = VCU()
    while not rospy.is_shutdown() and not shutdown.is_set():
        with lock:
            msg.MotorVelocity = speed_kph
        pub.publish(msg)
        rate.sleep()


def print_status():
    with lock:
        v = speed_kph
    bar = int(v / MAX_KPH * 30)
    print(f"\r  Speed: {v:6.1f} km/h  [{'#' * bar}{' ' * (30 - bar)}]  ", end='', flush=True)


def main():
    global speed_kph
    rospy.init_node('fake_vcu', anonymous=False)
    pub = rospy.Publisher('/VCU_Data', VCU, queue_size=1)

    t = threading.Thread(target=publish_loop, args=(pub,), daemon=True)
    t.start()

    print("=" * 54)
    print("  Fake VCU — /VCU_Data interactive speed controller")
    print("=" * 54)
    print("  Commands:")
    print("    <number>   set speed in km/h  e.g. 30")
    print(f"    +          increase {STEP:.0f} km/h")
    print(f"    -          decrease {STEP:.0f} km/h")
    print("    0 / stop   set to 0 km/h")
    print("    q          quit")
    print("-" * 54)

    while not rospy.is_shutdown():
        print_status()
        try:
            cmd = input().strip().lower()
        except (EOFError, KeyboardInterrupt):
            break

        if cmd in ('q', 'quit'):
            break
        elif cmd in ('+', '++'):
            with lock:
                speed_kph = min(MAX_KPH, speed_kph + STEP)
        elif cmd in ('-', '--'):
            with lock:
                speed_kph = max(0.0, speed_kph - STEP)
        elif cmd in ('0', 'stop'):
            with lock:
                speed_kph = 0.0
        else:
            try:
                v = float(cmd)
                with lock:
                    speed_kph = max(0.0, min(MAX_KPH, v))
            except ValueError:
                print(f"\n  Unknown command: '{cmd}'")

    shutdown.set()
    print(f"\n  Stopped. Final speed was {speed_kph:.1f} km/h")


if __name__ == '__main__':
    main()
