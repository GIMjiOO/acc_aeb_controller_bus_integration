#!/usr/bin/env python3
"""
Vehicle plant model for live-camera testing.

Subscribes to /control_value (controller output) and integrates simple
vehicle dynamics to produce a realistic ego speed on /VCU_Data.
Objects and lane polynomials come from the real YOLO/perception team —
this node does NOT publish them.

Reuses the same physics constants as acc_aeb_sim so the controller sees
a consistent vehicle model whether running scenarios or live camera tests.
"""
import rospy
from pro_can.msg import VCU, control_data

MPS_TO_KPH       = 3.6
MAX_ACCEL_MPS2   = 1.6    # physical ceiling for drive
MAX_DECEL_MPS2   = 5.5    # AEB-level decel ceiling
DRAG_MPS2        = 0.08   # rolling resistance + aero during coast
SPEED_P_GAIN     = 2.0    # P-gain: how aggressively plant chases target_speed


class VehiclePlant:

    def __init__(self):
        self._target_v  = 0.0
        self._mode      = "FAULT"
        self._ego_v     = 0.0

        dt_s            = float(rospy.get_param('~dt', 0.05))
        init_kph        = float(rospy.get_param('~init_speed_kph', 0.0))
        self._ego_v     = init_kph / MPS_TO_KPH
        self._dt        = dt_s

        self._vcu_pub = rospy.Publisher('/VCU_Data', VCU, queue_size=1)

        rospy.Subscriber('/control_value', control_data, self._ctrl_cb)

        rospy.sleep(0.3)
        rospy.loginfo('[PLANT] Vehicle plant ready | init=%.1f km/h | dt=%.3f s',
                      init_kph, dt_s)

    def _ctrl_cb(self, msg):
        self._target_v = msg.target_speed_value
        self._mode     = msg.mode_value

    def _step(self):
        speed_err = self._target_v - self._ego_v
        accel = speed_err * SPEED_P_GAIN
        accel = max(-MAX_DECEL_MPS2, min(MAX_ACCEL_MPS2, accel))
        if abs(speed_err) < 0.05 and self._ego_v > 0.0:
            accel = -DRAG_MPS2   # coast when on-target
        self._ego_v = max(0.0, self._ego_v + accel * self._dt)

    def _pub_vcu(self):
        msg = VCU()
        msg.MotorVelocity = float(self._ego_v * MPS_TO_KPH)
        self._vcu_pub.publish(msg)

    def run(self):
        rate  = rospy.Rate(1.0 / self._dt)
        t     = 0.0
        while not rospy.is_shutdown():
            self._step()
            self._pub_vcu()
            if int(t / self._dt) % 20 == 0:  # status every 1 s
                rospy.loginfo('[PLANT] ego=%.1f km/h  target=%.1f km/h  mode=%s',
                              self._ego_v * MPS_TO_KPH,
                              self._target_v * MPS_TO_KPH,
                              self._mode)
            t    += self._dt
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('vehicle_plant', anonymous=False)
    VehiclePlant().run()
