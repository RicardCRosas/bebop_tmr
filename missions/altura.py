#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
from geometry_msgs.msg import Twist
from bebop_msgs.msg import Ardrone3PilotingStateAltitudeChanged


class HoldAltitude:
    def __init__(self):
        rospy.init_node("hold_altitude_15m")

        self.cmd_pub = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber(
            "/bebop/states/ardrone3/PilotingState/AltitudeChanged",
            Ardrone3PilotingStateAltitudeChanged,
            self.alt_callback
        )

        self.target_z = rospy.get_param("~target_z", 1.5)
        self.kp_z = rospy.get_param("~kp_z", 0.8)
        self.vz_max = rospy.get_param("~vz_max", 0.15)
        self.ctrl_rate = rospy.get_param("~ctrl_rate", 20.0)

        self.current_z = None

    def alt_callback(self, msg):
        self.current_z = msg.altitude

    def saturate(self, val, limit):
        return max(min(val, limit), -limit)

    def run(self):
        rate = rospy.Rate(self.ctrl_rate)

        while not rospy.is_shutdown():
            if self.current_z is None:
                rate.sleep()
                continue

            error_z = self.target_z - self.current_z
            vz = self.kp_z * error_z
            vz = self.saturate(vz, self.vz_max)

            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.linear.z = vz
            cmd.angular.z = 0.0

            self.cmd_pub.publish(cmd)

            rospy.loginfo_throttle(
                0.5,
                "z=%.3f | z_ref=%.3f | ez=%.3f | vz=%.3f",
                self.current_z, self.target_z, error_z, vz
            )

            rate.sleep()


if __name__ == "__main__":
    try:
        HoldAltitude().run()
    except rospy.ROSInterruptException:
        pass
