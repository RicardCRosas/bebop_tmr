#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

from control.bebop_teleop_controller import BebopMovements
from bebop_core.mission_supervisor import MissionSupervisor


class AutonomousMissionManager:
    """
    Nodo autónomo para pruebas sin teleoperación.

    Arranca el supervisor y dispara una misión automáticamente.
    """

    def __init__(self):
        rospy.init_node("autonomous_mission_manager")

        self.pub_cmd = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1)
        self.pub_takeoff = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1)
        self.pub_land = rospy.Publisher('/bebop/land', Empty, queue_size=1)
        self.pub_camera = rospy.Publisher('/bebop/camera_control', Twist, queue_size=1)

        self.movements = BebopMovements(
            self.pub_cmd,
            self.pub_takeoff,
            self.pub_land,
            self.pub_camera
        )

        self.supervisor = MissionSupervisor(self.movements)

        self.mission_name = rospy.get_param("~mission_name", "mission_whiteboard_aruco")
        self.autostart_delay = rospy.get_param("~autostart_delay", 3.0)

        self.started = False
        self.start_time = rospy.Time.now()

        rospy.loginfo("AutonomousMissionManager ready")
        rospy.loginfo(f"mission_name = {self.mission_name}")
        rospy.loginfo(f"autostart_delay = {self.autostart_delay}")

    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if not self.started:
                elapsed = (rospy.Time.now() - self.start_time).to_sec()
                if elapsed >= self.autostart_delay:
                    rospy.loginfo(f"Autostarting mission: {self.mission_name}")
                    self.supervisor.start_mission(self.mission_name)
                    self.started = True

            rate.sleep()


if __name__ == "__main__":
    node = AutonomousMissionManager()
    node.run()