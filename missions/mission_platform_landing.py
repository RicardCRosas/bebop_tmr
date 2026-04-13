#!/usr/bin/env python3
# mission_platform_landing.py

import rospy
from std_msgs.msg import String

class MissionPlatformLanding:
    def __init__(self):
        rospy.init_node('mission_platform_landing')
        self.status_pub = rospy.Publisher('/mission/status', String, queue_size=1)
        self.finished = False

    def run(self):
        rospy.loginfo("Starting Mission: Land on platform")
        rospy.sleep(2)
        rospy.loginfo("Searching for platform...")
        rospy.sleep(3)
        rospy.loginfo("Landing...")
        rospy.sleep(2)
        self.status_pub.publish("done")
        self.finished = True
        rospy.loginfo("Mission Platform Landing completed")

if __name__ == "__main__":
    mission = MissionPlatformLanding()
    mission.run()
