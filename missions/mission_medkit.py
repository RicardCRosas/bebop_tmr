#!/usr/bin/env python3
# mission_medkit.py

import rospy
from std_msgs.msg import String

class MissionMedkit:
    def __init__(self):
        rospy.init_node('mission_medkit')
        self.status_pub = rospy.Publisher('/mission/status', String, queue_size=1)
        self.finished = False

    def run(self):
        rospy.loginfo("Starting Mission: Find cry for help & deliver med-kit")
        rospy.sleep(2)
        rospy.loginfo("Finding black box...")
        rospy.sleep(3)
        rospy.loginfo("Delivering medkit...")
        rospy.sleep(2)
        self.status_pub.publish("done")
        self.finished = True
        rospy.loginfo("Mission Medkit completed")

if __name__ == "__main__":
    mission = MissionMedkit()
    mission.run()
