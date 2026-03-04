#!/usr/bin/env python3
#mission_square.py

import rospy
from std_msgs.msg import String

import os
import sys

current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, '..'))
sys.path.append(project_root)

def main():
    rospy.init_node("mission_square")

    status_pub = rospy.Publisher('/mission/status', String, queue_size=1)


    # Lógica de misión aquíPaula
    rospy.sleep(8.0)

    # Cuando termina condición interna:
    #status_pub.publish("done")

if __name__ == "__main__":
    main()
