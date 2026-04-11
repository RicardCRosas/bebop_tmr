#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import rospy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.append(current_dir)

from perception.aruco_whiteboard_detector import ArucoWhiteboardDetector


class TestArucoOnly:
    def __init__(self):
        rospy.init_node("test_aruco_only")

        self.bridge = CvBridge()
        self.detector = ArucoWhiteboardDetector()
        self.latest_msg = None

        rospy.Subscriber(
            "/bebop/image_raw",
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2**24
        )

        rospy.loginfo("Nodo test_aruco_only iniciado")
        rospy.loginfo("Esperando imagen en /bebop/image_raw ...")

    def image_callback(self, msg):
        self.latest_msg = msg

    def run(self):
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            if self.latest_msg is not None:
                try:
                    frame = self.bridge.imgmsg_to_cv2(self.latest_msg, "bgr8")
                    frame = cv2.resize(frame, (640, 360))

                    processed, data = self.detector.process_image(frame)

                    if data["detected"]:
                        rospy.loginfo_throttle(
                            1.0,
                            f"ArUco detectado | id={data['marker_id']} | "
                            f"cx={data['cx']} cy={data['cy']} | "
                            f"err_x={data['error_x']} err_y={data['error_y']} | "
                            f"area={int(data['area'])}"
                        )
                    else:
                        rospy.loginfo_throttle(1.0, "Aruco objetivo no detectado")

                    cv2.imshow("aruco_test", processed)
                    cv2.waitKey(1)

                except Exception as e:
                    rospy.logerr_throttle(1.0, f"Error procesando imagen: {e}")

            rate.sleep()

        cv2.destroyAllWindows()


if __name__ == "__main__":
    node = TestArucoOnly()
    node.run()
