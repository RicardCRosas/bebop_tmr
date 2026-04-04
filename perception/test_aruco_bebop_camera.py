#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import rospy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, '..'))
if project_root not in sys.path:
    sys.path.append(project_root)

from perception.aruco_whiteboard_detector import ArucoWhiteboardDetector


class TestArucoBebopCamera:
    def __init__(self):
        rospy.init_node("test_aruco_bebop_camera")

        self.bridge = CvBridge()
        self.detector = ArucoWhiteboardDetector()

        self.image_sub = rospy.Subscriber(
            "/bebop/image_raw",
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2**24
        )

        rospy.loginfo("Nodo de prueba ArUco desde cámara del Bebop iniciado")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Error convirtiendo imagen: {e}")
            return

        frame = cv2.resize(frame, (640, 360))
        processed, data = self.detector.process_image(frame)

        estado = f"detected={data['detected']} id={data['marker_id']} area={int(data['area'])}"
        cv2.putText(processed, estado, (20, 340),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        cv2.imshow("Bebop ArUco Test", processed)
        cv2.waitKey(1)

    def run(self):
        rospy.spin()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    node = TestArucoBebopCamera()
    node.run()