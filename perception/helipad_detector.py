#!/usr/bin/env python3

import rospy
import os
from ultralytics import YOLO
import cv2
import numpy as np

current_dir = os.path.dirname(os.path.abspath(__file__))
model_path = os.path.join(current_dir, 'models', 'helipad_best.pt')


class BebopCameraProcessor:
    def __init__(self):
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"Model not found: {model_path}")

        self.model = YOLO(model_path)
        self.center = (320, 240)

    def detect_with_yolo(self, image):
        results = self.model.predict(source=image, conf=0.5, verbose=False)

        annotated = results[0].plot()

        best_conf = 0
        best_box = None
        cx, cy = None, None

        if results[0].boxes is not None:
            xyxy = results[0].boxes.xyxy.cpu().numpy()
            confs = results[0].boxes.conf.cpu().numpy()

            for box, conf in zip(xyxy, confs):
                if conf < 0.5:
                    continue

                if conf > best_conf:
                    best_conf = conf
                    best_box = box

        if best_box is not None:
            x1, y1, x2, y2 = best_box.astype(int)
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2

            cv2.circle(annotated, (cx, cy), 6, (0, 0, 255), -1)
            detected = True
        else:
            detected = False

        cv2.circle(annotated, self.center, 5, (255, 0, 0), -1)

        return annotated, detected, cx, cy

    def process_image(self, frame):
        h, w = frame.shape[:2]
        self.center = (w // 2, h // 2)

        annotated, detected, cx, cy = self.detect_with_yolo(frame)

        data = {
            "detected": detected,
            "cx": cx,
            "cy": cy,
            "center_x": self.center[0],
            "center_y": self.center[1]
        }

        return annotated, data


# =====================================================
# ROS NODE
# =====================================================

if __name__ == "__main__":
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge

    rospy.init_node("helipad_detector_node")

    bridge = CvBridge()
    detector = BebopCameraProcessor()

    def image_callback(msg):
        try:
            frame = bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"CvBridge error: {e}")
            return

        # 🔥 optimización (clave ahora que quitaste el filtro)
        frame = cv2.resize(frame, (640, 360))

        processed_image, data = detector.process_image(frame)

        cv2.imshow("Helipad Detector", processed_image)
        cv2.waitKey(1)

        if data["detected"]:
            rospy.loginfo(f"Detected | cx: {data['cx']} | cy: {data['cy']}")

    rospy.Subscriber("/bebop/image_raw", Image, image_callback)

    rospy.loginfo("Helipad detector running...")
    rospy.spin()