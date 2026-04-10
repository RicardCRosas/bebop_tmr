#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


# =========================
# TUS FUNCIONES (sin tocar mucho)
# =========================

def filtropasa_rgb(frame, r, g, b, tolerancia=15):
    color_bgr = np.uint8([[[b, g, r]]])
    hsv_target = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2HSV)[0][0]
    
    bajo = np.array([max(0, hsv_target[0] - tolerancia), 50, 50])
    alto = np.array([min(180, hsv_target[0] + tolerancia), 255, 255])
    
    mascara = cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), bajo, alto)
    return cv2.bitwise_and(frame, frame, mask=mascara)


def centros_por_momentos(frame, area_minima=500):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    canny = cv2.Canny(gray, 10, 150)
    canny = cv2.dilate(canny, None, iterations=1)
    canny = cv2.erode(canny, None, iterations=1)
    
    contornos, _ = cv2.findContours(canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    output = frame.copy()
    detected = False
    cx_final, cy_final = None, None

    for c in contornos:
        if cv2.contourArea(c) >= area_minima:
            M = cv2.moments(c)
            
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
            else:
                x, y, w, h = cv2.boundingRect(c)
                cx, cy = x + w//2, y + h//2
            
            detected = True
            cx_final, cy_final = cx, cy
            
            cv2.circle(output, (cx, cy), 4, (0, 0, 255), -1)
            cv2.circle(output, (cx, cy), 2, (255, 255, 255), -1)
            cv2.drawContours(output, [c], -1, (0, 255, 0), 1)
    
    return output, detected, cx_final, cy_final


# =========================
# ROS NODE
# =========================

class ColorDetectorROS:
    def __init__(self):
        self.bridge = CvBridge()

        # 🎯 Color objetivo
        self.r, self.g, self.b = 106, 174, 239
        self.tolerancia = 15

        rospy.Subscriber("/bebop/image_raw", Image, self.image_callback)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"CvBridge error: {e}")
            return

        # 🔥 optimización
        frame = cv2.resize(frame, (640, 360))

        # 1. Filtro
        filtrado = filtropasa_rgb(frame, self.r, self.g, self.b, self.tolerancia)

        # 2. Centros
        resultado, detected, cx, cy = centros_por_momentos(filtrado)

        # Mostrar
        cv2.imshow("Filtro + Centros (ROS)", resultado)
        cv2.waitKey(1)

        if detected:
            rospy.loginfo(f"Detected | cx: {cx} | cy: {cy}")


# =========================
# MAIN
# =========================

if __name__ == "__main__":
    rospy.init_node("color_detector_node")

    detector = ColorDetectorROS()

    rospy.loginfo("Color detector running...")
    rospy.spin()

    cv2.destroyAllWindows()