#!/usr/bin/env python3
# mission_aruco.py

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image


class BebopMission:

    def __init__(self):

        rospy.init_node("bebop_mission_full")

        self.pub = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=1)
        self.pub_cam = rospy.Publisher("/bebop/camera_control", Twist, queue_size=1)

        rospy.Subscriber("/bebop/image_raw", Image, self.image_callback)

        self.bridge = CvBridge()
        self.frame = None

        # 🔥 NUEVO ESTADO INICIAL (SUBIR)
        self.state = "TAKEOFF_UP"
        self.start_time = rospy.Time.now()

        # Ajustar cámara
        self.set_camera()

    # =============================
    # 🎥 CÁMARA
    # =============================

    def set_camera(self):
        """
        Cámara casi horizontal para ver ArUco a 1.6 m
        """
        msg = Twist()
        msg.angular.y = -8   # 🔧 entre -5 y -10 ideal
        msg.angular.z = 0
        self.pub_cam.publish(msg)
        rospy.sleep(1)

    # =============================
    # 📷 IMAGEN
    # =============================

    def image_callback(self, msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except:
            self.frame = None

    # =============================
    # ⏱ TIEMPO
    # =============================

    def elapsed(self):
        return (rospy.Time.now() - self.start_time).to_sec()

    def set_state(self, s):
        rospy.loginfo(f"STATE → {s}")
        self.state = s
        self.start_time = rospy.Time.now()
        self.stop()
        rospy.sleep(0.3)

    # =============================
    # 🚁 MOVIMIENTO
    # =============================

    def send(self, lx=0, ly=0, az=0, lz=0):
        msg = Twist()
        msg.linear.x = lx
        msg.linear.y = ly
        msg.linear.z = lz  # 🔥 altura
        msg.angular.z = az
        self.pub.publish(msg)

    def stop(self):
        self.send(0, 0, 0, 0)

    # =====================================================
    # 🆙 SUBIR ALTURA (~1.5 m)
    # =====================================================

    def takeoff_up(self):
        """
        Sube para alinear con ArUco (1.6 m)

        🔧 AJUSTAR:
        3.0 s ≈ ~1.5 m
        """
        if self.elapsed() < 3.0:
            self.send(lz=0.15)
        else:
            self.set_state("MOVE_LEFT")

    # =====================================================
    # 🧭 NAVEGACIÓN
    # =====================================================

    def move_left(self):
        if self.elapsed() < 1.5:
            self.send(ly=0.2)
        else:
            self.set_state("FORWARD")

    def forward(self):
        if self.elapsed() < 6.0:
            self.send(lx=0.2)
        else:
            self.set_state("TURN")

    def turn(self):
        if self.elapsed() < 2.0:
            self.send(az=-0.3)  # derecha = negativo
        else:
            self.set_state("SEARCH")

    # =====================================================
    # 🚧 DETECCIÓN NARANJA
    # =====================================================

    def detect_orange(self):

        if self.frame is None:
            return None

        hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)

        lower = np.array([5, 120, 120])
        upper = np.array([20, 255, 255])

        mask = cv2.inRange(hsv, lower, upper)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None

        c = max(contours, key=cv2.contourArea)

        if cv2.contourArea(c) < 800:
            return None

        x, y, w, h = cv2.boundingRect(c)
        return x + w // 2

    # =====================================================
    # 🧠 DETECCIÓN ARUCO
    # =====================================================

    def detect_aruco(self):

        if self.frame is None:
            return None

        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)

        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
        params = cv2.aruco.DetectorParameters()

        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=params)

        if ids is None:
            return None

        c = corners[0][0]
        cx = int(c[:, 0].mean())
        area = cv2.contourArea(c)

        return cx, area

    # =====================================================
    # 🔍 SEARCH + AVOID
    # =====================================================

    def search(self):

        aruco = self.detect_aruco()

        if aruco is not None:
            self.set_state("ALIGN")
            return

        obs = self.detect_orange()

        if obs is None:
            self.send(lx=0.12, az=0.1)
            return

        center = obs
        img_center = self.frame.shape[1] // 2
        error = center - img_center

        if abs(error) < 80:
            self.send(ly=0.15)
        elif error > 0:
            self.send(ly=0.12)
        else:
            self.send(ly=-0.12)

    # =====================================================
    # 🎯 ALIGN
    # =====================================================

    def align(self):

        aruco = self.detect_aruco()

        if aruco is None:
            self.set_state("SEARCH")
            return

        cx, _ = aruco
        img_center = self.frame.shape[1] // 2
        error = cx - img_center

        if abs(error) > 30:
            self.send(az=-0.002 * error)
        else:
            self.set_state("APPROACH")

    # =====================================================
    # 🚀 APPROACH
    # =====================================================

    def approach(self):

        aruco = self.detect_aruco()

        if aruco is None:
            self.set_state("SEARCH")
            return

        _, area = aruco

        if area < 15000:
            self.send(lx=0.1)
        else:
            self.stop()
            rospy.loginfo("OBJETIVO ALCANZADO")
            self.set_state("DONE")

    # =====================================================
    # 🔁 LOOP
    # =====================================================

    def run(self):

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            if self.state == "TAKEOFF_UP":
                self.takeoff_up()

            elif self.state == "MOVE_LEFT":
                self.move_left()

            elif self.state == "FORWARD":
                self.forward()

            elif self.state == "TURN":
                self.turn()

            elif self.state == "SEARCH":
                self.search()

            elif self.state == "ALIGN":
                self.align()

            elif self.state == "APPROACH":
                self.approach()

            elif self.state == "DONE":
                self.stop()

            rate.sleep()


if __name__ == "__main__":
    mission = BebopMission()
    mission.run()