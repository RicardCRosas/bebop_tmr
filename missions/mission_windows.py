#!/usr/bin/env python3
# mission_windows.py

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Empty
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import os
import sys

# =====================================================
# FIX IMPORT PATH
# =====================================================
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, '..'))
sys.path.append(project_root)

from control.bebop_teleop_controller import BebopMovements
from perception.square_detector import BebopCameraProcessor

# =====================================================
# MISSION CLASS
# =====================================================
class MissionWindows:

    def __init__(self):
        rospy.init_node('mission_windows')

        # ========================
        # Publishers
        # ========================
        self.pub_cmd = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1)
        self.pub_takeoff = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1)
        self.pub_land = rospy.Publisher('/bebop/land', Empty, queue_size=1)
        self.pub_camera = rospy.Publisher('/bebop/camera_control', Twist, queue_size=1)
        self.status_pub = rospy.Publisher('/mission/status', String, queue_size=1)

        # ========================
        # Movement + Vision
        # ========================
        self.movements = BebopMovements(
            self.pub_cmd,
            self.pub_takeoff,
            self.pub_land,
            self.pub_camera
        )

        self.detector = BebopCameraProcessor()
        self.bridge = CvBridge()

        # ========================
        # Subscriber
        # ========================
        self.image_sub = rospy.Subscriber(
            "/bebop/image_raw",
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2**24
        )

        # ========================
        # State Variables
        # ========================
        self.latest_image_msg = None
        self.latest_data = None
        self.token = 0  # 0 = buscando, 1 = detectando/seguimiento, 2 = post-detección
        self.last_detected_time = None

        self.finished = False
        self.center_tolerance = 10
        self.vertical_tolerance = 15

        self.kx = 0.00025
        self.ky = 0.00025

        # 🔥 CONTROL AVANCE
        self.desired_size = 160000
        self.k_forward = 0.0001
        self.max_vx = 0.2
        self.min_vx = 0.05

        self.debug_image = None

        rospy.loginfo("Mission Windows initialized")

    # =====================================================
    # IMAGE CALLBACK
    # =====================================================
    def image_callback(self, msg):
        if self.finished:
            return
        self.latest_image_msg = msg

    # =====================================================
    # PROCESS IMAGE
    # =====================================================
    def process_latest_image(self):

        if self.latest_image_msg is None:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(self.latest_image_msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"CvBridge error: {e}")
            return

        frame = cv2.resize(frame, (640, 360))

        processed_image, data, _ = self.detector.process_image(frame)

        self.debug_image = processed_image
        self.latest_data = data

    # =====================================================
    # CONTROL LOGIC
    # =====================================================
    def control_logic(self):

        if self.latest_data is None:
            return

        data = self.latest_data
        detected = data.get("detected", False)

        twist = Twist()

        #=============
        #TIME
        #=============

        now = rospy.Time.now()

        if self.last_detected_time is not None:
            elapsed = (now - self.last_detected_time).to_sec()
        else:
            elapsed = None

        
        # ========================
        # SIN DETECCIÓN
        # ========================
        if self.token == 0 and not detected:
            rospy.loginfo("Searching LEFT...")
            #self.movements.left("automatic")
            rospy.sleep(0.5)
            return
        
        if detected:
            self.last_detected_time = now

            if self.token != 1:
                rospy.loginfo("Target detected → switching to tracking")
                self.token = 1

        if self.token == 1 and not detected:
            if elapsed is None:
                return
            if elapsed >= 1.5:
                rospy.loginfo(f"Lost target... waiting {elapsed:.2f}s")
            # Espera 3.5 segundos para ver si reaparece
            if elapsed < 5:
                return
            rospy.loginfo("No detection → executing final sequence")

            # avanzar y girar
            if elapsed > 5 and elapsed <= 7:
                twist.linear.x = 0.20
                self.pub_cmd.publish(twist)
                return
            self.movements.reset_twist()
            self.movements.turn_right("automatic")
            rospy.sleep(1.0)
            # aterrizar
            self.movements.landing("automatic")
            self.finished = True
            return

        # ========================
        # ERRORES
        # ========================
        cx, cy = data["cx"], data["cy"]
        center_x, center_y = data["center_x"], data["center_y"]

        error_x = cx - center_x
        error_y = cy - center_y

        aligned_x = abs(error_x) < self.center_tolerance
        aligned_y = abs(error_y) < self.vertical_tolerance

        # ========================
        # CONTROL LATERAL
        # ========================
        if not aligned_x:
            twist.linear.y = -error_x * self.kx
            self.pub_cmd.publish(twist)
            rospy.loginfo("Align x")
            return

        # ========================
        # CONTROL VERTICAL
        # ========================
        if not aligned_y:
            twist.linear.z = -error_y * self.ky
            self.pub_cmd.publish(twist)
            rospy.loginfo("Align y")
            return

        # ========================
        # CONTROL AVANCE (SIZE)
        # ========================
        size = data.get("size", None)

        if size is None:
            return

        error_size = self.desired_size - size

        vx = self.k_forward * error_size

        # zona muerta
        if abs(error_size) < 10000:
            vx = 0.0

        # saturación
        vx = max(min(vx, self.max_vx), -self.max_vx)

        # evitar quedarse detenido lejos
        if vx > 0:
            vx = max(vx, self.min_vx)

        twist.linear.x = vx
        self.pub_cmd.publish(twist)

        rospy.loginfo(f"Size: {size} | vx: {vx:.3f}")

    # =====================================================
    # MAIN LOOP
    # =====================================================
    def run(self):

        rate = rospy.Rate(20)

        while not rospy.is_shutdown():

            if not self.finished:
                self.process_latest_image()
                self.control_logic()

                if self.debug_image is not None:
                    cv2.imshow("Detection", self.debug_image)
                    cv2.waitKey(1)

            rate.sleep()


# =====================================================
# MAIN
# =====================================================
if __name__ == "__main__":
    mission = MissionWindows()
    mission.run()