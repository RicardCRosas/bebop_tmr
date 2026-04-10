#!/usr/bin/env python3

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
from perception.window_detector import BebopCameraProcessor

# =====================================================
# MISSION CLASS
# =====================================================

class MissionOrangeWindow:

    def __init__(self):

        rospy.init_node('mission_orange_window')

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
        self.last_detection_time = rospy.Time.now()

        self.finished = False
        self.center_tolerance = 40
        self.vertical_tolerance = 25

        self.forward_counter = 0
        self.debug_image = None

        rospy.loginfo("Mission Orange Window initialized")

    # =====================================================
    # IMAGE CALLBACK (SIN PROCESAMIENTO)
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

        # Reducir resolución
        frame = cv2.resize(frame, (640, 360))

        # Medir inferencia
        start_time = rospy.Time.now()

        processed_image, data, _ = self.detector.process_image(frame)

        end_time = rospy.Time.now()
        inference_time = (end_time - start_time).to_sec()

        # Dibujar info
        cv2.putText(processed_image,
                    f"Inference: {inference_time*1000:.1f} ms",
                    (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2)

        self.debug_image = processed_image
        self.latest_data = data
        self.last_detection_time = rospy.Time.now()

    # =====================================================
    # CONTROL LOGIC
    # =====================================================

    def control_logic(self):

        if self.latest_data is None:
            return

        # Timeout detección
        if (rospy.Time.now() - self.last_detection_time).to_sec() > 0.5:
            rospy.loginfo("Detection timeout → searching")
            self.movements.turn_left("automatic")
            self.forward_counter = 0
            return

        data = self.latest_data

        if not data["detected"]:
            rospy.loginfo("Searching window...")
            self.movements.turn_left("automatic")
            self.forward_counter = 0
            return

        cx = data["cx"]
        cy = data["cy"]
        center_x = data["center_x"]
        center_y = data["center_y"]

        error_x = cx - center_x
        error_y = cy - center_y

        aligned_x = abs(error_x) < self.center_tolerance
        aligned_y = abs(error_y) < self.vertical_tolerance

        # ========================
        # CONTROL SECUENCIAL
        # ========================

        # 1️⃣ Alinear en X
        if not aligned_x:
            if error_x < 0:
                rospy.loginfo("Adjusting LEFT")
                self.movements.left("automatic")
            else:
                rospy.loginfo("Adjusting RIGHT")
                self.movements.right("automatic")

            self.forward_counter = 0
            return

        # 2️⃣ Alinear en Y
        if not aligned_y:
            if error_y < 0:
                rospy.loginfo("Adjusting UP")
                self.movements.up("automatic")
            else:
                rospy.loginfo("Adjusting DOWN")
                self.movements.down("automatic")

            self.forward_counter = 0
            return

        # 3️⃣ Avanzar
        rospy.loginfo("Centered → FORWARD")
        self.movements.forward("automatic")
        self.forward_counter += 1

        # 4️⃣ Condición de éxito
        if self.forward_counter > 20:
            rospy.loginfo("Window passed!")
            self.finish_mission()

    # =====================================================
    # FINISH MISSION
    # =====================================================

    def finish_mission(self):
        self.movements.stop("automatic")
        self.status_pub.publish("done")
        self.finished = True
        rospy.loginfo("Mission completed")

    # =====================================================
    # MAIN LOOP
    # =====================================================

    def run(self):

        rate = rospy.Rate(15)

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
    mission = MissionOrangeWindow()
    mission.run()