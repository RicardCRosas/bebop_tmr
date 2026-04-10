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
from perception.helipad_detector import BebopCameraProcessor

# =====================================================
# MISSION CLASS
# =====================================================
class MissionHelipad:

    def __init__(self):
        rospy.init_node('mission_helipad')

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
        self.center_tolerance = 30
        self.vertical_tolerance = 30
        self.camera_angle = 0
        self.search_complete = False
        self.aligned_time = None
        self.required_stable_time = 1.5  # segundos (ajustable)

        self.debug_image = None

        # Control de cámara (solo una vez)
        self.camera_initialized = False

        # Umbral de aterrizaje por tamaño
        self.landing_area_threshold = 30000  # 🔥 AJUSTABLE

        rospy.loginfo("Mission Helipad initialized")

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

        start_time = rospy.Time.now()
        processed_image, data = self.detector.process_image(frame)
        end_time = rospy.Time.now()

        inference_time = (end_time - start_time).to_sec()

        cv2.putText(processed_image,
                    f"Inference: {inference_time*1000:.1f} ms",
                    (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2)

        self.debug_image = processed_image
        self.latest_data = data

        if data["detected"]:
            self.last_detection_time = rospy.Time.now()
 
    # =====================================================
    # CONTROL LOGIC
    # =====================================================
    def control_logic(self):

        # Inicializar cámara una sola vez
        if not self.camera_initialized:
            self.camera_angle = 0
            self.movements.camera_tilt(self.camera_angle)
            rospy.sleep(1.5)
            self.camera_initialized = True
            return

        if self.latest_data is None:
            return

        data = self.latest_data

        # ========================
        # 🔍 FASE 1: BÚSQUEDA
        # ========================
        if not data.get("detected", False) and not self.search_complete:

            rospy.loginfo(f"Searching... Camera angle: {self.camera_angle}")

            # Avanza lentamente mientras busca
            self.movements.forward("automatic")

            # Cambiar ángulo gradualmente
            if self.camera_angle > -90:
                self.camera_angle -= 30
                self.movements.camera_tilt(self.camera_angle)
                rospy.sleep(1.0)
            else:
                rospy.loginfo("Reached downward view → assuming above helipad")
                self.search_complete = True

            return

        # ========================
        # 🧠 FASE 2: CONTROL (YA ABAJO)
        # ========================

        cx = data["cx"]
        cy = data["cy"]
        center_x = data["center_x"]
        center_y = data["center_y"]

        error_x = cx - center_x
        error_y = cy - center_y

        if self.camera_angle != 0:
            extra_tol = 90 / abs(self.camera_angle)
        else:
            extra_tol = 0

        aligned_x = abs(error_x) < (self.center_tolerance + extra_tol)
        aligned_y = abs(error_y) < self.vertical_tolerance

        # ========================
        # CONTROL EN X
        # ========================
        if not aligned_x:
            if error_x < 0:
                rospy.loginfo("Adjusting LEFT")
                self.movements.left("automatic")
            else:
                rospy.loginfo("Adjusting RIGHT")
                self.movements.right("automatic")
            return

        # ========================
        # CONTROL EN Y
        # ========================
        if not aligned_y:
            if error_y < 0:
                rospy.loginfo("Adjusting FRONT")
                self.movements.forward("automatic")
            else:
                rospy.loginfo("Adjusting BACK")
                self.movements.backwards("automatic")
            return

        # ========================
        # DESCENSO + ATERRIZAJE
        # ========================

        if aligned_x and aligned_y and not self.search_complete:
            self.camera_angle -= 30
            self.movements.camera_tilt(self.camera_angle)
            rospy.sleep(1.0)


        area = data.get("area", 0)

        rospy.loginfo(f"Centered | Area: {area}")

        current_time = rospy.Time.now().to_sec()

        # ========================
        # VERIFICAR ESTABILIDAD
        # ========================
        if aligned_x and aligned_y:
            if self.aligned_time is None:
                self.aligned_time = current_time
            elif (current_time - self.aligned_time) >= self.required_stable_time:
                # 🔥 SOLO aquí baja
                if area < self.landing_area_threshold:
                    rospy.loginfo("Stable → Descending...")
                    self.movements.down("automatic")
                else:
                    rospy.loginfo("Landing condition met")
                    self.finish_mission()
        else:
            # Se perdió alineación → reiniciar contador
            self.aligned_time = None

    # =====================================================
    # FINISH MISSION
    # =====================================================
    def finish_mission(self):
        rospy.sleep(10)
        self.movements.f_landing("automatic")
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
    mission = MissionHelipad()
    mission.run()