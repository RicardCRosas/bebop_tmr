#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# mission_helipad.py - Versión con parámetros ROS

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Empty
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import sys
import os
import cv2
import numpy as np 
import time

# =====================================================
# FIX IMPORT PATH
# =====================================================

current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, '..'))
sys.path.append(project_root)

from control.bebop_teleop_controller import BebopMovements
from perception.helipad_detector import HelipadDetector

# =====================================================
# MISSION CLASS
# ====================================================
class MissionHelipad:

    def __init__(self):

        rospy.init_node('mission_helipad')
        
        # ========================
        # Parámetros ROS
        # ========================
        # Parámetros de la misión
        self.center_tolerance = rospy.get_param("~center_tolerance", 20)
        self.helipad_real_size_cm = rospy.get_param("~helipad_real_size_cm", 50)
        self.landing_distance_threshold_cm = rospy.get_param("~landing_distance_threshold_cm", 40)
        self.target_distance_cm = rospy.get_param("~target_distance_cm", 35)
        self.control_rate = rospy.get_param("~control_rate", 20)
        self.descend_duration = rospy.get_param("~descend_duration", 2.0)  # segundos
        self.show_debug = rospy.get_param("~show_debug", True)
        
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

        # Perception module optimizado
        self.detector = HelipadDetector()
        self.bridge = CvBridge()

        # ROS utilities
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
        self.last_control_time = time.time()

        self.finished = False
        self.alignment_counter = 0
        self.approach_counter = 0
        self.landing_phase = False
        self.no_detection_counter = 0
        
        # Control de frecuencia
        self.last_control_cycle = time.time()
        
        # Debug
        self.debug_image = None

        # Log de inicialización
        rospy.loginfo("=" * 50)
        rospy.loginfo("Mission Helipad initialized")
        rospy.loginfo(f"  Center tolerance: {self.center_tolerance}px")
        rospy.loginfo(f"  Helipad real size: {self.helipad_real_size_cm}cm")
        rospy.loginfo(f"  Target distance: {self.target_distance_cm}cm")
        rospy.loginfo(f"  Landing threshold: {self.landing_distance_threshold_cm}cm")
        rospy.loginfo(f"  Control rate: {self.control_rate}Hz")
        rospy.loginfo(f"  Descend duration: {self.descend_duration}s")
        rospy.loginfo("=" * 50)

    # =====================================================
    # IMAGE CALLBACK (LIGERO)
    # =====================================================
    def image_callback(self, msg):
        if self.finished:
            return
        self.latest_image_msg = msg

    # =====================================================
    # ESTIMATE DISTANCE FROM PIXEL SIZE
    # =====================================================
    def estimate_distance(self, pixel_width):
        """
        Estima la distancia al helipad basado en su tamaño en píxeles
        """
        if pixel_width <= 0:
            return float('inf')
        
        # Calibrated constant: at 100cm, pixel_width = (real_size * 100) / distance
        # So distance = (real_size * 100) / pixel_width
        calibration_constant = self.helipad_real_size_cm * 100
        estimated_distance = calibration_constant / pixel_width
        
        return estimated_distance

    # =====================================================
    # PROCESS IMAGE
    # =====================================================
    def process_latest_image(self):
        if self.latest_image_msg is None:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(self.latest_image_msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge error: {e}")
            return

        # Procesar con detector optimizado
        processed_image, data = self.detector.process_image(frame)
        
        # Estimar distancia si hay detección
        if data['detected']:
            pixel_width = data.get("width", 0)
            if pixel_width > 0:
                estimated_distance = self.estimate_distance(pixel_width)
                data['estimated_distance_cm'] = estimated_distance
                self.no_detection_counter = 0
            else:
                data['estimated_distance_cm'] = float('inf')
        else:
            self.no_detection_counter += 1

        self.debug_image = processed_image
        self.latest_data = data
        self.last_detection_time = rospy.Time.now()

    # =====================================================
    # CONTROL LOGIC
    # =====================================================
    def control_logic(self):
        # Control de frecuencia
        current_time = time.time()
        if (current_time - self.last_control_cycle) < (1.0 / self.control_rate):
            return
        self.last_control_cycle = current_time
        
        if self.latest_data is None:
            return

        # Timeout si no hay detección
        if self.no_detection_counter > 10:
            rospy.loginfo_throttle(1, "No helipad detected - searching...")
            self.movements.turn_left("automatic")
            return

        data = self.latest_data

        if not data["detected"]:
            rospy.loginfo_throttle(1, "Searching for helipad...")
            self.movements.turn_left("automatic")
            return

        # Extraer datos
        cx = data["cx"]
        cy = data["cy"]
        center_x = data["center_x"]
        center_y = data["center_y"]
        estimated_distance = data.get("estimated_distance_cm", float('inf'))
        
        # ========================
        # ALIGNMENT LOGIC
        # ========================
        
        # Centrado horizontal
        if abs(cx - center_x) > self.center_tolerance:
            if cx < center_x - self.center_tolerance:
                self.movements.left("automatic")
            elif cx > center_x + self.center_tolerance:
                self.movements.right("automatic")
            return
        
        # Centrado vertical
        if abs(cy - center_y) > self.center_tolerance:
            if cy < center_y - self.center_tolerance:
                self.movements.forward("automatic")
            elif cy > center_y + self.center_tolerance:
                self.movements.backward("automatic")
            return
        
        # ========================
        # APPROACH & LANDING LOGIC
        # ========================
        
        # Si estamos en fase de aterrizaje
        if self.landing_phase:
            rospy.loginfo_throttle(1, f"Landing phase - descending... ({self.alignment_counter}/{self.descend_duration*self.control_rate:.0f})")
            self.movements.down("automatic")
            
            # Descenso por tiempo
            self.alignment_counter += 1
            if self.alignment_counter > (self.descend_duration * self.control_rate):
                self.finish_mission()
            return
        
        # Fase de aproximación
        if estimated_distance <= self.landing_distance_threshold_cm:
            rospy.loginfo(f"Distance {estimated_distance:.0f}cm - Starting landing sequence!")
            self.landing_phase = True
            self.alignment_counter = 0
            return
        
        elif estimated_distance <= self.target_distance_cm + 20:
            # Aproximación lenta
            rospy.loginfo_throttle(1, f"Slow approach - distance: {estimated_distance:.0f}cm")
            self.movements.forward("automatic")
            self.approach_counter += 1
        else:
            # Aproximación normal
            rospy.loginfo_throttle(1, f"Approaching - distance: {estimated_distance:.0f}cm")
            self.movements.forward("automatic")
            self.approach_counter = 0

    # =====================================================
    # FINISH MISSION
    # =====================================================
    def finish_mission(self):
        rospy.loginfo("=" * 50)
        rospy.loginfo("Final landing sequence...")
        
        # Detener movimiento
        self.movements.stop("automatic")
        rospy.sleep(0.5)
        
        # Aterrizar
        self.movements.landing("automatic")
        
        self.status_pub.publish("done")
        self.finished = True
        
        # Mostrar estadísticas finales
        perf = self.detector.get_performance_summary()
        if perf.get('total_time'):
            rospy.loginfo(f"Performance summary:")
            rospy.loginfo(f"  Avg inference: {perf['inference_time']['avg']:.1f}ms")
            rospy.loginfo(f"  Avg preprocess: {perf['preprocess_time']['avg']:.1f}ms")
            rospy.loginfo(f"  Avg total latency: {perf['total_time']['avg']:.1f}ms")
        
        rospy.loginfo("Mission Helipad completed - Successfully landed!")
        rospy.loginfo("=" * 50)

    # =====================================================
    # MAIN LOOP
    # =====================================================
    def run(self):
        rate = rospy.Rate(30)

        # Inicializar cámara
        rospy.loginfo("Initializing camera to look down...")
        self.movements.camera_tilt(-90)
        rospy.sleep(2)
        
        # Despegar
        rospy.loginfo("Taking off...")
        self.movements.takeoff("automatic")
        rospy.sleep(3)

        # Bucle principal
        while not rospy.is_shutdown() and not self.finished:
            # Procesar imagen
            self.process_latest_image()
            
            # Ejecutar control
            self.control_logic()
            
            # Mostrar debug
            if self.show_debug and self.debug_image is not None:
                if self.latest_data and self.latest_data.get('detected', False):
                    distance = self.latest_data.get('estimated_distance_cm', 0)
                    phase = 'Landing' if self.landing_phase else 'Approach'
                    cv2.putText(self.debug_image, 
                               f"Dist: {distance:.0f}cm | {phase}",
                               (10, 80),
                               cv2.FONT_HERSHEY_SIMPLEX,
                               0.6,
                               (0, 255, 0),
                               2)
                
                cv2.imshow("Helipad Detection", self.debug_image)
                cv2.waitKey(1)
            
            rate.sleep()

        cv2.destroyAllWindows()

# =====================================================
# MAIN
# =====================================================
if __name__ == "__main__":
    try:
        mission = MissionHelipad()
        mission.run()
    except rospy.ROSInterruptException:
        pass
