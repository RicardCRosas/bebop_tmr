#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import math
import rospy
import cv2

from sensor_msgs.msg import Image
from std_msgs.msg import String, Empty
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, '..'))
if project_root not in sys.path:
    sys.path.append(project_root)

from control.bebop_teleop_controller import BebopMovements
from perception.aruco_whiteboard_detector import ArucoWhiteboardDetector


class MissionWhiteboardAruco:
    """
    Misión autónoma para:
      1) buscar el ArUco del pizarrón
      2) alinearse
      3) aproximarse
      4) moverse al punto de inicio del trazo
      5) tocar suavemente
      6) dibujar una línea horizontal
      7) retirarse
      8) reportar done

    Importante:
    Esta primera versión usa control visual heurístico basado en:
      - error de centro en imagen
      - tamaño del marcador en píxeles
      - tiempos de movimiento

    Es una base funcional para pruebas rápidas.
    En la pista real habrá que ajustar tolerancias, velocidades y tiempos.
    """

    def __init__(self):
        rospy.init_node('mission_whiteboard_aruco')

        self.pub_cmd = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1)
        self.pub_takeoff = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1)
        self.pub_land = rospy.Publisher('/bebop/land', Empty, queue_size=1)
        self.pub_camera = rospy.Publisher('/bebop/camera_control', Twist, queue_size=1)
        self.status_pub = rospy.Publisher('/mission/status', String, queue_size=1)

        self.movements = BebopMovements(
            self.pub_cmd,
            self.pub_takeoff,
            self.pub_land,
            self.pub_camera
        )

        self.detector = ArucoWhiteboardDetector()
        self.bridge = CvBridge()

        self.latest_image_msg = None
        self.latest_data = None
        self.debug_image = None
        self.last_detection_time = None

        self.finished = False
        self.state = "SEARCH_ARUCO"
        self.state_start_time = rospy.Time.now()

        self.image_sub = rospy.Subscriber(
            "/bebop/image_raw",
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2**24
        )

        # -------------------------
        # Parámetros de prueba
        # -------------------------
        self.test_mode = rospy.get_param("~test_mode", "full")
        self.show_debug = rospy.get_param("~show_debug", False)
        self.image_width = rospy.get_param("~image_width", 640)
        self.image_height = rospy.get_param("~image_height", 360)

        self.center_tolerance_x = rospy.get_param("~center_tolerance_x", 35)
        self.center_tolerance_y = rospy.get_param("~center_tolerance_y", 30)

        self.draw_target_tolerance_x = rospy.get_param("~draw_target_tolerance_x", 35)
        self.draw_target_tolerance_y = rospy.get_param("~draw_target_tolerance_y", 35)

        self.approach_area_threshold = rospy.get_param("~approach_area_threshold", 17000)
        self.touch_area_threshold = rospy.get_param("~touch_area_threshold", 25000)

        self.search_yaw_speed = rospy.get_param("~search_yaw_speed", 0.22)
        self.align_lateral_speed = rospy.get_param("~align_lateral_speed", 0.16)
        self.align_vertical_speed = rospy.get_param("~align_vertical_speed", 0.14)
        self.approach_speed = rospy.get_param("~approach_speed", 0.12)

        self.touch_forward_speed = rospy.get_param("~touch_forward_speed", 0.09)
        self.touch_forward_time = rospy.get_param("~touch_forward_time", 1.2)

        self.draw_forward_bias = rospy.get_param("~draw_forward_bias", 0.03)
        self.draw_lateral_speed = rospy.get_param("~draw_lateral_speed", -0.16)
        self.draw_time = rospy.get_param("~draw_time", 1.8)

        self.backoff_speed = rospy.get_param("~backoff_speed", -0.12)
        self.backoff_time = rospy.get_param("~backoff_time", 1.2)

        self.detection_timeout = rospy.get_param("~detection_timeout", 0.6)
        self.max_mission_time = rospy.get_param("~max_mission_time", 45.0)

        rospy.loginfo("MissionWhiteboardAruco initialized")
        rospy.loginfo(f"test_mode = {self.test_mode}")

    # =====================================================
    # Helpers
    # =====================================================

    def image_callback(self, msg):
        if not self.finished:
            self.latest_image_msg = msg

    def process_latest_image(self):
        if self.latest_image_msg is None:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(self.latest_image_msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"CvBridge error: {e}")
            return

        frame = cv2.resize(frame, (self.image_width, self.image_height))
        processed_image, data = self.detector.process_image(frame)

        self.debug_image = processed_image
        self.latest_data = data

        if data["detected"]:
            self.last_detection_time = rospy.Time.now()

    def set_state(self, new_state):
        if self.state != new_state:
            rospy.loginfo(f"MISSION STATE -> {new_state}")
            self.state = new_state
            self.state_start_time = rospy.Time.now()
            self.movements.reset_twist()

    def elapsed_in_state(self):
        return (rospy.Time.now() - self.state_start_time).to_sec()

    def mission_elapsed(self):
        return (rospy.Time.now() - self.start_time).to_sec()

    def detection_recent(self):
        if self.last_detection_time is None:
            return False
        return (rospy.Time.now() - self.last_detection_time).to_sec() < self.detection_timeout

    def publish_direct_twist(self, lx=0.0, ly=0.0, lz=0.0, az=0.0):
        msg = Twist()
        msg.linear.x = lx
        msg.linear.y = ly
        msg.linear.z = lz
        msg.angular.z = az
        self.pub_cmd.publish(msg)

    def stop(self):
        self.movements.reset_twist()

    def finish_mission(self):
        self.stop()
        self.status_pub.publish("done")
        self.finished = True
        rospy.loginfo("Mission Whiteboard ArUco completed")

    def fail_mission(self, reason="unknown"):
        rospy.logwarn(f"Mission Whiteboard ArUco failed: {reason}")
        self.stop()
        self.status_pub.publish("failed")
        self.finished = True

    def should_finish_after(self, mode_name):
        """
        Permite probar por etapas.
        """
        order = {
            "search_align": 1,
            "approach": 2,
            "draw_start": 3,
            "touch": 4,
            "draw": 5,
            "full": 6,
        }

        current_stage_map = {
            "ALIGN_OK": 1,
            "APPROACH_OK": 2,
            "DRAW_START_OK": 3,
            "TOUCH_OK": 4,
            "DRAW_OK": 5,
            "FULL_OK": 6,
        }

        wanted = order.get(self.test_mode, 6)
        current = current_stage_map.get(mode_name, 999)
        return current >= wanted

    # =====================================================
    # Etapas de la misión
    # =====================================================

    def handle_search_aruco(self):
        data = self.latest_data

        if data is None:
            self.publish_direct_twist(az=self.search_yaw_speed)
            return

        if not data["detected"]:
            self.publish_direct_twist(az=self.search_yaw_speed)
            return

        self.set_state("ALIGN_TO_MARKER")

    def handle_align_to_marker(self):
        data = self.latest_data

        if data is None or not self.detection_recent() or not data["detected"]:
            self.set_state("SEARCH_ARUCO")
            return

        err_x = data["error_x"]
        err_y = data["error_y"]

        aligned_x = abs(err_x) <= self.center_tolerance_x
        aligned_y = abs(err_y) <= self.center_tolerance_y

        if not aligned_x:
            if err_x < 0:
                # marcador a la izquierda en imagen -> moverse izquierda
                self.publish_direct_twist(ly=self.align_lateral_speed)
            else:
                self.publish_direct_twist(ly=-self.align_lateral_speed)
            return

        if not aligned_y:
            if err_y < 0:
                # marcador alto en imagen -> subir
                self.publish_direct_twist(lz=self.align_vertical_speed)
            else:
                # marcador bajo en imagen -> bajar
                self.publish_direct_twist(lz=-self.align_vertical_speed)
            return

        self.stop()
        if self.should_finish_after("ALIGN_OK"):
            self.finish_mission()
            return

        self.set_state("APPROACH_BOARD")

    def handle_approach_board(self):
        data = self.latest_data

        if data is None or not self.detection_recent() or not data["detected"]:
            self.set_state("SEARCH_ARUCO")
            return

        err_x = data["error_x"]
        err_y = data["error_y"]
        area = data["area"]

        # sigue corrigiendo mientras se acerca
        if abs(err_x) > self.center_tolerance_x:
            if err_x < 0:
                self.publish_direct_twist(ly=self.align_lateral_speed * 0.8)
            else:
                self.publish_direct_twist(ly=-self.align_lateral_speed * 0.8)
            return

        if abs(err_y) > self.center_tolerance_y:
            if err_y < 0:
                self.publish_direct_twist(lz=self.align_vertical_speed * 0.8)
            else:
                self.publish_direct_twist(lz=-self.align_vertical_speed * 0.8)
            return

        if area < self.approach_area_threshold:
            self.publish_direct_twist(lx=self.approach_speed)
            return

        self.stop()
        if self.should_finish_after("APPROACH_OK"):
            self.finish_mission()
            return

        self.set_state("MOVE_TO_DRAW_START")

    def handle_move_to_draw_start(self):
        data = self.latest_data

        if data is None or not self.detection_recent() or not data["detected"]:
            self.set_state("SEARCH_ARUCO")
            return

        tx = data["target_draw_x"]
        ty = data["target_draw_y"]
        if tx is None or ty is None:
            self.set_state("SEARCH_ARUCO")
            return

        img_cx = data["center_x"]
        img_cy = data["center_y"]

        err_x = tx - img_cx
        err_y = ty - img_cy
        area = data["area"]

        aligned_x = abs(err_x) <= self.draw_target_tolerance_x
        aligned_y = abs(err_y) <= self.draw_target_tolerance_y

        if not aligned_x:
            if err_x < 0:
                self.publish_direct_twist(ly=self.align_lateral_speed * 0.85)
            else:
                self.publish_direct_twist(ly=-self.align_lateral_speed * 0.85)
            return

        if not aligned_y:
            if err_y < 0:
                self.publish_direct_twist(lz=self.align_vertical_speed * 0.85)
            else:
                self.publish_direct_twist(lz=-self.align_vertical_speed * 0.85)
            return

        # Acerca un poco más antes del contacto suave
        if area < self.touch_area_threshold:
            self.publish_direct_twist(lx=self.approach_speed * 0.8)
            return

        self.stop()
        if self.should_finish_after("DRAW_START_OK"):
            self.finish_mission()
            return

        self.set_state("TOUCH_BOARD")

    def handle_touch_board(self):
        elapsed = self.elapsed_in_state()

        if elapsed < self.touch_forward_time:
            self.publish_direct_twist(lx=self.touch_forward_speed)
            return

        self.stop()
        if self.should_finish_after("TOUCH_OK"):
            self.finish_mission()
            return

        self.set_state("DRAW_LINE")

    def handle_draw_line(self):
        elapsed = self.elapsed_in_state()

        if elapsed < self.draw_time:
            # pequeño empuje al frente + movimiento lateral para trazar
            self.publish_direct_twist(
                lx=self.draw_forward_bias,
                ly=self.draw_lateral_speed
            )
            return

        self.stop()
        if self.should_finish_after("DRAW_OK"):
            self.finish_mission()
            return

        self.set_state("BACK_OFF")

    def handle_back_off(self):
        elapsed = self.elapsed_in_state()

        if elapsed < self.backoff_time:
            self.publish_direct_twist(lx=self.backoff_speed)
            return

        self.stop()
        self.set_state("DONE")

    def handle_done(self):
        self.finish_mission()

    # =====================================================
    # Lógica principal
    # =====================================================

    def control_logic(self):
        if self.finished:
            return

        if self.mission_elapsed() > self.max_mission_time:
            self.fail_mission("timeout")
            return

        if self.state == "SEARCH_ARUCO":
            self.handle_search_aruco()

        elif self.state == "ALIGN_TO_MARKER":
            self.handle_align_to_marker()

        elif self.state == "APPROACH_BOARD":
            self.handle_approach_board()

        elif self.state == "MOVE_TO_DRAW_START":
            self.handle_move_to_draw_start()

        elif self.state == "TOUCH_BOARD":
            self.handle_touch_board()

        elif self.state == "DRAW_LINE":
            self.handle_draw_line()

        elif self.state == "BACK_OFF":
            self.handle_back_off()

        elif self.state == "DONE":
            self.handle_done()

        else:
            self.fail_mission(f"unknown_state_{self.state}")

    def run(self):
        self.start_time = rospy.Time.now()
        rate = rospy.Rate(15)

        while not rospy.is_shutdown() and not self.finished:
            self.process_latest_image()
            self.control_logic()

            if self.show_debug and self.debug_image is not None:
                cv2.imshow("whiteboard_aruco_debug", self.debug_image)
                cv2.waitKey(1)

            rate.sleep()

        self.stop()
        if self.show_debug:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    mission = MissionWhiteboardAruco()
    mission.run()