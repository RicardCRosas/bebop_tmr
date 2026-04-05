#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import math
import rospy
import cv2
import threading
import select
import termios
import tty
import numpy as np

from sensor_msgs.msg import Image
from std_msgs.msg import String, Empty
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from tf.transformations import euler_from_quaternion

current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, '..'))
if project_root not in sys.path:
    sys.path.append(project_root)

from control.bebop_teleop_controller import BebopMovements
from perception.aruco_whiteboard_detector import ArucoWhiteboardDetector


class MissionWhiteboardAruco:
    """
    Versión estable para pruebas de:
      - search_align
      - approach

    Objetivo:
      - no exigir alineación perfecta
      - no perder el ArUco tan fácil
      - no acercarse demasiado
      - corregir sin avanzar cuando esté desalineado
    """

    def __init__(self):
        rospy.init_node("mission_whiteboard_aruco")

        self.pub_cmd = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1)
        self.pub_takeoff = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1)
        self.pub_land = rospy.Publisher('/bebop/land', Empty, queue_size=1)
        self.pub_reset = rospy.Publisher('/bebop/reset', Empty, queue_size=1)
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

        self.image_sub = rospy.Subscriber(
            "/bebop/image_raw",
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2**24
        )

        self.odom_sub = rospy.Subscriber(
            "/bebop/odom",
            Odometry,
            self.odom_callback,
            queue_size=1
        )

        # Imagen / detección
        self.latest_image_msg = None
        self.latest_data = None
        self.debug_image = None
        self.last_detection_time = None

        # Odom
        self.current_x = None
        self.current_y = None
        self.current_z = None
        self.current_yaw = None

        # Refs
        self.approach_start_x = None
        self.approach_start_y = None
        self.approach_start_z = None

        # Estado
        self.finished = False
        self.state = "SET_CAMERA"
        self.state_start_time = rospy.Time.now()
        self.start_time = rospy.Time.now()

        # Detección / recuperación
        self.detect_stable_count = 0
        self.lost_count = 0
        self.recover_return_state = None

        # Emergencia
        self.abort_land_requested = False
        self.emergency_reset_requested = False
        self.term_settings = None
        self.keyboard_thread = None

        # -------------------------
        # Params
        # -------------------------
        self.test_mode = rospy.get_param("~test_mode", "approach")
        self.show_debug = rospy.get_param("~show_debug", True)
        self.image_width = rospy.get_param("~image_width", 640)
        self.image_height = rospy.get_param("~image_height", 360)

        self.camera_tilt_start = rospy.get_param("~camera_tilt_start", 18.0)
        self.camera_pan_start = rospy.get_param("~camera_pan_start", 0.0)
        self.camera_settle_time = rospy.get_param("~camera_settle_time", 1.5)

        # Tolerancias MÁS abiertas
        self.center_tolerance_x = rospy.get_param("~center_tolerance_x", 40)
        self.center_tolerance_y = rospy.get_param("~center_tolerance_y", 34)
        self.visual_yaw_tolerance = rospy.get_param("~visual_yaw_tolerance", 0.11)

        # Tolerancias aún más abiertas para permitir avanzar
        self.approach_tolerance_x = rospy.get_param("~approach_tolerance_x", 55)
        self.approach_tolerance_y = rospy.get_param("~approach_tolerance_y", 45)
        self.approach_visual_yaw_tolerance = rospy.get_param("~approach_visual_yaw_tolerance", 0.16)

        # Histeresis fuerte
        self.min_stable_detections = rospy.get_param("~min_stable_detections", 10)
        self.max_lost_before_search = rospy.get_param("~max_lost_before_search", 22)
        self.recover_hold_time = rospy.get_param("~recover_hold_time", 1.2)
        self.align_hold_time = rospy.get_param("~align_hold_time", 0.8)

        # Velocidades MÁS bajas
        self.search_yaw_speed = rospy.get_param("~search_yaw_speed", 0.03)
        self.align_lateral_speed = rospy.get_param("~align_lateral_speed", 0.02)
        self.align_vertical_speed = rospy.get_param("~align_vertical_speed", 0.02)

        self.approach_speed = rospy.get_param("~approach_speed", 0.004)
        self.approach_yaw_gain = rospy.get_param("~approach_yaw_gain", 0.28)
        self.approach_yaw_max = rospy.get_param("~approach_yaw_max", 0.025)

        # Más distancia segura = detenerse antes
        self.safe_approach_distance = rospy.get_param("~safe_approach_distance", 0.07)
        self.safe_approach_timeout = rospy.get_param("~safe_approach_timeout", 3.5)

        # Freno visual más conservador
        self.max_safe_area = rospy.get_param("~max_safe_area", 4200)

        self.max_mission_time = rospy.get_param("~max_mission_time", 35.0)

        rospy.loginfo("MissionWhiteboardAruco BETTER STABLE version initialized")
        rospy.loginfo(f"test_mode = {self.test_mode}")

    # =====================================================
    # Keyboard emergency
    # =====================================================

    def keyboard_listener(self):
        while not rospy.is_shutdown() and not self.finished:
            if select.select([sys.stdin], [], [], 0.05)[0]:
                key = sys.stdin.read(1)
                if key.lower() == 'q':
                    rospy.logwarn("Tecla 'q' detectada -> aterrizaje inmediato")
                    self.abort_land_requested = True
                    break
                elif key.lower() == 'e':
                    rospy.logerr("Tecla 'e' detectada -> EMERGENCY RESET")
                    self.emergency_reset_requested = True
                    break

    def start_keyboard_listener(self):
        if not sys.stdin.isatty():
            rospy.logwarn("stdin no es TTY; emergencia por teclado no disponible.")
            return

        try:
            self.term_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
            self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
            self.keyboard_thread.daemon = True
            self.keyboard_thread.start()
            rospy.loginfo("Emergencia activa: 'q' = land, 'e' = reset")
        except Exception as e:
            rospy.logwarn(f"No se pudo activar listener de teclado: {e}")

    def restore_terminal(self):
        if self.term_settings is not None and sys.stdin.isatty():
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.term_settings)
            except Exception:
                pass

    def handle_emergency_requests(self):
        if self.emergency_reset_requested:
            self.stop()
            rospy.sleep(0.1)
            self.pub_reset.publish(Empty())
            self.status_pub.publish("failed")
            self.finished = True
            return True

        if self.abort_land_requested:
            self.stop()
            rospy.sleep(0.1)
            self.pub_land.publish(Empty())
            self.status_pub.publish("failed")
            self.finished = True
            return True

        return False

    # =====================================================
    # Callbacks
    # =====================================================

    def image_callback(self, msg):
        if not self.finished:
            self.latest_image_msg = msg

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_z = msg.pose.pose.position.z

        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_yaw = yaw

    # =====================================================
    # Helpers
    # =====================================================

    def has_odom(self):
        return (
            self.current_x is not None and
            self.current_y is not None and
            self.current_z is not None and
            self.current_yaw is not None
        )

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
            self.detect_stable_count += 1
            self.lost_count = 0
        else:
            self.detect_stable_count = 0
            self.lost_count += 1

    def set_camera_pose(self, tilt_deg, pan_deg):
        cam = Twist()
        cam.angular.y = tilt_deg
        cam.angular.z = pan_deg
        self.pub_camera.publish(cam)

    def set_state(self, new_state, recover_return_state=None):
        if self.state != new_state:
            rospy.loginfo(f"MISSION STATE -> {new_state}")
            self.state = new_state
            self.state_start_time = rospy.Time.now()
            self.stop()

            if new_state == "APPROACH_BOARD" and self.has_odom():
                self.approach_start_x = self.current_x
                self.approach_start_y = self.current_y
                self.approach_start_z = self.current_z

            if new_state == "RECOVER_MARKER":
                self.recover_return_state = recover_return_state

    def elapsed_in_state(self):
        return (rospy.Time.now() - self.state_start_time).to_sec()

    def mission_elapsed(self):
        return (rospy.Time.now() - self.start_time).to_sec()

    def publish_direct_twist(self, lx=0.0, ly=0.0, lz=0.0, az=0.0):
        msg = Twist()
        msg.linear.x = lx
        msg.linear.y = ly
        msg.linear.z = lz
        msg.angular.z = az
        self.pub_cmd.publish(msg)

    def stop(self):
        self.movements.reset_twist()

    def odom_distance_since_approach_start(self):
        if not self.has_odom():
            return None
        if self.approach_start_x is None:
            return None

        dx = self.current_x - self.approach_start_x
        dy = self.current_y - self.approach_start_y
        dz = self.current_z - self.approach_start_z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def compute_visual_yaw_error(self, data):
        pts = data.get("corners", None)
        if pts is None or len(pts) != 4:
            return 0.0

        pts = np.array(pts, dtype=np.float32)
        tl, tr, br, bl = pts

        top_angle = math.atan2((tr[1] - tl[1]), (tr[0] - tl[0]))
        left_h = np.linalg.norm(bl - tl)
        right_h = np.linalg.norm(br - tr)
        avg_h = max((left_h + right_h) / 2.0, 1e-6)
        side_asym = (left_h - right_h) / avg_h

        return (0.8 * side_asym) + (0.7 * top_angle)

    def should_finish_after(self, mode_name):
        order = {
            "search_align": 1,
            "approach": 2,
        }

        current_stage_map = {
            "ALIGN_OK": 1,
            "APPROACH_OK": 2,
        }

        wanted = order.get(self.test_mode, 2)
        current = current_stage_map.get(mode_name, 999)
        return current >= wanted

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

    # =====================================================
    # States
    # =====================================================

    def handle_set_camera(self):
        self.set_camera_pose(self.camera_tilt_start, self.camera_pan_start)
        if self.elapsed_in_state() < self.camera_settle_time:
            return
        self.set_state("SEARCH_ARUCO")

    def handle_search_aruco(self):
        data = self.latest_data

        if data is None or not data["detected"]:
            self.publish_direct_twist(az=self.search_yaw_speed)
            return

        if self.detect_stable_count < self.min_stable_detections:
            self.stop()
            return

        self.set_state("ALIGN_TO_MARKER")

    def handle_align_to_marker(self):
        data = self.latest_data

        if data is None or not data["detected"]:
            if self.lost_count > self.max_lost_before_search:
                self.set_state("RECOVER_MARKER", recover_return_state="ALIGN_TO_MARKER")
            return

        err_x = data["error_x"]
        err_y = data["error_y"]
        yaw_error = self.compute_visual_yaw_error(data)

        aligned_x = abs(err_x) <= self.center_tolerance_x
        aligned_y = abs(err_y) <= self.center_tolerance_y
        aligned_yaw = abs(yaw_error) <= self.visual_yaw_tolerance

        if not aligned_yaw:
            wz = max(min(self.approach_yaw_gain * yaw_error, self.approach_yaw_max), -self.approach_yaw_max)
            self.publish_direct_twist(az=wz)
            return

        if not aligned_x:
            if err_x < 0:
                self.publish_direct_twist(ly=self.align_lateral_speed)
            else:
                self.publish_direct_twist(ly=-self.align_lateral_speed)
            return

        if not aligned_y:
            if err_y < 0:
                self.publish_direct_twist(lz=self.align_vertical_speed)
            else:
                self.publish_direct_twist(lz=-self.align_vertical_speed)
            return

        # NO busca perfección instantánea; mantiene hover un poco
        self.stop()

        if self.elapsed_in_state() < self.align_hold_time:
            return

        if self.should_finish_after("ALIGN_OK"):
            self.finish_mission()
            return

        self.set_state("APPROACH_BOARD")

    def handle_approach_board(self):
        data = self.latest_data

        if data is None or not data["detected"]:
            self.stop()
            if self.lost_count > self.max_lost_before_search:
                self.set_state("RECOVER_MARKER", recover_return_state="APPROACH_BOARD")
            return

        area = data["area"]
        if area >= self.max_safe_area:
            rospy.logwarn("Área demasiado alta. Frenando por seguridad.")
            self.stop()
            self.finish_mission()
            return

        err_x = data["error_x"]
        err_y = data["error_y"]
        yaw_error = self.compute_visual_yaw_error(data)

        ly = 0.0
        lz = 0.0
        wz = 0.0

        if abs(err_x) > self.approach_tolerance_x:
            ly = self.align_lateral_speed if err_x < 0 else -self.align_lateral_speed

        if abs(err_y) > self.approach_tolerance_y:
            lz = self.align_vertical_speed if err_y < 0 else -self.align_vertical_speed

        if abs(yaw_error) > self.approach_visual_yaw_tolerance:
            wz = max(min(self.approach_yaw_gain * yaw_error, self.approach_yaw_max), -self.approach_yaw_max)

        misaligned = (
            abs(err_x) > self.approach_tolerance_x or
            abs(err_y) > self.approach_tolerance_y or
            abs(yaw_error) > self.approach_visual_yaw_tolerance
        )

        progress = self.odom_distance_since_approach_start()

        if progress is not None and progress >= self.safe_approach_distance:
            rospy.loginfo(f"Distancia segura alcanzada por odometría: {progress:.3f} m")
            self.stop()
            self.finish_mission()
            return

        if self.elapsed_in_state() > self.safe_approach_timeout:
            rospy.logwarn("Timeout en APPROACH_BOARD. Deteniendo por seguridad.")
            self.stop()
            self.finish_mission()
            return

        # si se desalineó, corrige sin avanzar
        if misaligned:
            self.publish_direct_twist(
                lx=0.0,
                ly=ly,
                lz=lz,
                az=wz
            )
            return

        # solo avanza cuando ya está suficientemente bien colocado
        self.publish_direct_twist(
            lx=self.approach_speed,
            ly=0.0,
            lz=0.0,
            az=0.0
        )

    def handle_recover_marker(self):
        """
        Hover y espera un momento a que el ArUco reaparezca.
        No se va directo a SEARCH.
        """
        self.stop()

        data = self.latest_data
        if data is not None and data["detected"]:
            if self.detect_stable_count >= self.min_stable_detections:
                next_state = self.recover_return_state if self.recover_return_state is not None else "ALIGN_TO_MARKER"
                self.set_state(next_state)
                return

        if self.elapsed_in_state() > self.recover_hold_time:
            self.set_state("SEARCH_ARUCO")

    def control_logic(self):
        if self.finished:
            return

        if self.mission_elapsed() > self.max_mission_time:
            self.fail_mission("timeout")
            return

        if self.state == "SET_CAMERA":
            self.handle_set_camera()
        elif self.state == "SEARCH_ARUCO":
            self.handle_search_aruco()
        elif self.state == "ALIGN_TO_MARKER":
            self.handle_align_to_marker()
        elif self.state == "APPROACH_BOARD":
            self.handle_approach_board()
        elif self.state == "RECOVER_MARKER":
            self.handle_recover_marker()
        else:
            self.fail_mission(f"unknown_state_{self.state}")

    def run(self):
        self.start_keyboard_listener()
        rate = rospy.Rate(15)

        while not rospy.is_shutdown() and not self.finished:
            if self.handle_emergency_requests():
                break

            self.process_latest_image()
            self.control_logic()

            if self.show_debug and self.debug_image is not None:
                cv2.imshow("whiteboard_aruco_debug", self.debug_image)
                cv2.waitKey(1)

            rate.sleep()

        self.stop()
        if self.show_debug:
            cv2.destroyAllWindows()
        self.restore_terminal()


if __name__ == "__main__":
    mission = MissionWhiteboardAruco()
    mission.run()