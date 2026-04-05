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
    Misión whiteboard con lógica más segura para pruebas.

    IDEAS CLAVE:
    - El ArUco NO se usa para una alineación perfecta.
    - Solo se usa como referencia aproximada para saber dónde está el pizarrón.
    - Si el dron pierde el ArUco cerca del pizarrón, NO entra a búsqueda agresiva.
    - Primero se queda quieto.
    - Luego retrocede un poco.
    - Si no lo recupera, falla en seguro.
    - El avance es corto y lento para pruebas.
    """

    def __init__(self):
        rospy.init_node("mission_whiteboard_aruco")

        # Publishers
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

        # Subscribers
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

        # Vision
        self.latest_image_msg = None
        self.latest_data = None
        self.debug_image = None
        self.last_detection_time = None
        self.last_good_data = None
        self.last_seen_area = 0.0

        # filtered vision for smoother control
        self.filtered_error_x = None
        self.filtered_error_y = None
        self.filtered_yaw_error = None
        self.filtered_area = None

        # safety on marker loss
        self.loss_is_close = False

        # Odom
        self.current_x = None
        self.current_y = None
        self.current_z = None
        self.current_yaw = None

        # References
        self.approach_start_x = None
        self.approach_start_y = None
        self.approach_start_z = None

        self.predraw_start_x = None
        self.predraw_start_y = None
        self.predraw_start_z = None

        self.rotate_yaw_start = None
        self.rotate_yaw_target = None
        self.recover_return_state = None

        # Mission state
        self.finished = False
        self.state = "SET_CAMERA"
        self.state_start_time = rospy.Time.now()
        self.start_time = rospy.Time.now()

        # Detection hysteresis
        self.detect_stable_count = 0
        self.lost_count = 0

        # Emergency
        self.abort_land_requested = False
        self.emergency_reset_requested = False
        self.term_settings = None
        self.keyboard_thread = None

        # =====================================================
        # Params
        # =====================================================

        self.test_mode = rospy.get_param("~test_mode", "approach")
        self.show_debug = rospy.get_param("~show_debug", True)
        self.image_width = rospy.get_param("~image_width", 640)
        self.image_height = rospy.get_param("~image_height", 360)

        # Camera
        self.camera_tilt_start = rospy.get_param("~camera_tilt_start", 18.0)
        self.camera_pan_start = rospy.get_param("~camera_pan_start", 0.0)
        self.camera_settle_time = rospy.get_param("~camera_settle_time", 1.5)

        # -----------------------------------------------------
        # Alineación SUAVE, no total
        # -----------------------------------------------------
        # Se dejan tolerancias más abiertas para que el dron no
        # se obsesione con centrar perfecto el ArUco.
        self.center_tolerance_x = rospy.get_param("~center_tolerance_x", 55)
        self.center_tolerance_y = rospy.get_param("~center_tolerance_y", 42)
        self.visual_yaw_tolerance = rospy.get_param("~visual_yaw_tolerance", 0.18)

        # En approach todavía más permisivo
        self.approach_tolerance_x = rospy.get_param("~approach_tolerance_x", 75)
        self.approach_tolerance_y = rospy.get_param("~approach_tolerance_y", 58)
        self.approach_visual_yaw_tolerance = rospy.get_param("~approach_visual_yaw_tolerance", 0.24)

        # Detecciones estables
        self.min_stable_detections = rospy.get_param("~min_stable_detections", 10)
        self.max_lost_before_recover = rospy.get_param("~max_lost_before_recover", 18)
        self.recover_hold_time = rospy.get_param("~recover_hold_time", 1.2)
        self.align_hold_time = rospy.get_param("~align_hold_time", 0.8)

        # -----------------------------------------------------
        # Velocidades muy suaves
        # -----------------------------------------------------
        self.search_yaw_speed = rospy.get_param("~search_yaw_speed", 0.012)
        self.align_lateral_speed = rospy.get_param("~align_lateral_speed", 0.010)
        self.align_vertical_speed = rospy.get_param("~align_vertical_speed", 0.010)

        self.approach_speed = rospy.get_param("~approach_speed", 0.0015)
        self.approach_yaw_gain = rospy.get_param("~approach_yaw_gain", 0.18)
        self.approach_yaw_max = rospy.get_param("~approach_yaw_max", 0.010)

        # -----------------------------------------------------
        # IMPORTANTE:
        # Distancia corta para pruebas.
        # Así NO se acerca mucho al pizarrón.
        # -----------------------------------------------------
        self.safe_approach_travel = rospy.get_param("~safe_approach_travel", 0.02)
        self.safe_approach_timeout = rospy.get_param("~safe_approach_timeout", 2.2)

        # Freno visual secundario
        self.max_safe_area = rospy.get_param("~max_safe_area", 2200)

        # -----------------------------------------------------
        # Seguridad extra cuando pierde el ArUco
        # -----------------------------------------------------
        self.resume_stable_detections = rospy.get_param("~resume_stable_detections", 5)
        self.allow_search_after_loss = rospy.get_param("~allow_search_after_loss", False)
        self.near_board_area = rospy.get_param("~near_board_area", 1450)
        self.loss_hover_time = rospy.get_param("~loss_hover_time", 0.8)
        self.loss_backoff_speed = rospy.get_param("~loss_backoff_speed", -0.018)
        self.loss_backoff_time = rospy.get_param("~loss_backoff_time", 0.8)
        self.filtered_alpha = rospy.get_param("~filtered_alpha", 0.35)

        # -----------------------------------------------------
        # Pre-draw muy conservador
        # -----------------------------------------------------
        self.pre_draw_speed = rospy.get_param("~pre_draw_speed", 0.003)
        self.pre_draw_travel = rospy.get_param("~pre_draw_travel", 0.002)
        self.pre_draw_timeout = rospy.get_param("~pre_draw_timeout", 0.8)

        # Dibujo
        self.draw_forward_bias = rospy.get_param("~draw_forward_bias", 0.003)
        self.draw_lateral_speed = rospy.get_param("~draw_lateral_speed", -0.040)
        self.draw_time = rospy.get_param("~draw_time", 0.8)

        # Separación
        self.backoff_speed = rospy.get_param("~backoff_speed", -0.05)
        self.backoff_time = rospy.get_param("~backoff_time", 1.1)

        # Giro opcional
        self.enable_rotate_before_land = rospy.get_param("~enable_rotate_before_land", False)
        self.rotate_right_speed = rospy.get_param("~rotate_right_speed", -0.12)
        self.rotate_timeout = rospy.get_param("~rotate_timeout", 2.5)
        self.rotate_yaw_tolerance = rospy.get_param("~rotate_yaw_tolerance", 0.12)

        self.max_mission_time = rospy.get_param("~max_mission_time", 45.0)
        self.post_land_wait = rospy.get_param("~post_land_wait", 2.0)

        rospy.loginfo("MissionWhiteboardAruco SAFE TEST version initialized")
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

    def _smooth_value(self, previous, current):
        if current is None:
            return previous
        if previous is None:
            return float(current)
        a = self.filtered_alpha
        return ((1.0 - a) * float(previous)) + (a * float(current))

    def marker_is_close(self):
        area = self.last_seen_area
        if self.latest_data is not None and self.latest_data.get("detected", False):
            area = self.latest_data.get("area", area)
        return area >= self.near_board_area

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
        show_draw_ref = self.state in ["PRE_DRAW", "DRAW_LINE", "BACK_OFF", "LAND"]
        processed_image, data = self.detector.process_image(frame, show_draw_ref=show_draw_ref)

        if data["detected"]:
            yaw_error = self.compute_visual_yaw_error(data)
            self.filtered_error_x = self._smooth_value(self.filtered_error_x, data["error_x"])
            self.filtered_error_y = self._smooth_value(self.filtered_error_y, data["error_y"])
            self.filtered_yaw_error = self._smooth_value(self.filtered_yaw_error, yaw_error)
            self.filtered_area = self._smooth_value(self.filtered_area, data["area"])

            data["error_x_f"] = self.filtered_error_x
            data["error_y_f"] = self.filtered_error_y
            data["yaw_error_f"] = self.filtered_yaw_error
            data["area_f"] = self.filtered_area

            self.last_good_data = dict(data)
            self.last_seen_area = float(data["area"])
            self.last_detection_time = rospy.Time.now()
            self.detect_stable_count += 1
            self.lost_count = 0
        else:
            data["error_x_f"] = self.filtered_error_x
            data["error_y_f"] = self.filtered_error_y
            data["yaw_error_f"] = self.filtered_yaw_error
            data["area_f"] = self.filtered_area
            self.detect_stable_count = 0
            self.lost_count += 1

        self.debug_image = processed_image
        self.latest_data = data

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

            if new_state == "PRE_DRAW" and self.has_odom():
                self.predraw_start_x = self.current_x
                self.predraw_start_y = self.current_y
                self.predraw_start_z = self.current_z

            if new_state == "RECOVER_MARKER":
                self.recover_return_state = recover_return_state

            if new_state == "LOSS_HOVER":
                self.recover_return_state = recover_return_state
                self.loss_is_close = self.marker_is_close()

            if new_state == "LOSS_BACKOFF":
                self.loss_is_close = self.marker_is_close()

            if new_state == "ROTATE_RIGHT_90":
                self.rotate_yaw_start = None
                self.rotate_yaw_target = None

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

    def odom_distance(self, x0, y0, z0):
        if not self.has_odom():
            return None
        if x0 is None:
            return None

        dx = self.current_x - x0
        dy = self.current_y - y0
        dz = self.current_z - z0
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
            "pre_draw": 3,
            "draw": 4,
            "full": 5,
        }

        current_stage_map = {
            "ALIGN_OK": 1,
            "APPROACH_OK": 2,
            "PREDRAW_OK": 3,
            "DRAW_OK": 4,
            "FULL_OK": 5,
        }

        wanted = order.get(self.test_mode, 5)
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
            if self.lost_count > self.max_lost_before_recover:
                self.set_state("LOSS_HOVER", recover_return_state="ALIGN_TO_MARKER")
            return

        err_x = data.get("error_x_f", data["error_x"])
        err_y = data.get("error_y_f", data["error_y"])
        yaw_error = data.get("yaw_error_f", self.compute_visual_yaw_error(data))

        aligned_x = abs(err_x) <= self.center_tolerance_x
        aligned_y = abs(err_y) <= self.center_tolerance_y
        aligned_yaw = abs(yaw_error) <= self.visual_yaw_tolerance

        # 1) corrige yaw suave
        if not aligned_yaw:
            wz = max(min(self.approach_yaw_gain * yaw_error, self.approach_yaw_max),
                     -self.approach_yaw_max)
            self.publish_direct_twist(az=wz)
            return

        # 2) corrige lateral suave
        if not aligned_x:
            if err_x < 0:
                self.publish_direct_twist(ly=self.align_lateral_speed)
            else:
                self.publish_direct_twist(ly=-self.align_lateral_speed)
            return

        # 3) corrige vertical suave
        if not aligned_y:
            if err_y < 0:
                self.publish_direct_twist(lz=self.align_vertical_speed)
            else:
                self.publish_direct_twist(lz=-self.align_vertical_speed)
            return

        # 4) NO busca perfección, solo referencia suficientemente buena
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
            if self.lost_count > self.max_lost_before_recover:
                self.set_state("LOSS_HOVER", recover_return_state="APPROACH_BOARD")
            return

        area = data.get("area_f", data["area"])
        if area >= self.max_safe_area:
            rospy.logwarn("Área demasiado alta. Frenando por seguridad.")
            self.stop()

            if self.should_finish_after("APPROACH_OK"):
                self.finish_mission()
                return

            self.set_state("PRE_DRAW")
            return

        err_x = data.get("error_x_f", data["error_x"])
        err_y = data.get("error_y_f", data["error_y"])
        yaw_error = data.get("yaw_error_f", self.compute_visual_yaw_error(data))

        if self.detect_stable_count < self.resume_stable_detections:
            self.stop()
            return

        ly = 0.0
        lz = 0.0
        wz = 0.0

        if abs(err_x) > self.approach_tolerance_x:
            ly = self.align_lateral_speed if err_x < 0 else -self.align_lateral_speed

        if abs(err_y) > self.approach_tolerance_y:
            lz = self.align_vertical_speed if err_y < 0 else -self.align_vertical_speed

        if abs(yaw_error) > self.approach_visual_yaw_tolerance:
            wz = max(min(self.approach_yaw_gain * yaw_error, self.approach_yaw_max),
                     -self.approach_yaw_max)

        misaligned = (
            abs(err_x) > self.approach_tolerance_x or
            abs(err_y) > self.approach_tolerance_y or
            abs(yaw_error) > self.approach_visual_yaw_tolerance
        )

        progress = self.odom_distance(
            self.approach_start_x,
            self.approach_start_y,
            self.approach_start_z
        )

        # límite por odometría para NO acercarse mucho
        if progress is not None and progress >= self.safe_approach_travel:
            rospy.loginfo(f"Distancia segura alcanzada por odometría: {progress:.3f} m")
            self.stop()

            if self.should_finish_after("APPROACH_OK"):
                self.finish_mission()
                return

            self.set_state("PRE_DRAW")
            return

        # límite por tiempo por seguridad
        if self.elapsed_in_state() > self.safe_approach_timeout:
            rospy.logwarn("Timeout en APPROACH_BOARD. Deteniendo por seguridad.")
            self.stop()

            if self.should_finish_after("APPROACH_OK"):
                self.finish_mission()
                return

            self.set_state("PRE_DRAW")
            return

        # corrige pero NO avanza si todavía está muy chueco
        if misaligned:
            self.publish_direct_twist(
                lx=0.0,
                ly=ly,
                lz=lz,
                az=wz
            )
            return

        # avanza muy lento
        self.publish_direct_twist(
            lx=self.approach_speed,
            ly=0.0,
            lz=0.0,
            az=0.0
        )

    def handle_pre_draw(self):
        data = self.latest_data

        if data is None or not data["detected"]:
            self.stop()
            if self.lost_count > self.max_lost_before_recover:
                self.set_state("LOSS_HOVER", recover_return_state="PRE_DRAW")
            return

        area = data.get("area_f", data["area"])
        if area >= self.max_safe_area:
            self.stop()

            if self.should_finish_after("PREDRAW_OK"):
                self.finish_mission()
                return

            self.set_state("DRAW_LINE")
            return

        err_x = data.get("error_x_f", data["error_x"])
        err_y = data.get("error_y_f", data["error_y"])
        yaw_error = data.get("yaw_error_f", self.compute_visual_yaw_error(data))

        if self.detect_stable_count < self.resume_stable_detections:
            self.stop()
            return

        ly = 0.0
        lz = 0.0
        wz = 0.0

        if abs(err_x) > self.approach_tolerance_x:
            ly = self.align_lateral_speed if err_x < 0 else -self.align_lateral_speed

        if abs(err_y) > self.approach_tolerance_y:
            lz = self.align_vertical_speed if err_y < 0 else -self.align_vertical_speed

        if abs(yaw_error) > self.approach_visual_yaw_tolerance:
            wz = max(min(self.approach_yaw_gain * yaw_error, self.approach_yaw_max),
                     -self.approach_yaw_max)

        misaligned = (
            abs(err_x) > self.approach_tolerance_x or
            abs(err_y) > self.approach_tolerance_y or
            abs(yaw_error) > self.approach_visual_yaw_tolerance
        )

        progress = self.odom_distance(
            self.predraw_start_x,
            self.predraw_start_y,
            self.predraw_start_z
        )

        if progress is not None and progress >= self.pre_draw_travel:
            self.stop()

            if self.should_finish_after("PREDRAW_OK"):
                self.finish_mission()
                return

            self.set_state("DRAW_LINE")
            return

        if self.elapsed_in_state() > self.pre_draw_timeout:
            self.stop()

            if self.should_finish_after("PREDRAW_OK"):
                self.finish_mission()
                return

            self.set_state("DRAW_LINE")
            return

        if misaligned:
            self.publish_direct_twist(
                lx=0.0,
                ly=ly,
                lz=lz,
                az=wz
            )
            return

        self.publish_direct_twist(
            lx=self.pre_draw_speed,
            ly=0.0,
            lz=0.0,
            az=0.0
        )

    def handle_draw_line(self):
        elapsed = self.elapsed_in_state()

        if elapsed < self.draw_time:
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

        if self.test_mode == "full":
            if self.enable_rotate_before_land:
                self.set_state("ROTATE_RIGHT_90")
            else:
                self.set_state("LAND")
        else:
            self.finish_mission()

    def handle_loss_hover(self):
        # al perder el ArUco cerca del pizarrón:
        # 1) se queda quieto
        # 2) espera un poco
        # 3) si no lo recupera, retrocede
        self.stop()

        data = self.latest_data
        if data is not None and data.get("detected", False):
            if self.detect_stable_count >= self.resume_stable_detections:
                next_state = self.recover_return_state if self.recover_return_state is not None else "ALIGN_TO_MARKER"
                self.set_state(next_state)
                return

        if self.elapsed_in_state() < self.loss_hover_time:
            return

        if self.loss_is_close or not self.allow_search_after_loss:
            self.set_state("LOSS_BACKOFF")
            return

        self.set_state("SEARCH_ARUCO")

    def handle_loss_backoff(self):
        data = self.latest_data
        if data is not None and data.get("detected", False):
            if self.detect_stable_count >= self.resume_stable_detections:
                next_state = self.recover_return_state if self.recover_return_state is not None else "ALIGN_TO_MARKER"
                self.set_state(next_state)
                return

        if self.elapsed_in_state() < self.loss_backoff_time:
            self.publish_direct_twist(lx=self.loss_backoff_speed)
            return

        self.stop()
        self.fail_mission("marker_lost_close_range")

    def handle_recover_marker(self):
        self.stop()

        data = self.latest_data
        if data is not None and data["detected"]:
            if self.detect_stable_count >= self.min_stable_detections:
                next_state = self.recover_return_state if self.recover_return_state is not None else "ALIGN_TO_MARKER"
                self.set_state(next_state)
                return

        if self.elapsed_in_state() > self.recover_hold_time:
            self.set_state("SEARCH_ARUCO")

    def handle_rotate_right_90(self):
        if self.current_yaw is not None:
            if self.rotate_yaw_start is None:
                self.rotate_yaw_start = self.current_yaw
                self.rotate_yaw_target = self.rotate_yaw_start - math.pi / 2.0

            yaw_error = self.rotate_yaw_target - self.current_yaw
            yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

            if abs(yaw_error) <= self.rotate_yaw_tolerance:
                self.stop()
                self.set_state("LAND")
                return

            if self.elapsed_in_state() > self.rotate_timeout:
                self.stop()
                self.set_state("LAND")
                return

            self.publish_direct_twist(az=self.rotate_right_speed)
            return

        if self.elapsed_in_state() < 2.0:
            self.publish_direct_twist(az=self.rotate_right_speed)
            return

        self.stop()
        self.set_state("LAND")

    def handle_land(self):
        self.stop()
        rospy.sleep(0.2)
        self.pub_land.publish(Empty())
        rospy.sleep(self.post_land_wait)
        self.finish_mission()

    # =====================================================
    # Main logic
    # =====================================================

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
        elif self.state == "PRE_DRAW":
            self.handle_pre_draw()
        elif self.state == "DRAW_LINE":
            self.handle_draw_line()
        elif self.state == "BACK_OFF":
            self.handle_back_off()
        elif self.state == "LOSS_HOVER":
            self.handle_loss_hover()
        elif self.state == "LOSS_BACKOFF":
            self.handle_loss_backoff()
        elif self.state == "RECOVER_MARKER":
            self.handle_recover_marker()
        elif self.state == "ROTATE_RIGHT_90":
            self.handle_rotate_right_90()
        elif self.state == "LAND":
            self.handle_land()
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