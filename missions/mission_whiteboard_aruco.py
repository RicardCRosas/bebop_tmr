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
    Mision robusta para whiteboard:
      1) ajustar camara
      2) buscar ArUco
      3) alinearse
      4) aproximarse de forma segura
      5) moverse al punto de inicio del trazo
      6) tocar el pizarron (confirmacion por vision + odometria)
      7) dibujar linea
      8) retroceder
      9) girar 90 grados a la derecha
     10) aterrizar
    """

    def __init__(self):
        rospy.init_node('mission_whiteboard_aruco')

        # Publishers
        self.pub_cmd = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1)
        self.pub_takeoff = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1)
        self.pub_land = rospy.Publisher('/bebop/land', Empty, queue_size=1)
        self.pub_reset = rospy.Publisher('/bebop/reset', Empty, queue_size=1)
        self.pub_camera = rospy.Publisher('/bebop/camera_control', Twist, queue_size=1)
        self.status_pub = rospy.Publisher('/mission/status', String, queue_size=1)

        # Helpers
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
        self.prev_area = None

        # Odom
        self.current_x = None
        self.current_y = None
        self.current_z = None
        self.current_yaw = None

        # Touch reference
        self.touch_start_x = None
        self.touch_start_y = None
        self.touch_start_z = None

        # State
        self.finished = False
        self.state = "SET_CAMERA"
        self.state_start_time = rospy.Time.now()
        self.start_time = rospy.Time.now()

        # Rotation refs
        self.rotate_yaw_start = None
        self.rotate_yaw_target = None

        # Contact flag
        self.board_contact_confirmed = False

        # Emergency flags
        self.abort_land_requested = False
        self.emergency_reset_requested = False
        self.keyboard_thread = None
        self.term_settings = None

        # -------------------------
        # Params
        # -------------------------
        self.test_mode = rospy.get_param("~test_mode", "full")
        self.show_debug = rospy.get_param("~show_debug", True)
        self.image_width = rospy.get_param("~image_width", 640)
        self.image_height = rospy.get_param("~image_height", 360)

        # Camera
        self.camera_tilt_start = rospy.get_param("~camera_tilt_start", 20.0)
        self.camera_pan_start = rospy.get_param("~camera_pan_start", 0.0)
        self.camera_settle_time = rospy.get_param("~camera_settle_time", 1.2)

        # Centering tolerances
        self.center_tolerance_x = rospy.get_param("~center_tolerance_x", 35)
        self.center_tolerance_y = rospy.get_param("~center_tolerance_y", 30)

        self.draw_target_tolerance_x = rospy.get_param("~draw_target_tolerance_x", 30)
        self.draw_target_tolerance_y = rospy.get_param("~draw_target_tolerance_y", 30)

        # Areas
        self.approach_area_safe = rospy.get_param("~approach_area_safe", 12000)
        self.approach_area_near = rospy.get_param("~approach_area_near", 17000)
        self.touch_area_threshold = rospy.get_param("~touch_area_threshold", 23500)
        self.touch_confirm_area = rospy.get_param("~touch_confirm_area", 26000)
        self.max_safe_area = rospy.get_param("~max_safe_area", 32000)

        # Speeds - Velocidades mas bajas por seguridad
        self.search_yaw_speed = rospy.get_param("~search_yaw_speed", 0.10)
        self.align_lateral_speed = rospy.get_param("~align_lateral_speed", 0.08)
        self.align_vertical_speed = rospy.get_param("~align_vertical_speed", 0.08)

        self.approach_speed_far = rospy.get_param("~approach_speed_far", 0.06)
        self.approach_speed_near = rospy.get_param("~approach_speed_near", 0.035)

        self.touch_forward_speed = rospy.get_param("~touch_forward_speed", 0.025)
        self.touch_forward_time_max = rospy.get_param("~touch_forward_time_max", 2.5)

        self.draw_forward_bias = rospy.get_param("~draw_forward_bias", 0.015)
        self.draw_lateral_speed = rospy.get_param("~draw_lateral_speed", -0.06)
        self.draw_time = rospy.get_param("~draw_time", 3.4)

        self.backoff_speed = rospy.get_param("~backoff_speed", -0.06)
        self.backoff_time = rospy.get_param("~backoff_time", 2.3)

        self.rotate_right_speed = rospy.get_param("~rotate_right_speed", -0.12)
        self.rotate_timeout = rospy.get_param("~rotate_timeout", 8.0)
        self.rotate_yaw_tolerance = rospy.get_param("~rotate_yaw_tolerance", 0.10)

        # Detection / safety times
        self.detection_timeout = rospy.get_param("~detection_timeout", 0.5)
        self.marker_lost_timeout = rospy.get_param("~marker_lost_timeout", 0.7)
        self.max_mission_time = rospy.get_param("~max_mission_time", 75.0)
        self.post_land_wait = rospy.get_param("~post_land_wait", 2.0)

        # Odom-based touch confirmation
        self.touch_min_progress_for_no_contact = rospy.get_param("~touch_min_progress_for_no_contact", 0.05)
        self.touch_low_progress_threshold = rospy.get_param("~touch_low_progress_threshold", 0.025)
        self.area_growth_small_threshold = rospy.get_param("~area_growth_small_threshold", 350.0)

        rospy.loginfo("MissionWhiteboardAruco initialized")
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
            rospy.logerr("Enviando /bebop/reset ...")
            self.pub_reset.publish(Empty())
            self.status_pub.publish("failed")
            self.finished = True
            return True

        if self.abort_land_requested:
            self.stop()
            rospy.sleep(0.1)
            rospy.logwarn("Enviando /bebop/land ...")
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

    def set_camera_pose(self, tilt_deg, pan_deg):
        cam = Twist()
        cam.angular.y = tilt_deg
        cam.angular.z = pan_deg
        self.pub_camera.publish(cam)

    def set_state(self, new_state):
        if self.state != new_state:
            rospy.loginfo(f"MISSION STATE -> {new_state}")
            self.state = new_state
            self.state_start_time = rospy.Time.now()
            self.stop()

            if new_state == "TOUCH_BOARD" and self.has_odom():
                self.touch_start_x = self.current_x
                self.touch_start_y = self.current_y
                self.touch_start_z = self.current_z

            if new_state == "ROTATE_RIGHT_90":
                self.rotate_yaw_start = None
                self.rotate_yaw_target = None

    def elapsed_in_state(self):
        return (rospy.Time.now() - self.state_start_time).to_sec()

    def mission_elapsed(self):
        return (rospy.Time.now() - self.start_time).to_sec()

    def detection_recent(self):
        if self.last_detection_time is None:
            return False
        return (rospy.Time.now() - self.last_detection_time).to_sec() < self.detection_timeout

    def marker_lost_for_too_long(self):
        if self.last_detection_time is None:
            return True
        return (rospy.Time.now() - self.last_detection_time).to_sec() > self.marker_lost_timeout

    def publish_direct_twist(self, lx=0.0, ly=0.0, lz=0.0, az=0.0):
        msg = Twist()
        msg.linear.x = lx
        msg.linear.y = ly
        msg.linear.z = lz
        msg.angular.z = az
        self.pub_cmd.publish(msg)

    def stop(self):
        self.movements.reset_twist()

    def normalize_angle(self, a):
        return math.atan2(math.sin(a), math.cos(a))

    def odom_progress_since_touch_start(self):
        if not self.has_odom():
            return None
        if self.touch_start_x is None or self.touch_start_y is None or self.touch_start_z is None:
            return None

        dx = self.current_x - self.touch_start_x
        dy = self.current_y - self.touch_start_y
        dz = self.current_z - self.touch_start_z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def area_growth_small(self, current_area):
        if self.prev_area is None:
            self.prev_area = current_area
            return False

        delta = abs(current_area - self.prev_area)
        self.prev_area = current_area
        return delta < self.area_growth_small_threshold

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

        self.set_state("ALIGN_TO_MARKER")

    def handle_align_to_marker(self):
        data = self.latest_data

        if data is None or not self.detection_recent() or not data["detected"]:
            self.set_state("SEARCH_ARUCO")
            return

        err_x = data["error_x"]
        err_y = data["error_y"]
        marker_yaw = data.get("marker_yaw", 0.0)

        aligned_x = abs(err_x) <= self.center_tolerance_x
        aligned_y = abs(err_y) <= self.center_tolerance_y
        aligned_yaw = abs(marker_yaw) <= 0.15

        if aligned_x and aligned_y and aligned_yaw:
            self.stop()

            if self.should_finish_after("ALIGN_OK"):
                self.finish_mission()
                return

            self.set_state("APPROACH_BOARD")
            return

        ly = 0.0
        lz = 0.0
        az = 0.0

        if not aligned_y:
            lz = self.align_vertical_speed if err_y < 0 else -self.align_vertical_speed

        if not aligned_yaw:
            # Correccion de angulo con desplazamiento lateral (strafe)
            ly = -self.align_lateral_speed if marker_yaw > 0 else self.align_lateral_speed

        if not aligned_x:
            # Mantener centrado girando la camara (yaw)
            az = self.search_yaw_speed if err_x < 0 else -self.search_yaw_speed

        self.publish_direct_twist(ly=ly, lz=lz, az=az)

    def handle_approach_board(self):
        data = self.latest_data

        if data is None or not data["detected"] or self.marker_lost_for_too_long():
            self.stop()
            self.set_state("SEARCH_ARUCO")
            return

        err_x = data["error_x"]
        err_y = data["error_y"]
        marker_yaw = data.get("marker_yaw", 0.0)
        area = data["area"]

        if area >= self.max_safe_area:
            rospy.logwarn("Area demasiado alta; frenando para evitar choque.")
            self.stop()
            self.set_state("TOUCH_BOARD")
            return

        aligned_x = abs(err_x) <= self.center_tolerance_x
        aligned_y = abs(err_y) <= self.center_tolerance_y
        aligned_yaw = abs(marker_yaw) <= 0.20

        if not (aligned_x and aligned_y and aligned_yaw):
            ly = 0.0
            lz = 0.0
            az = 0.0
            lx = 0.0
            
            if not aligned_y:
                lz = self.align_vertical_speed * 0.75 if err_y < 0 else -self.align_vertical_speed * 0.75
            
            if not aligned_yaw:
                ly = -self.align_lateral_speed * 0.75 if marker_yaw > 0 else self.align_lateral_speed * 0.75

            if not aligned_x:
                az = self.search_yaw_speed * 0.75 if err_x < 0 else -self.search_yaw_speed * 0.75
                
            if area < self.approach_area_near:
                lx = self.approach_speed_far * 0.5
                
            self.publish_direct_twist(lx=lx, ly=ly, lz=lz, az=az)
            return

        if area < self.approach_area_safe:
            self.publish_direct_twist(lx=self.approach_speed_far)
            return

        if area < self.approach_area_near:
            self.publish_direct_twist(lx=self.approach_speed_near)
            return

        self.stop()

        if self.should_finish_after("APPROACH_OK"):
            self.finish_mission()
            return

        self.set_state("MOVE_TO_DRAW_START")

    def handle_move_to_draw_start(self):
        data = self.latest_data

        if data is None or not data["detected"] or self.marker_lost_for_too_long():
            rospy.logwarn("Se perdio el ArUco cerca del pizarron. Reintentando busqueda.")
            self.stop()
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
        marker_yaw = data.get("marker_yaw", 0.0)
        area = data["area"]

        aligned_x = abs(err_x) <= self.draw_target_tolerance_x
        aligned_y = abs(err_y) <= self.draw_target_tolerance_y
        aligned_yaw = abs(marker_yaw) <= 0.15

        if area >= self.max_safe_area:
            rospy.logwarn("Peligro de choque (inercia) en MOVE_TO_DRAW_START. Abortando alineacion.")
            self.stop()
            self.set_state("TOUCH_BOARD")
            return

        if not (aligned_x and aligned_y and aligned_yaw):
            ly = 0.0
            lz = 0.0
            az = 0.0
            
            if not aligned_y:
                lz = self.align_vertical_speed * 0.75 if err_y < 0 else -self.align_vertical_speed * 0.75
                
            if not aligned_x:
                # Target lateral centrado por strafing
                ly = self.align_lateral_speed * 0.75 if err_x < 0 else -self.align_lateral_speed * 0.75
                
            if not aligned_yaw:
                # Mantener perpendicularidad usando yaw
                az = -self.search_yaw_speed * 0.5 if marker_yaw > 0 else self.search_yaw_speed * 0.5
                
            self.publish_direct_twist(ly=ly, lz=lz, az=az)
            return

        if area < self.touch_area_threshold:
            self.publish_direct_twist(lx=self.approach_speed_near)
            return

        self.stop()

        if self.should_finish_after("DRAW_START_OK"):
            self.finish_mission()
            return

        self.set_state("TOUCH_BOARD")

    def handle_touch_board(self):
        data = self.latest_data

        if data is None or self.marker_lost_for_too_long():
            rospy.logwarn("Referencia perdida en toque. Retrocediendo por seguridad.")
            self.stop()
            self.set_state("BACK_OFF")
            return

        area = data["area"]
        tx = data["target_draw_x"]
        ty = data["target_draw_y"]
        elapsed = self.elapsed_in_state()

        if area >= self.max_safe_area:
            rospy.logwarn("SAFE AREA SUPERADA. Parada de emergencia. Deteniendo avance y confirmando toque.")
            self.board_contact_confirmed = True
            self.stop()

            if self.should_finish_after("TOUCH_OK"):
                self.finish_mission()
                return

            self.set_state("DRAW_LINE")
            return

        # Alineacion final al punto de dibujo
        err_x = tx - data["center_x"] if tx is not None else 0
        err_y = ty - data["center_y"] if ty is not None else 0
        aligned_draw = (
            abs(err_x) <= self.draw_target_tolerance_x and
            abs(err_y) <= self.draw_target_tolerance_y
        )

        progress = self.odom_progress_since_touch_start()
        growth_small = self.area_growth_small(area)

        # Confirmacion combinada:
        # 1) vision fuerte
        if area >= self.touch_confirm_area and aligned_draw:
            rospy.loginfo("Toque confirmado por vision fuerte.")
            self.board_contact_confirmed = True
            self.stop()

            if self.should_finish_after("TOUCH_OK"):
                self.finish_mission()
                return

            self.set_state("DRAW_LINE")
            return

        # 2) vision + poco avance real + tiempo corto
        if elapsed >= 0.45 and progress is not None:
            if area >= self.touch_area_threshold and progress <= self.touch_low_progress_threshold and growth_small:
                rospy.loginfo(
                    f"Toque confirmado por vision + odometria. area={area:.1f}, progress={progress:.4f}"
                )
                self.board_contact_confirmed = True
                self.stop()

                if self.should_finish_after("TOUCH_OK"):
                    self.finish_mission()
                    return

                self.set_state("DRAW_LINE")
                return

        # 3) timeout final controlado
        if elapsed >= self.touch_forward_time_max:
            rospy.loginfo("Tiempo maximo de toque alcanzado. Confirmando toque controlado.")
            self.board_contact_confirmed = True
            self.stop()

            if self.should_finish_after("TOUCH_OK"):
                self.finish_mission()
                return

            self.set_state("DRAW_LINE")
            return

        self.publish_direct_twist(lx=self.touch_forward_speed)

    def handle_draw_line(self):
        if not self.board_contact_confirmed:
            rospy.logwarn("No hay contacto confirmado. Cancelando trazo.")
            self.set_state("BACK_OFF")
            return

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
        self.set_state("ROTATE_RIGHT_90")

    def handle_rotate_right_90(self):
        if self.current_yaw is not None:
            if self.rotate_yaw_start is None:
                self.rotate_yaw_start = self.current_yaw
                self.rotate_yaw_target = self.normalize_angle(self.rotate_yaw_start - math.pi / 2.0)
                rospy.loginfo(
                    f"Rotacion derecha 90 iniciada. yaw_start={self.rotate_yaw_start:.3f}, yaw_target={self.rotate_yaw_target:.3f}"
                )

            yaw_error = self.normalize_angle(self.rotate_yaw_target - self.current_yaw)

            if abs(yaw_error) <= self.rotate_yaw_tolerance:
                self.stop()
                self.set_state("LAND")
                return

            if self.elapsed_in_state() > self.rotate_timeout:
                rospy.logwarn("Timeout en rotacion. Continuando a aterrizaje.")
                self.stop()
                self.set_state("LAND")
                return

            self.publish_direct_twist(az=self.rotate_right_speed)
            return

        # Fallback si no hay yaw
        if self.elapsed_in_state() < 2.0:
            self.publish_direct_twist(az=self.rotate_right_speed)
            return

        self.stop()
        self.set_state("LAND")

    def handle_land(self):
        self.stop()
        rospy.sleep(0.2)
        rospy.loginfo("Aterrizando...")
        self.pub_land.publish(Empty())
        rospy.sleep(self.post_land_wait)
        self.set_state("DONE")

    def handle_done(self):
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
        elif self.state == "MOVE_TO_DRAW_START":
            self.handle_move_to_draw_start()
        elif self.state == "TOUCH_BOARD":
            self.handle_touch_board()
        elif self.state == "DRAW_LINE":
            self.handle_draw_line()
        elif self.state == "BACK_OFF":
            self.handle_back_off()
        elif self.state == "ROTATE_RIGHT_90":
            self.handle_rotate_right_90()
        elif self.state == "LAND":
            self.handle_land()
        elif self.state == "DONE":
            self.handle_done()
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