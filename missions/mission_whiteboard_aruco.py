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
      1) ajustar camara, a una posiccion 
      2) buscar ArUco
      3) alinearse y orientarse correcatmente 
      4) aproximarse de forma segura
      5) moverse al punto de inicio del trazo
      6) moverse un poco mas para alcanzar a trazar en el pizarron 
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
        self.latest_known_area = 0

        # Odom
        self.current_x = None
        self.current_y = None
        self.current_z = None
        self.current_yaw = None

        # Reference variables for reaching board
        self.reach_start_x = None
        self.reach_start_y = None
        self.reach_start_z = None

        # State
        self.finished = False
        self.state = "SET_CAMERA"
        self.state_start_time = rospy.Time.now()
        self.start_time = rospy.Time.now()

        # Rotation refs
        self.rotate_yaw_start = None
        self.rotate_yaw_target = None

        # Board reached flag
        self.board_reached_confirmed = False

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
        self.camera_tilt_start = rospy.get_param("~camera_tilt_start", 0.0) # Apuntando al frente para no perderlo al inicio
        self.camera_pan_start = rospy.get_param("~camera_pan_start", 0.0)
        self.camera_settle_time = rospy.get_param("~camera_settle_time", 1.5)

        # Tolerances
        self.center_tolerance_x = rospy.get_param("~center_tolerance_x", 45) # Ampliada (era 30)
        self.center_tolerance_y = rospy.get_param("~center_tolerance_y", 40) # Ampliada (era 25)
        self.draw_target_tolerance_x = rospy.get_param("~draw_target_tolerance_x", 40) # Ampliada (era 25)
        self.draw_target_tolerance_y = rospy.get_param("~draw_target_tolerance_y", 40) # Ampliada (era 25)

        # Areas (Vision approach sizing)
        self.approach_area_safe = rospy.get_param("~approach_area_safe", 14000)
        self.approach_area_near = rospy.get_param("~approach_area_near", 18000)
        self.draw_start_area_threshold = rospy.get_param("~draw_start_area_threshold", 22000)
        self.max_safe_area = rospy.get_param("~max_safe_area", 30000) # Prevencion de choque basado en vision

        # Speeds
        self.search_yaw_speed = rospy.get_param("~search_yaw_speed", 0.15)
        self.align_lateral_speed = rospy.get_param("~align_lateral_speed", 0.08)
        self.align_vertical_speed = rospy.get_param("~align_vertical_speed", 0.08)

        self.approach_speed_far = rospy.get_param("~approach_speed_far", 0.05) # Mucho más lento desde lejos (era 0.08)
        self.approach_speed_near = rospy.get_param("~approach_speed_near", 0.015) # Super lento al acercarse (era 0.04)

        # Reach Board
        self.reach_forward_speed = rospy.get_param("~reach_forward_speed", 0.015)
        self.reach_forward_time_max = rospy.get_param("~reach_forward_time_max", 4.5)
        
        # Drawing
        self.draw_forward_bias = rospy.get_param("~draw_forward_bias", 0.0) # Sin presionar fuertemente
        self.draw_lateral_speed = rospy.get_param("~draw_lateral_speed", -0.07) # Dibujar de izq a derecha
        self.draw_time = rospy.get_param("~draw_time", 3.0)

        # Backoff & Landing
        self.backoff_speed = rospy.get_param("~backoff_speed", -0.07)
        self.backoff_time = rospy.get_param("~backoff_time", 2.5)

        self.rotate_right_speed = rospy.get_param("~rotate_right_speed", -0.15)
        self.rotate_timeout = rospy.get_param("~rotate_timeout", 8.0)
        self.rotate_yaw_tolerance = rospy.get_param("~rotate_yaw_tolerance", 0.08)

        # Timeouts / Odometry constraints
        self.detection_timeout = rospy.get_param("~detection_timeout", 0.8)
        self.marker_lost_timeout = rospy.get_param("~marker_lost_timeout", 1.0)
        self.max_mission_time = rospy.get_param("~max_mission_time", 90.0)
        self.post_land_wait = rospy.get_param("~post_land_wait", 2.0)

        # Odom based reaching limits
        self.reach_min_odometry_progress = rospy.get_param("~reach_min_odometry_progress", 0.02) # Tolerancia mas amplia (termina la mision con menos recorrido)

        rospy.loginfo("MissionWhiteboardAruco Reloaded (From 0) Initialized")

    # =====================================================
    # Keyboard Emergency Protocol
    # =====================================================

    def keyboard_listener(self):
        while not rospy.is_shutdown() and not self.finished:
            if select.select([sys.stdin], [], [], 0.05)[0]:
                key = sys.stdin.read(1)
                if key.lower() == 'q':
                    rospy.logwarn("EMERGENCIA LIGERA: Aterrizaje Inmediato solicitado.")
                    self.abort_land_requested = True
                    break
                elif key.lower() == 'e':
                    rospy.logerr("EMERGENCIA DURA: Reset de Motores solicitado.")
                    self.emergency_reset_requested = True
                    break

    def start_keyboard_listener(self):
        if not sys.stdin.isatty():
            return
        try:
            self.term_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
            self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
            self.keyboard_thread.daemon = True
            self.keyboard_thread.start()
        except: pass

    def restore_terminal(self):
        if self.term_settings is not None and sys.stdin.isatty():
            try: termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.term_settings)
            except: pass

    def handle_emergency_requests(self):
        if self.emergency_reset_requested:
            self.stop()
            rospy.logerr("Ejecutando Emergencia Dura...")
            self.pub_reset.publish(Empty())
            self.status_pub.publish("failed")
            self.finished = True
            return True
        if self.abort_land_requested:
            self.stop()
            rospy.logwarn("Ejecutando Aterrizaje...")
            self.pub_land.publish(Empty())
            self.status_pub.publish("failed")
            self.finished = True
            return True
        return False

    # =====================================================
    # Node Callbacks
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
    # Process Helpers
    # =====================================================

    def has_odom(self):
        return (self.current_x is not None)

    def process_latest_image(self):
        if self.latest_image_msg is None: return
        try: frame = self.bridge.imgmsg_to_cv2(self.latest_image_msg, "bgr8")
        except: return
        frame = cv2.resize(frame, (self.image_width, self.image_height))
        processed_img, data = self.detector.process_image(frame)
        self.debug_image = processed_img
        self.latest_data = data
        if data["detected"]:
            self.last_detection_time = rospy.Time.now()
            self.latest_known_area = data["area"]

    def set_state(self, new_state):
        if self.state != new_state:
            rospy.loginfo(f"--> MISSION STATE CHANGED TO: {new_state}")
            self.state = new_state
            self.state_start_time = rospy.Time.now()
            self.stop()

            # Record odom state for reaching blind phase
            if new_state == "REACH_BOARD" and self.has_odom():
                self.reach_start_x = self.current_x
                self.reach_start_y = self.current_y
                self.reach_start_z = self.current_z

    def elapsed_in_state(self):
        return (rospy.Time.now() - self.state_start_time).to_sec()

    def detection_recent(self):
        if not self.last_detection_time: return False
        return (rospy.Time.now() - self.last_detection_time).to_sec() < self.detection_timeout

    def publish_direct_twist(self, lx=0.0, ly=0.0, lz=0.0, az=0.0):
        t = Twist()
        t.linear.x, t.linear.y, t.linear.z, t.angular.z = lx, ly, lz, az
        self.pub_cmd.publish(t)

    def stop(self):
        self.movements.reset_twist()

    def normalize_angle(self, a):
        return math.atan2(math.sin(a), math.cos(a))

    def get_odom_progress_in_reach(self):
        if not self.has_odom() or self.reach_start_x is None: return 0.0
        dx = self.current_x - self.reach_start_x
        dy = self.current_y - self.reach_start_y
        dz = self.current_z - self.reach_start_z
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def finish_mission(self):
        self.stop()
        self.status_pub.publish("done")
        self.finished = True
        rospy.loginfo("MISION EXITOSA")

    def should_finish_after(self, target_stage):
        order = {
            "search_align": 1,
            "approach": 2,
            "draw_start": 3,
            "reach_board": 4,
            "draw": 5,
            "full": 6,
        }
        test_mapped = order.get(self.test_mode, 6)
        stage_mapped = order.get(target_stage, 999)
        return test_mapped <= stage_mapped

    # =====================================================
    # State Logic
    # =====================================================

    def step_set_camera(self):
        """1) Ajustar camara a una posicion"""
        t = Twist()
        t.angular.y = self.camera_tilt_start
        t.angular.z = self.camera_pan_start
        self.pub_camera.publish(t)

        if self.elapsed_in_state() >= self.camera_settle_time:
            self.set_state("SEARCH_ARUCO")

    def step_search_aruco(self):
        """2) Buscar ArUco"""
        data = self.latest_data
        if data and data["detected"]:
            self.set_state("ALIGN_AND_ORIENT")
        else:
            # Esperar 2 segundos flotando antes de empezar a rotar como loco
            if self.elapsed_in_state() < 2.0:
                self.publish_direct_twist(az=0.0)
            else:
                self.publish_direct_twist(az=self.search_yaw_speed)

    def step_align_and_orient(self):
        """3) Alinearse y orientarse correctamente"""
        data = self.latest_data
        if not data or not self.detection_recent():
            if self.latest_known_area > 8000:
                rospy.logwarn("ArUco perdido en Alineación Estando Cerca. Simulando éxito y avanzando para completar.")
                self.set_state("REACH_BOARD")
            else:
                self.set_state("SEARCH_ARUCO")
            return

        err_x, err_y = data["error_x"], data["error_y"]
        yaw_err = data.get("marker_yaw", 0.0)

        ok_x = abs(err_x) < self.center_tolerance_x
        ok_y = abs(err_y) < self.center_tolerance_y
        ok_yaw = abs(yaw_err) < 0.15

        if ok_x and ok_y and ok_yaw:
            if self.should_finish_after("search_align"):
                self.finish_mission()
                return
            self.set_state("APPROACH_SAFELY")
            return

        lx, ly, lz, az = 0.0, 0.0, 0.0, 0.0
        if not ok_y: lz = self.align_vertical_speed if err_y < 0 else -self.align_vertical_speed
        if not ok_yaw: ly = -self.align_lateral_speed if yaw_err > 0 else self.align_lateral_speed
        if not ok_x: az = self.search_yaw_speed if err_x < 0 else -self.search_yaw_speed
        self.publish_direct_twist(lx, ly, lz, az)

    def step_approach_safely(self):
        """4) Aproximarse de forma segura"""
        data = self.latest_data
        if not self.detection_recent() or not data["detected"]:
            if self.latest_known_area > 8000:
                rospy.logwarn("ArUco perdido cerca del pizarron (Approach). Saltando directo a completar mision.")
                self.set_state("REACH_BOARD")
            else:
                self.set_state("SEARCH_ARUCO")
            return

        area = data["area"]
        if area > self.max_safe_area:
            self.stop()
            self.set_state("REACH_BOARD")
            return

        if area >= self.approach_area_near:
            if self.should_finish_after("approach"):
                self.finish_mission()
                return
            self.set_state("MOVE_TO_DRAW_START")
            return
            
        progress = max(0.0, min(1.0, area / self.approach_area_near))
        dyn_lx = self.approach_speed_far - (progress * (self.approach_speed_far - self.approach_speed_near))
        dyn_lx = max(self.approach_speed_near, dyn_lx)

        self.publish_direct_twist(lx=dyn_lx)

    def step_move_to_draw_start(self):
        """5) Moverse al punto de inicio del trazo"""
        data = self.latest_data
        if not self.detection_recent() or not data["detected"]:
            if self.latest_known_area > 8000:
                rospy.logwarn("ArUco perdido preparandose para trazar. Saltando directo a completar mision.")
                self.set_state("REACH_BOARD")
            else:
                self.set_state("SEARCH_ARUCO")
            return

        tx, ty = data["target_draw_x"], data["target_draw_y"]
        if tx is None or ty is None: return

        err_x = tx - data["center_x"]
        err_y = ty - data["center_y"]

        ok_x = abs(err_x) < self.draw_target_tolerance_x
        ok_y = abs(err_y) < self.draw_target_tolerance_y

        area = data["area"]
        if (ok_x and ok_y) or area >= self.draw_start_area_threshold:
            if self.should_finish_after("draw_start"):
                self.finish_mission()
                return
            self.set_state("REACH_BOARD")
            return

        progress = min(1.0, area / self.draw_start_area_threshold)
        dyn_lx = self.approach_speed_near * (1.0 - progress)
        dyn_lx = max(0.010, dyn_lx)

        ly, lz, az = 0.0, 0.0, 0.0
        if not ok_y: lz = self.align_vertical_speed if err_y < 0 else -self.align_vertical_speed
        if not ok_x: az = self.search_yaw_speed if err_x < 0 else -self.search_yaw_speed
        
        self.publish_direct_twist(lx=dyn_lx, ly=ly, lz=lz, az=az)

    def step_reach_board(self):
        """6) Moverse un poco mas para alcanzar a trazar en el pizarron"""
        elapsed = self.elapsed_in_state()
        odom_progress = self.get_odom_progress_in_reach()

        data = self.latest_data
        if data and data["detected"] and data["area"] > self.max_safe_area:
            self.board_reached_confirmed = True
            if self.should_finish_after("reach_board"):
                self.finish_mission()
                return
            self.set_state("DRAW_LINE")
            return

        if elapsed >= self.reach_forward_time_max or odom_progress >= self.reach_min_odometry_progress:
            self.board_reached_confirmed = True
            if self.should_finish_after("reach_board"):
                self.finish_mission()
                return
            self.set_state("DRAW_LINE")
            return

        self.publish_direct_twist(lx=self.reach_forward_speed)

    def step_draw_line(self):
        """7) Dibujar linea"""
        if self.elapsed_in_state() > self.draw_time:
            if self.should_finish_after("draw"):
                self.finish_mission()
                return
            self.set_state("BACK_OFF")
            return

        self.publish_direct_twist(lx=self.draw_forward_bias, ly=self.draw_lateral_speed)

    def step_back_off(self):
        """8) Retroceder"""
        if self.elapsed_in_state() > self.backoff_time:
            self.set_state("ROTATE_RIGHT_90")
            return
        self.publish_direct_twist(lx=self.backoff_speed)

    def step_rotate_right_90(self):
        """9) Girar 90 grados a la derecha"""
        if self.current_yaw is not None:
            if self.rotate_yaw_start is None:
                self.rotate_yaw_start = self.current_yaw
                self.rotate_yaw_target = self.normalize_angle(self.current_yaw - math.pi/2.0)

            diff = self.normalize_angle(self.rotate_yaw_target - self.current_yaw)
            if abs(diff) < self.rotate_yaw_tolerance or self.elapsed_in_state() > self.rotate_timeout:
                self.set_state("LAND")
                return
        elif self.elapsed_in_state() > 2.5:
            self.set_state("LAND")
            return

        self.publish_direct_twist(az=self.rotate_right_speed)

    def step_land(self):
        """10) Aterrizar"""
        self.stop()
        self.pub_land.publish(Empty())
        rospy.sleep(self.post_land_wait)
        self.set_state("DONE")

    def control_logic(self):
        if self.finished: return
        t = (rospy.Time.now() - self.start_time).to_sec()
        if t > self.max_mission_time:
            self.stop()
            self.fail_mission("timeout global superado")
            return

        if self.state == "SET_CAMERA": self.step_set_camera()
        elif self.state == "SEARCH_ARUCO": self.step_search_aruco()
        elif self.state == "ALIGN_AND_ORIENT": self.step_align_and_orient()
        elif self.state == "APPROACH_SAFELY": self.step_approach_safely()
        elif self.state == "MOVE_TO_DRAW_START": self.step_move_to_draw_start()
        elif self.state == "REACH_BOARD": self.step_reach_board()
        elif self.state == "DRAW_LINE": self.step_draw_line()
        elif self.state == "BACK_OFF": self.step_back_off()
        elif self.state == "ROTATE_RIGHT_90": self.step_rotate_right_90()
        elif self.state == "LAND": self.step_land()
        elif self.state == "DONE": self.finish_mission()

    def fail_mission(self, msg="n/a"):
        rospy.logerr(f"Mision Fallida: {msg}")
        self.status_pub.publish("failed")
        self.finished = True

    def run(self):
        self.start_keyboard_listener()
        rate = rospy.Rate(15)
        while not rospy.is_shutdown() and not self.finished:
            if self.handle_emergency_requests(): break
            self.process_latest_image()
            self.control_logic()

            if self.show_debug and self.debug_image is not None:
                cv2.imshow("Mision Whiteboard ArUco - Vision Debug", self.debug_image)
                cv2.waitKey(1)
            rate.sleep()

        self.stop()
        if self.show_debug: cv2.destroyAllWindows()
        self.restore_terminal()

if __name__ == "__main__":
    node = MissionWhiteboardAruco()
    node.run()