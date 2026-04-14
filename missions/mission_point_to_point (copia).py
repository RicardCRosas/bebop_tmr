#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
import threading
import sys
import select
import termios
import tty

from collections import deque
from std_msgs.msg import Empty, Float32, Float32MultiArray, Int32, Int32MultiArray, String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class MissionPointToPointPercentAnyTube:
    def __init__(self):
        rospy.init_node("mission_point_to_point_decision")

        self.takeoff_topic = rospy.get_param("~takeoff_topic", "/bebop/takeoff")
        self.land_topic = rospy.get_param("~land_topic", "/bebop/land")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/bebop/cmd_vel")
        self.odom_topic = rospy.get_param("~odom_topic", "/bebop/odom")
        self.average_topic = rospy.get_param("~average_topic", "/average_color")
        self.bboxes_topic = rospy.get_param("~bboxes_topic", "/orange_bboxes")
        self.solution_topic = rospy.get_param("~solution_topic", "/mission_solution")
        self.goal_pose_topic = rospy.get_param("~goal_pose_topic", "/goal_pose")
        self.numero_tubos_topic = rospy.get_param("~numero_tubos_topic", "/numero_tubos")
        self.mission_case_topic = rospy.get_param("~mission_case_topic", "/mission_case")

        # Tiempos
        self.initial_hover_time = rospy.get_param("~initial_hover_time", 2.0)
        self.final_hold_time = rospy.get_param("~final_hold_time", 1.0)
        self.ctrl_rate = rospy.get_param("~ctrl_rate", 30.0)

        # Rango de parada
        self.percent_low = rospy.get_param("~percent_low", 21.0)
        self.percent_high = rospy.get_param("~percent_high", 25.0)
        self.target_percent = rospy.get_param("~target_percent", 23.5)

        # Distancia final si no hay tubos
        self.max_forward_distance = rospy.get_param("~max_forward_distance", 2.0)

        # Centro del frame (680x480 -> 340x240)
        self.frame_w = rospy.get_param("~frame_w", 680)
        self.frame_h = rospy.get_param("~frame_h", 480)
        self.center_frame_x = self.frame_w // 2
        self.center_frame_y = self.frame_h // 2

        # Control longitudinal visual
        self.vx_forward_max = rospy.get_param("~vx_forward_max", 0.04)
        self.vx_reverse_max = rospy.get_param("~vx_reverse_max", 0.02)
        self.vx_forward_min = rospy.get_param("~vx_forward_min", 0.008)
        self.vx_reverse_min = rospy.get_param("~vx_reverse_min", 0.006)

        self.k_far = rospy.get_param("~k_far", 0.015)
        self.k_mid = rospy.get_param("~k_mid", 0.004)
        self.k_near = rospy.get_param("~k_near", 0.002)
        self.err_far = rospy.get_param("~err_far", 10.0)
        self.err_mid = rospy.get_param("~err_mid", 4.0)

        self.k_rate_forward = rospy.get_param("~k_rate_forward", 0.18)
        self.k_rate_reverse = rospy.get_param("~k_rate_reverse", 0.10)

        self.kd_forward = rospy.get_param("~kd_forward", 0.006)
        self.kd_reverse = rospy.get_param("~kd_reverse", 0.004)

        # Control transversal / altura / yaw
        self.kp_y = rospy.get_param("~kp_y", 0.6)
        self.kp_z = rospy.get_param("~kp_z", 0.9)
        self.kp_yaw = rospy.get_param("~kp_yaw", 1.0)

        self.vy_max = rospy.get_param("~vy_max", 0.02)
        self.vz_max = rospy.get_param("~vz_max", 0.04)
        self.wz_max = rospy.get_param("~wz_max", 0.20)

        # HOLD
        self.kp_hold_x = rospy.get_param("~kp_hold_x", 0.9)
        self.kp_hold_y = rospy.get_param("~kp_hold_y", 0.9)
        self.kp_hold_z = rospy.get_param("~kp_hold_z", 0.9)
        self.kp_hold_yaw = rospy.get_param("~kp_hold_yaw", 1.0)

        self.vx_hold_max = rospy.get_param("~vx_hold_max", 0.025)
        self.vy_hold_max = rospy.get_param("~vy_hold_max", 0.025)
        self.vz_hold_max = rospy.get_param("~vz_hold_max", 0.045)
        self.wz_hold_max = rospy.get_param("~wz_hold_max", 0.18)

        # Seguridad mínima
        self.max_move_time = rospy.get_param("~max_move_time", 35.0)
        self.no_detection_timeout = rospy.get_param("~no_detection_timeout", 1.5)

        # Estado pose
        self.current_x = None
        self.current_y = None
        self.current_z = None
        self.current_yaw = None

        self.x_ref = None
        self.y_ref = None
        self.z_ref = None
        self.yaw_ref = None

        # Porcentajes de tubos detectados
        self.tube_percents_raw = []
        self.tube_percents_filt = []
        self.last_average_msg_time = None

        # Bounding boxes / centros detectados
        self.num_tubes_bbox = 0
        self.tube_bboxes = []
        self.tube_centers = []
        self.last_bboxes_msg_time = None

        # Historial de la señal seleccionada para derivada
        self.selected_percent_history = deque(maxlen=8)
        self.alpha_percent = rospy.get_param("~alpha_percent", 0.25)

        # Safety con teclado
        self.emergency_land = False
        self.keyboard_thread = None
        self.term_settings = None

        # Estado de handoff
        self.solution_sent = False

        # ROS
        self.takeoff_pub = rospy.Publisher(self.takeoff_topic, Empty, queue_size=1)
        self.land_pub = rospy.Publisher(self.land_topic, Empty, queue_size=1)
        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=1)
        self.solution_pub = rospy.Publisher(self.solution_topic, String, queue_size=1, latch=True)
        self.goal_pose_pub = rospy.Publisher(self.goal_pose_topic, Float32, queue_size=1, latch=True)
        self.numero_tubos_pub = rospy.Publisher(self.numero_tubos_topic, Int32, queue_size=1, latch=True)
        self.mission_case_pub = rospy.Publisher(self.mission_case_topic, Int32, queue_size=1, latch=True)

        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=1)
        rospy.Subscriber(self.average_topic, Float32MultiArray, self.average_callback, queue_size=1)
        rospy.Subscriber(self.bboxes_topic, Int32MultiArray, self.bboxes_callback, queue_size=1)

        self.rate = rospy.Rate(self.ctrl_rate)

    # -------------------------
    # Safety keyboard
    # -------------------------
    def keyboard_listener(self):
        while not rospy.is_shutdown() and not self.emergency_land:
            if select.select([sys.stdin], [], [], 0.05)[0]:
                key = sys.stdin.read(1)
                if key.lower() == 'q':
                    rospy.logwarn("SAFETY: tecla 'q' detectada. Aterrizaje inmediato.")
                    self.emergency_land = True
                    break

    def start_keyboard_listener(self):
        if not sys.stdin.isatty():
            rospy.logwarn("stdin no es TTY; safety con tecla 'q' no disponible en esta sesión.")
            return

        try:
            self.term_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
            self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
            self.keyboard_thread.daemon = True
            self.keyboard_thread.start()
            rospy.loginfo("Safety activo: presiona 'q' para aterrizar inmediatamente.")
        except Exception as e:
            rospy.logwarn("No se pudo activar listener de teclado: %s", str(e))

    def restore_terminal(self):
        if self.term_settings is not None and sys.stdin.isatty():
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.term_settings)
            except Exception:
                pass

    def check_emergency_land(self):
        if self.emergency_land:
            self.publish_cmd_raw(0.0, 0.0, 0.0, 0.0)
            rospy.sleep(0.1)
            self.land_pub.publish(Empty())
            rospy.sleep(1.0)
            return True
        return False

    # -------------------------
    # Callbacks
    # -------------------------
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_z = msg.pose.pose.position.z

        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_yaw = yaw

    def average_callback(self, msg):
        now = rospy.Time.now().to_sec()

        raw = [float(v) for v in msg.data]
        self.tube_percents_raw = raw

        if len(raw) == 0:
            self.tube_percents_filt = []
            self.last_average_msg_time = now
            return

        if len(self.tube_percents_filt) != len(raw):
            self.tube_percents_filt = raw[:]
        else:
            new_filt = []
            for old_v, new_v in zip(self.tube_percents_filt, raw):
                filt_v = self.alpha_percent * new_v + (1.0 - self.alpha_percent) * old_v
                new_filt.append(filt_v)
            self.tube_percents_filt = new_filt

        selected = self.select_relevant_percent()
        if selected is not None:
            self.selected_percent_history.append((now, selected))

        self.last_average_msg_time = now

    def bboxes_callback(self, msg):
        now = rospy.Time.now().to_sec()
        data = list(msg.data)

        self.num_tubes_bbox = 0
        self.tube_bboxes = []
        self.tube_centers = []

        if len(data) < 1:
            self.numero_tubos_pub.publish(Int32(data=0))
            self.last_bboxes_msg_time = now
            return

        self.num_tubes_bbox = int(data[0])
        self.numero_tubos_pub.publish(Int32(data=self.num_tubes_bbox))

        for i in range(min(self.num_tubes_bbox, 3)):
            base = 1 + i * 12
            if len(data) >= base + 4:
                x1 = int(data[base + 0])
                y1 = int(data[base + 1])
                x2 = int(data[base + 2])
                y2 = int(data[base + 3])

                if x1 >= 0 and y1 >= 0 and x2 >= 0 and y2 >= 0:
                    self.tube_bboxes.append((x1, y1, x2, y2))
                    cx = (x1 + x2) // 2
                    cy = (y1 + y2) // 2
                    self.tube_centers.append((cx, cy))

        self.last_bboxes_msg_time = now

    # -------------------------
    # Helpers
    # -------------------------
    def has_pose(self):
        return (
            self.current_x is not None and
            self.current_y is not None and
            self.current_z is not None and
            self.current_yaw is not None
        )

    def wait_for_pose(self, timeout=10.0):
        start = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            if self.check_emergency_land():
                return False
            if self.has_pose():
                return True
            if rospy.Time.now().to_sec() - start > timeout:
                return False
            self.rate.sleep()
        return False

    def normalize_angle(self, a):
        return math.atan2(math.sin(a), math.cos(a))

    def saturate(self, val, limit):
        return max(min(val, limit), -limit)

    def sat_tanh(self, k, err, vmax):
        return vmax * math.tanh(k * err)

    def publish_cmd_raw(self, vx=0.0, vy=0.0, vz=0.0, wz=0.0):
        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.linear.z = vz
        cmd.angular.z = wz
        self.cmd_pub.publish(cmd)

    def publish_cmd(self, vx=0.0, vy=0.0, vz=0.0, wz=0.0):
        # Solo bloquear cmd_vel después de enviar solución
        if self.solution_sent:
            return
        self.publish_cmd_raw(vx, vy, vz, wz)

    def detect_case(self):
        n = len(self.tube_percents_filt)
        if n <= 0:
            return 0
        if n == 1:
            return 1
        if n == 2:
            return 2
        return 3

    def any_tube_in_range(self):
        for p in self.tube_percents_filt:
            if self.percent_low <= p <= self.percent_high:
                return True, p
        return False, None

    def select_relevant_percent(self):
        if len(self.tube_percents_filt) == 0:
            return None
        return min(self.tube_percents_filt, key=lambda p: abs(p - self.target_percent))

    def estimate_selected_percent_rate(self):
        if len(self.selected_percent_history) < 2:
            return 0.0
        t0, p0 = self.selected_percent_history[0]
        t1, p1 = self.selected_percent_history[-1]
        dt = max(1e-3, t1 - t0)
        return (p1 - p0) / dt

    def compute_visual_vx_pd(self, percent_now, percent_rate):
        if self.percent_low <= percent_now <= self.percent_high:
            return 0.0, 0.0, "IN_RANGE"

        if percent_now < self.percent_low:
            error = self.target_percent - percent_now

            if error >= self.err_far:
                vx_p = self.k_far * math.sqrt(error)
            elif error >= self.err_mid:
                vx_p = self.k_mid * error
            else:
                vx_p = self.k_near * error

            vx_d = -self.kd_forward * max(0.0, percent_rate)
            vx = vx_p + vx_d

            if percent_rate > 0.0:
                vx = vx / (1.0 + self.k_rate_forward * percent_rate)

            vx = max(0.0, min(vx, self.vx_forward_max))
            if vx > 0.0:
                vx = max(vx, self.vx_forward_min)

            return vx, error, "FORWARD"

        error = percent_now - self.target_percent

        if error >= self.err_far:
            vx_p = self.k_far * math.sqrt(error)
        elif error >= self.err_mid:
            vx_p = self.k_mid * error
        else:
            vx_p = self.k_near * error

        vx_d = -self.kd_reverse * max(0.0, -percent_rate)
        vx_mag = vx_p + vx_d

        if percent_rate < 0.0:
            vx_mag = vx_mag / (1.0 + self.k_rate_reverse * abs(percent_rate))

        vx_mag = max(0.0, min(vx_mag, self.vx_reverse_max))
        if vx_mag > 0.0:
            vx_mag = max(vx_mag, self.vx_reverse_min)

        return -vx_mag, -error, "REVERSE"

    def compute_hold_cmd(self, x_ref, y_ref, z_ref, yaw_ref):
        ex = x_ref - self.current_x
        ey = y_ref - self.current_y
        ez = z_ref - self.current_z
        eyaw = self.normalize_angle(yaw_ref - self.current_yaw)

        vx = self.sat_tanh(self.kp_hold_x, ex, self.vx_hold_max)
        vy = self.sat_tanh(self.kp_hold_y, ey, self.vy_hold_max)
        vz = self.sat_tanh(self.kp_hold_z, ez, self.vz_hold_max)
        wz = self.saturate(self.kp_hold_yaw * eyaw, self.wz_hold_max)

        return vx, vy, vz, wz

    def do_hold_then_land(self, hold_time):
        hold_x = self.current_x if self.current_x is not None else self.x_ref
        hold_y = self.current_y if self.current_y is not None else self.y_ref
        hold_z = self.current_z if self.current_z is not None else self.z_ref
        hold_yaw = self.current_yaw if self.current_yaw is not None else self.yaw_ref

        rospy.loginfo("HOLD %.1f s antes de aterrizar...", hold_time)
        hold_start = rospy.Time.now().to_sec()

        while not rospy.is_shutdown():
            if self.check_emergency_land():
                return

            if rospy.Time.now().to_sec() - hold_start >= hold_time:
                break

            if self.has_pose():
                vx, vy, vz, wz = self.compute_hold_cmd(
                    hold_x, hold_y, hold_z, hold_yaw
                )
                self.publish_cmd_raw(vx, vy, vz, wz)
            else:
                self.publish_cmd_raw(0.0, 0.0, 0.0, 0.0)

            self.rate.sleep()

        rospy.loginfo("Aterrizando...")
        self.land_pub.publish(Empty())
        rospy.sleep(2.0)

    def do_hold_then_publish_case2(self, hold_time, solution):
        hold_x = self.current_x if self.current_x is not None else self.x_ref
        hold_y = self.current_y if self.current_y is not None else self.y_ref
        hold_z = self.current_z if self.current_z is not None else self.z_ref
        hold_yaw = self.current_yaw if self.current_yaw is not None else self.yaw_ref

        rospy.loginfo("HOLD %.1f s antes de publicar solucion CASE 2...", hold_time)
        hold_start = rospy.Time.now().to_sec()

        while not rospy.is_shutdown():
            if self.check_emergency_land():
                return

            if rospy.Time.now().to_sec() - hold_start >= hold_time:
                break

            if self.has_pose():
                vx, vy, vz, wz = self.compute_hold_cmd(
                    hold_x, hold_y, hold_z, hold_yaw
                )
                self.publish_cmd_raw(vx, vy, vz, wz)
            else:
                self.publish_cmd_raw(0.0, 0.0, 0.0, 0.0)

            self.rate.sleep()

        self.publish_case2_solution_and_stop_control(solution)

    def choose_case1_solution(self):
        if len(self.tube_centers) < 1:
            rospy.logwarn("CASE 1: NO HAY CENTRO DE TUBO DISPONIBLE; POR DEFECTO AVANZAR_DERECHA")
            return "AVANZAR_DERECHA"

        cx_tubo = self.tube_centers[0][0]

        if cx_tubo < self.center_frame_x:
            rospy.loginfo("SOLUCION ENCONTRADA AVANZAR_DERECHA (TUBO IZQUIERDA)")
            return "AVANZAR_DERECHA"
        elif cx_tubo > self.center_frame_x:
            rospy.loginfo("SOLUCION ENCONTRADA AVANZAR_IZQUIERDA (TUBO DERECHA)")
            return "AVANZAR_IZQUIERDA"
        else:
            rospy.loginfo("SOLUCION ENCONTRADA AVANZAR_DERECHA (TUBO CENTRADO)")
            return "AVANZAR_DERECHA"

    def choose_case2_solution(self):
        if len(self.tube_centers) < 2:
            rospy.logwarn("CASE 2: NO HAY DOS CENTROS DE TUBO DISPONIBLES; POR DEFECTO CENTRADO")
            return "CENTRADO"

        cx1 = self.tube_centers[0][0]
        cx2 = self.tube_centers[1][0]

        diff = abs(cx2 - cx1)

        if diff >= 420:
            rospy.loginfo("SOLUCIÓN ENCONTRADA CENTRADO")
            return "CENTRADO"

        if cx1 < self.center_frame_x and cx2 < self.center_frame_x:
            rospy.loginfo("SOLUCIÓN ENCONTRADA DERECHA")
            return "DERECHA"

        if cx1 > self.center_frame_x and cx2 > self.center_frame_x:
            rospy.loginfo("SOLUCIÓN ENCONTRADA IZQUIERDA")
            return "IZQUIERDA"

        left_cx = cx1 if cx1 < self.center_frame_x else cx2
        right_cx = cx1 if cx1 > self.center_frame_x else cx2

        d_left = abs(left_cx - self.center_frame_x)
        d_right = abs(right_cx - self.center_frame_x)

        if d_right < d_left:
            rospy.loginfo("SOLUCIÓN ENCONTRADA DERECHA")
            return "DERECHA"
        else:
            rospy.loginfo("SOLUCIÓN ENCONTRADA IZQUIERDA")
            return "IZQUIERDA"

    def publish_solution_and_stop_control(self, solution):
        rospy.loginfo("PUBLICANDO SOLUCION EN %s: %s", self.solution_topic, solution)
        self.solution_pub.publish(String(data=solution))

        # mandar ceros una vez y luego dejar de publicar cmd_vel
        self.publish_cmd_raw(0.0, 0.0, 0.0, 0.0)
        rospy.sleep(0.1)

        self.solution_sent = True
        rospy.loginfo("CONTROL CEDIDO. ESTE NODO YA NO PUBLICA CMD_VEL.")

    def publish_case2_solution_and_stop_control(self, solution):
        rospy.loginfo("PUBLICANDO MISSION_CASE EN %s: %d", self.mission_case_topic, 2)
        self.mission_case_pub.publish(Int32(data=2))

        rospy.loginfo("PUBLICANDO SOLUCION EN %s: %s", self.solution_topic, solution)
        self.solution_pub.publish(String(data=solution))

        # mandar ceros una vez y luego dejar de publicar cmd_vel
        self.publish_cmd_raw(0.0, 0.0, 0.0, 0.0)
        rospy.sleep(0.1)

        self.solution_sent = True
        rospy.loginfo("CONTROL CEDIDO. ESTE NODO YA NO PUBLICA CMD_VEL.")

    def run_case_0(self, traveled, ey, ez, eyaw):
        if traveled >= self.max_forward_distance:
            rospy.loginfo("CASE 0: pose final alcanzada (%.3f m)", traveled)
            self.publish_cmd_raw(0.0, 0.0, 0.0, 0.0)
            return True

        vx = min(self.vx_forward_max * 0.70, 0.040)
        vy = self.sat_tanh(self.kp_y, ey, self.vy_max)
        vz = self.sat_tanh(self.kp_z, ez, self.vz_max)
        wz = self.saturate(self.kp_yaw * eyaw, self.wz_max)

        self.publish_cmd(vx=vx, vy=vy, vz=vz, wz=wz)

        rospy.loginfo_throttle(
            0.4,
            "CASE 0 | traveled=%.2f / %.2f | cmd=(%.3f, %.3f, %.3f, %.3f)",
            traveled, self.max_forward_distance, vx, vy, vz, wz
        )
        return False

    def run_case_1(self, ey, ez, eyaw):
        if self.last_average_msg_time is None or \
           (rospy.Time.now().to_sec() - self.last_average_msg_time > self.no_detection_timeout):
            vx = min(self.vx_forward_max * 0.55, 0.030)
            vy = self.sat_tanh(self.kp_y, ey, self.vy_max)
            vz = self.sat_tanh(self.kp_z, ez, self.vz_max)
            wz = self.saturate(self.kp_yaw * eyaw, self.wz_max)

            self.publish_cmd(vx=vx, vy=vy, vz=vz, wz=wz)
            rospy.loginfo_throttle(
                0.4,
                "CASE 1 | sin dato visual reciente | cmd=(%.3f, %.3f, %.3f, %.3f)",
                vx, vy, vz, wz
            )
            return False

        in_range, tube_percent_in_range = self.any_tube_in_range()
        if in_range:
            solution = self.choose_case1_solution()
            self.publish_solution_and_stop_control(solution)
            return True

        selected_percent = self.select_relevant_percent()
        if selected_percent is None:
            vx = min(self.vx_forward_max * 0.55, 0.030)
            vy = self.sat_tanh(self.kp_y, ey, self.vy_max)
            vz = self.sat_tanh(self.kp_z, ez, self.vz_max)
            wz = self.saturate(self.kp_yaw * eyaw, self.wz_max)

            self.publish_cmd(vx=vx, vy=vy, vz=vz, wz=wz)
            return False

        percent_rate = self.estimate_selected_percent_rate()
        vx, _, mode = self.compute_visual_vx_pd(selected_percent, percent_rate)

        vy = self.sat_tanh(self.kp_y, ey, self.vy_max)
        vz = self.sat_tanh(self.kp_z, ez, self.vz_max)
        wz = self.saturate(self.kp_yaw * eyaw, self.wz_max)

        self.publish_cmd(vx=vx, vy=vy, vz=vz, wz=wz)

        rospy.loginfo_throttle(
            0.35,
            "CASE 1 | tubes=%d | raw=%s | filt=%s | selected=%.2f rate=%.2f | mode=%s | cmd=(%.3f, %.3f, %.3f, %.3f)",
            len(self.tube_percents_filt),
            str([round(v, 2) for v in self.tube_percents_raw]),
            str([round(v, 2) for v in self.tube_percents_filt]),
            selected_percent,
            percent_rate,
            mode,
            vx, vy, vz, wz
        )
        return False

    def run_case_2(self, ey, ez, eyaw):
        if self.last_average_msg_time is None or \
           (rospy.Time.now().to_sec() - self.last_average_msg_time > self.no_detection_timeout):
            vx = min(self.vx_forward_max * 0.55, 0.030)
            vy = self.sat_tanh(self.kp_y, ey, self.vy_max)
            vz = self.sat_tanh(self.kp_z, ez, self.vz_max)
            wz = self.saturate(self.kp_yaw * eyaw, self.wz_max)

            self.publish_cmd(vx=vx, vy=vy, vz=vz, wz=wz)
            rospy.loginfo_throttle(
                0.4,
                "CASE 2 | sin dato visual reciente | cmd=(%.3f, %.3f, %.3f, %.3f)",
                vx, vy, vz, wz
            )
            return False

        in_range, tube_percent_in_range = self.any_tube_in_range()
        if in_range:
            solution = self.choose_case2_solution()
            self.do_hold_then_publish_case2(self.final_hold_time, solution)
            return True

        selected_percent = self.select_relevant_percent()
        if selected_percent is None:
            vx = min(self.vx_forward_max * 0.55, 0.030)
            vy = self.sat_tanh(self.kp_y, ey, self.vy_max)
            vz = self.sat_tanh(self.kp_z, ez, self.vz_max)
            wz = self.saturate(self.kp_yaw * eyaw, self.wz_max)

            self.publish_cmd(vx=vx, vy=vy, vz=vz, wz=wz)
            return False

        percent_rate = self.estimate_selected_percent_rate()
        vx, _, mode = self.compute_visual_vx_pd(selected_percent, percent_rate)

        vy = self.sat_tanh(self.kp_y, ey, self.vy_max)
        vz = self.sat_tanh(self.kp_z, ez, self.vz_max)
        wz = self.saturate(self.kp_yaw * eyaw, self.wz_max)

        self.publish_cmd(vx=vx, vy=vy, vz=vz, wz=wz)

        rospy.loginfo_throttle(
            0.35,
            "CASE 2 | tubes=%d | raw=%s | filt=%s | selected=%.2f rate=%.2f | mode=%s | cmd=(%.3f, %.3f, %.3f, %.3f)",
            len(self.tube_percents_filt),
            str([round(v, 2) for v in self.tube_percents_raw]),
            str([round(v, 2) for v in self.tube_percents_filt]),
            selected_percent,
            percent_rate,
            mode,
            vx, vy, vz, wz
        )
        return False

    def run_case_3(self, ey, ez, eyaw):
        if self.last_average_msg_time is None or \
           (rospy.Time.now().to_sec() - self.last_average_msg_time > self.no_detection_timeout):
            vx = min(self.vx_forward_max * 0.55, 0.030)
            vy = self.sat_tanh(self.kp_y, ey, self.vy_max)
            vz = self.sat_tanh(self.kp_z, ez, self.vz_max)
            wz = self.saturate(self.kp_yaw * eyaw, self.wz_max)

            self.publish_cmd(vx=vx, vy=vy, vz=vz, wz=wz)
            rospy.loginfo_throttle(
                0.4,
                "CASE 3 | sin dato visual reciente | cmd=(%.3f, %.3f, %.3f, %.3f)",
                vx, vy, vz, wz
            )
            return False

        in_range, tube_percent_in_range = self.any_tube_in_range()
        if in_range:
            rospy.loginfo(
                "CASE %d: algún tubo entró al rango [%.1f, %.1f]: %.2f%%",
                3, self.percent_low, self.percent_high, tube_percent_in_range
            )
            self.do_hold_then_land(self.final_hold_time)
            return True

        selected_percent = self.select_relevant_percent()
        if selected_percent is None:
            vx = min(self.vx_forward_max * 0.55, 0.030)
            vy = self.sat_tanh(self.kp_y, ey, self.vy_max)
            vz = self.sat_tanh(self.kp_z, ez, self.vz_max)
            wz = self.saturate(self.kp_yaw * eyaw, self.wz_max)

            self.publish_cmd(vx=vx, vy=vy, vz=vz, wz=wz)
            return False

        percent_rate = self.estimate_selected_percent_rate()
        vx, _, mode = self.compute_visual_vx_pd(selected_percent, percent_rate)

        vy = self.sat_tanh(self.kp_y, ey, self.vy_max)
        vz = self.sat_tanh(self.kp_z, ez, self.vz_max)
        wz = self.saturate(self.kp_yaw * eyaw, self.wz_max)

        self.publish_cmd(vx=vx, vy=vy, vz=vz, wz=wz)

        rospy.loginfo_throttle(
            0.35,
            "CASE %d | tubes=%d | raw=%s | filt=%s | selected=%.2f rate=%.2f | mode=%s | cmd=(%.3f, %.3f, %.3f, %.3f)",
            3,
            len(self.tube_percents_filt),
            str([round(v, 2) for v in self.tube_percents_raw]),
            str([round(v, 2) for v in self.tube_percents_filt]),
            selected_percent,
            percent_rate,
            mode,
            vx, vy, vz, wz
        )
        return False

    # -------------------------
    # Main
    # -------------------------
    def run(self):
        self.start_keyboard_listener()

        rospy.sleep(1.0)

        rospy.loginfo("Esperando pose...")
        if not self.wait_for_pose():
            return

        if self.check_emergency_land():
            return

        rospy.loginfo("Despegando...")
        self.takeoff_pub.publish(Empty())
        rospy.sleep(3.0)

        if self.check_emergency_land():
            return

        rospy.loginfo("Hover inicial durante %.1f s...", self.initial_hover_time)
        start_hover = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            if self.check_emergency_land():
                return
            if rospy.Time.now().to_sec() - start_hover >= self.initial_hover_time:
                break
            self.publish_cmd(0.0, 0.0, 0.0, 0.0)
            self.rate.sleep()

        if not self.has_pose():
            rospy.logerr("Se perdió la pose tras despegar. Abortando.")
            return

        self.x_ref = self.current_x
        self.y_ref = self.current_y
        self.z_ref = self.current_z
        self.yaw_ref = self.current_yaw

        rospy.loginfo(
            "Referencia capturada | x=%.3f y=%.3f z=%.3f yaw=%.2f deg",
            self.x_ref, self.y_ref, self.z_ref, math.degrees(self.yaw_ref)
        )

        rospy.loginfo("PUBLICANDO GOAL_POSE EN %s: %.3f", self.goal_pose_topic, self.max_forward_distance)
        self.goal_pose_pub.publish(Float32(data=self.max_forward_distance))

        move_start = rospy.Time.now().to_sec()

        while not rospy.is_shutdown():
            if self.check_emergency_land():
                return

            if self.solution_sent:
                rospy.loginfo("SOLUCION YA ENVIADA. NODO EN ESPERA SIN PUBLICAR CMD_VEL.")
                return

            if not self.has_pose():
                self.publish_cmd(0.0, 0.0, 0.0, 0.0)
                self.rate.sleep()
                continue

            ey = self.y_ref - self.current_y
            ez = self.z_ref - self.current_z
            eyaw = self.normalize_angle(self.yaw_ref - self.current_yaw)
            traveled = self.current_x - self.x_ref

            if rospy.Time.now().to_sec() - move_start > self.max_move_time:
                rospy.logwarn("Timeout de misión.")
                self.do_hold_then_land(1.0)
                return

            case_id = self.detect_case()

            if case_id == 0:
                finished = self.run_case_0(traveled, ey, ez, eyaw)
            elif case_id == 1:
                finished = self.run_case_1(ey, ez, eyaw)
            elif case_id == 2:
                finished = self.run_case_2(ey, ez, eyaw)
            else:
                finished = self.run_case_3(ey, ez, eyaw)

            if finished:
                return

            self.rate.sleep()


if __name__ == "__main__":
    node = None
    try:
        node = MissionPointToPointPercentAnyTube()
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        if node is not None:
            node.restore_terminal()
