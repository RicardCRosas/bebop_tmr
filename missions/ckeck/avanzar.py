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
from std_msgs.msg import String, Empty, Float32, Float32MultiArray, Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion


class MissionAdvanceNode:
    def __init__(self):
        rospy.init_node("mission_advance_node")

        self.executor_state_topic = rospy.get_param("~executor_state_topic", "/ejecutor_state")
        self.goal_pose_topic = rospy.get_param("~goal_pose_topic", "/goal_pose")
        self.solution_topic = rospy.get_param("~solution_topic", "/mission_solution")
        self.target_tube_bbox_topic = rospy.get_param("~target_tube_bbox_topic", "/target_tube_bbox")

        # Handshake misión 2
        self.mission2_syn_topic = rospy.get_param("~mission2_syn_topic", "/mission2_syn")
        self.mission2_syn_ack_topic = rospy.get_param("~mission2_syn_ack_topic", "/mission2_syn_ack")
        self.mission2_ack_topic = rospy.get_param("~mission2_ack_topic", "/mission2_ack")
        self.mission2_timeout = rospy.get_param("~mission2_timeout", 1.0)

        # Nuevo handoff a approach_mission_land
        self.approach_mission_land_topic = rospy.get_param("~approach_mission_land_topic", "/approach_mission_land")

        self.land_topic = rospy.get_param("~land_topic", "/bebop/land")
        self.odom_topic = rospy.get_param("~odom_topic", "/bebop/odom")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/bebop/cmd_vel")
        self.average_topic = rospy.get_param("~average_topic", "/average_color")
        self.bboxes_topic = rospy.get_param("~bboxes_topic", "/orange_bboxes")
        self.ctrl_rate = rospy.get_param("~ctrl_rate", 30.0)

        # Tiempos
        self.goal_hold_time = rospy.get_param("~goal_hold_time", 1.0)
        self.handoff_hold_time = rospy.get_param("~handoff_hold_time", 0.5)

        # Rango de handoff visual
        self.percent_low = rospy.get_param("~percent_low", 21.0)
        self.percent_high = rospy.get_param("~percent_high", 25.0)
        self.target_percent = rospy.get_param("~target_percent", 23.5)

        # Control hacia delante
        self.kp_x_goal = rospy.get_param("~kp_x_goal", 0.9)
        self.vx_goal_max = rospy.get_param("~vx_goal_max", 0.055)
        self.vx_goal_min = rospy.get_param("~vx_goal_min", 0.015)
        self.goal_tolerance = rospy.get_param("~goal_tolerance", 0.08)

        # Visual PD en X
        self.vx_forward_max = rospy.get_param("~vx_forward_max", 0.03)
        self.vx_reverse_max = rospy.get_param("~vx_reverse_max", 0.02)
        self.vx_forward_min = rospy.get_param("~vx_forward_min", 0.01)
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

        # Mantener pose mientras avanza en x
        self.kp_hold_y = rospy.get_param("~kp_hold_y", 1.0)
        self.kp_hold_z = rospy.get_param("~kp_hold_z", 0.9)
        self.kp_hold_yaw = rospy.get_param("~kp_hold_yaw", 1.0)
        self.vy_hold_max = rospy.get_param("~vy_hold_max", 0.03)
        self.vz_hold_max = rospy.get_param("~vz_hold_max", 0.05)
        self.wz_hold_max = rospy.get_param("~wz_hold_max", 0.18)

        self.no_detection_timeout = rospy.get_param("~no_detection_timeout", 1.5)
        self.alpha_percent = rospy.get_param("~alpha_percent", 0.25)
        self.frame_w = rospy.get_param("~frame_w", 680)
        self.center_frame_x = self.frame_w // 2

        self.land_pub = rospy.Publisher(self.land_topic, Empty, queue_size=1)
        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=1)
        self.solution_pub = rospy.Publisher(self.solution_topic, String, queue_size=1, latch=True)
        self.target_tube_bbox_pub = rospy.Publisher(
            self.target_tube_bbox_topic, Int32MultiArray, queue_size=1, latch=True
        )
        self.mission2_syn_pub = rospy.Publisher(self.mission2_syn_topic, String, queue_size=1)
        self.mission2_ack_pub = rospy.Publisher(self.mission2_ack_topic, String, queue_size=1)
        self.approach_mission_land_pub = rospy.Publisher(self.approach_mission_land_topic, String, queue_size=1)

        rospy.Subscriber(self.executor_state_topic, String, self.executor_state_callback, queue_size=1)
        rospy.Subscriber(self.goal_pose_topic, Float32, self.goal_callback, queue_size=1)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=1)
        rospy.Subscriber(self.average_topic, Float32MultiArray, self.average_callback, queue_size=1)
        rospy.Subscriber(self.bboxes_topic, Int32MultiArray, self.bboxes_callback, queue_size=1)
        rospy.Subscriber(self.mission2_syn_ack_topic, String, self.mission2_syn_ack_callback, queue_size=1)

        self.executing = False
        self.emergency_land = False
        self.keyboard_thread = None
        self.term_settings = None

        self.current_x = None
        self.current_y = None
        self.current_z = None
        self.current_yaw = None

        self.goal_x = None
        self.activation_received = False

        self.tube_percents_raw = []
        self.tube_percents_filt = []
        self.last_average_msg_time = None

        self.tube_bboxes = []
        self.tube_centers = []
        self.last_bboxes_msg_time = None

        self.selected_percent_history = deque(maxlen=8)
        self.mission2_syn_ack_received = False
        self.rate = rospy.Rate(self.ctrl_rate)

        rospy.loginfo("Nodo avanzar.py esperando COMPLETADO en %s", self.executor_state_topic)

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
            self.publish_zero_cmd_burst()
            rospy.sleep(0.1)
            rospy.logwarn("Aterrizando por emergencia...")
            self.land_pub.publish(Empty())
            rospy.sleep(1.0)
            return True
        return False

    # -------------------------
    # Callbacks
    # -------------------------
    def executor_state_callback(self, msg):
        state = msg.data.strip().upper()
        if state == "COMPLETADO" and not self.executing:
            self.activation_received = True
            rospy.loginfo("avanzar.py ACTIVADO por COMPLETADO")

    def goal_callback(self, msg):
        self.goal_x = float(msg.data)

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
        self.tube_bboxes = []
        self.tube_centers = []
        usable = len(data) - (len(data) % 4)
        for i in range(0, usable, 4):
            x1 = int(data[i + 0])
            y1 = int(data[i + 1])
            x2 = int(data[i + 2])
            y2 = int(data[i + 3])
            if x1 >= 0 and y1 >= 0 and x2 > x1 and y2 > y1:
                self.tube_bboxes.append((x1, y1, x2, y2))
                self.tube_centers.append(((x1 + x2) // 2, (y1 + y2) // 2))
        self.last_bboxes_msg_time = now

    def mission2_syn_ack_callback(self, msg):
        data = msg.data.strip().upper()
        if data == "LISTO":
            self.mission2_syn_ack_received = True
            rospy.loginfo("MISSION2_SYN_ACK recibido: LISTO")

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

    def has_goal(self):
        return self.goal_x is not None

    def normalize_angle(self, a):
        return math.atan2(math.sin(a), math.cos(a))

    def saturate(self, val, limit):
        return max(min(val, limit), -limit)

    def sat_tanh(self, k, err, vmax):
        return vmax * math.tanh(k * err)

    def publish_cmd(self, vx=0.0, vy=0.0, vz=0.0, wz=0.0):
        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.linear.z = vz
        cmd.angular.z = wz
        self.cmd_pub.publish(cmd)

    def publish_zero_cmd_burst(self, repeats=6, sleep_dt=0.03):
        for _ in range(repeats):
            self.publish_cmd(0.0, 0.0, 0.0, 0.0)
            rospy.sleep(sleep_dt)

    def select_relevant_percent(self):
        if len(self.tube_percents_filt) == 0:
            return None
        return self.tube_percents_filt[0]

    def tube_in_range(self):
        if len(self.tube_percents_filt) == 0:
            return False, None
        p = self.tube_percents_filt[0]
        if self.percent_low <= p <= self.percent_high:
            return True, p
        return False, p

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

    def compute_forward_only_vx(self, remaining_distance):
        if remaining_distance <= 0.0:
            return 0.0
        vx = self.sat_tanh(self.kp_x_goal, remaining_distance, self.vx_goal_max)
        if vx > 0.0:
            vx = max(vx, self.vx_goal_min)
        return min(vx, self.vx_goal_max)

    def compute_hold_cmd(self, x_ref, y_ref, z_ref, yaw_ref):
        ex = x_ref - self.current_x
        ey = y_ref - self.current_y
        ez = z_ref - self.current_z
        eyaw = self.normalize_angle(yaw_ref - self.current_yaw)
        vx = self.sat_tanh(self.kp_x_goal, ex, self.vx_goal_max)
        vy = self.sat_tanh(self.kp_hold_y, ey, self.vy_hold_max)
        vz = self.sat_tanh(self.kp_hold_z, ez, self.vz_hold_max)
        wz = self.saturate(self.kp_hold_yaw * eyaw, self.wz_hold_max)
        return vx, vy, vz, wz

    def choose_solution_from_center(self):
        if len(self.tube_centers) == 0:
            rospy.logwarn("No hay centro de tubo disponible; por defecto DERECHA")
            return "DERECHA"
        cx_tubo = self.tube_centers[0][0]
        if 0 <= cx_tubo <= 339:
            rospy.loginfo("SOLUCION ENCONTRADA: DERECHA")
            return "DERECHA"
        rospy.loginfo("SOLUCION ENCONTRADA: IZQUIERDA")
        return "IZQUIERDA"

    def publish_solution_and_release_control(self, solution):
        rospy.loginfo("PUBLICANDO SOLUCION EN %s: %s", self.solution_topic, solution)
        self.solution_pub.publish(String(data=solution))
        if len(self.tube_bboxes) > 0:
            bbox = self.tube_bboxes[0]
            self.target_tube_bbox_pub.publish(Int32MultiArray(data=list(bbox)))
            rospy.loginfo("PUBLICANDO TARGET EN %s: %s", self.target_tube_bbox_topic, str(list(bbox)))
        else:
            self.target_tube_bbox_pub.publish(Int32MultiArray(data=[]))
            rospy.logwarn("No había bbox válido para publicar target_tube_bbox")
        self.publish_zero_cmd_burst()
        rospy.sleep(0.1)

    def do_hold_then_land(self, hold_time, x_ref, y_ref, z_ref, yaw_ref):
        rospy.loginfo("HOLD %.1f s antes de aterrizar...", hold_time)
        hold_start = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            if self.check_emergency_land():
                return
            if rospy.Time.now().to_sec() - hold_start >= hold_time:
                break
            if self.has_pose():
                vx, vy, vz, wz = self.compute_hold_cmd(x_ref, y_ref, z_ref, yaw_ref)
                self.publish_cmd(vx=vx, vy=vy, vz=vz, wz=wz)
            else:
                self.publish_cmd(0.0, 0.0, 0.0, 0.0)
            self.rate.sleep()
        self.publish_zero_cmd_burst()
        rospy.sleep(0.1)
        rospy.loginfo("Aterrizando...")
        self.land_pub.publish(Empty())

    def do_hold_then_handoff(self, hold_time):
        if not self.has_pose():
            self.publish_zero_cmd_burst()
            rospy.sleep(0.1)
            solution = self.choose_solution_from_center()
            self.publish_solution_and_release_control(solution)
            return
        x_ref = self.current_x
        y_ref = self.current_y
        z_ref = self.current_z
        yaw_ref = self.current_yaw
        rospy.loginfo("HOLD %.1f s antes de publicar solución...", hold_time)
        hold_start = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            if self.check_emergency_land():
                return
            if rospy.Time.now().to_sec() - hold_start >= hold_time:
                break
            if self.has_pose():
                vx, vy, vz, wz = self.compute_hold_cmd(x_ref, y_ref, z_ref, yaw_ref)
                self.publish_cmd(vx=vx, vy=vy, vz=vz, wz=wz)
            else:
                self.publish_cmd(0.0, 0.0, 0.0, 0.0)
            self.rate.sleep()
        solution = self.choose_solution_from_center()
        self.publish_solution_and_release_control(solution)

    def hold_position_no_land(self, hold_time, x_ref, y_ref, z_ref, yaw_ref, label="HOLD"):
        rospy.loginfo("%s %.1f s...", label, hold_time)
        hold_start = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            if self.check_emergency_land():
                return
            if rospy.Time.now().to_sec() - hold_start >= hold_time:
                break
            if self.has_pose():
                vx, vy, vz, wz = self.compute_hold_cmd(x_ref, y_ref, z_ref, yaw_ref)
                self.publish_cmd(vx=vx, vy=vy, vz=vz, wz=wz)
            else:
                self.publish_cmd(0.0, 0.0, 0.0, 0.0)
            self.rate.sleep()
        self.publish_zero_cmd_burst()
        rospy.sleep(0.1)

    def rotate_90_deg_clockwise_then_handoff_approach(self):
        if not self.has_pose():
            self.publish_zero_cmd_burst()
            rospy.sleep(0.1)
            rospy.logwarn("Sin pose para rotar. Publicando MOVE a approach_mission_land.")
            self.approach_mission_land_pub.publish(String(data="MOVE"))
            return

        yaw_ref = self.current_yaw
        target_yaw = self.normalize_angle(yaw_ref - math.pi / 2.0)

        rospy.loginfo("Misión 2 no disponible. Rotando 90 grados horario...")
        start_time = rospy.Time.now().to_sec()

        while not rospy.is_shutdown():
            if self.check_emergency_land():
                return

            if not self.has_pose():
                self.publish_cmd(0.0, 0.0, 0.0, 0.0)
                self.rate.sleep()
                continue

            eyaw = self.normalize_angle(target_yaw - self.current_yaw)
            if abs(eyaw) <= math.radians(3.0):
                break

            wz = self.saturate(1.0 * eyaw, self.wz_hold_max)
            self.publish_cmd(0.0, 0.0, 0.0, wz)
            self.rate.sleep()

        self.publish_zero_cmd_burst()
        rospy.sleep(0.1)

        if self.has_pose():
            x_hold = self.current_x
            y_hold = self.current_y
            z_hold = self.current_z
            yaw_hold = self.current_yaw
            self.hold_position_no_land(
                self.goal_hold_time, x_hold, y_hold, z_hold, yaw_hold,
                label="HOLD después de rotar antes de ceder a approach_mission_land"
            )

        # Importante: liberar cmd_vel antes de avisar al siguiente nodo
        self.publish_zero_cmd_burst()
        rospy.sleep(0.1)

        rospy.loginfo("PUBLICANDO MOVE EN %s", self.approach_mission_land_topic)
        self.approach_mission_land_pub.publish(String(data="MOVE"))

    def do_hold_then_check_mission2(self, hold_time, x_ref, y_ref, z_ref, yaw_ref):
        rospy.loginfo("HOLD %.1f s antes de verificar misión 2...", hold_time)
        hold_start = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            if self.check_emergency_land():
                return True
            if rospy.Time.now().to_sec() - hold_start >= hold_time:
                break
            if self.has_pose():
                vx, vy, vz, wz = self.compute_hold_cmd(x_ref, y_ref, z_ref, yaw_ref)
                self.publish_cmd(vx=vx, vy=vy, vz=vz, wz=wz)
            else:
                self.publish_cmd(0.0, 0.0, 0.0, 0.0)
            self.rate.sleep()

        self.mission2_syn_ack_received = False
        rospy.loginfo("PUBLICANDO SYN EN %s: LISTO", self.mission2_syn_topic)
        self.mission2_syn_pub.publish(String(data="LISTO"))

        wait_start = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            if self.check_emergency_land():
                return True

            if self.has_pose():
                vx, vy, vz, wz = self.compute_hold_cmd(x_ref, y_ref, z_ref, yaw_ref)
                self.publish_cmd(vx=vx, vy=vy, vz=vz, wz=wz)
            else:
                self.publish_cmd(0.0, 0.0, 0.0, 0.0)

            if self.mission2_syn_ack_received:
                rospy.loginfo("MISSION2_SYN_ACK recibido dentro del timeout.")
                # liberar cmd_vel antes del ACK final
                self.publish_zero_cmd_burst()
                rospy.sleep(0.1)
                rospy.loginfo("PUBLICANDO ACK EN %s: LISTO", self.mission2_ack_topic)
                self.mission2_ack_pub.publish(String(data="LISTO"))
                rospy.loginfo("Control liberado para misión 2. avanzar.py termina.")
                return True

            if rospy.Time.now().to_sec() - wait_start >= self.mission2_timeout:
                rospy.logwarn("Timeout esperando respuesta de misión 2.")
                break

            self.rate.sleep()

        self.publish_zero_cmd_burst()
        rospy.sleep(0.1)
        self.rotate_90_deg_clockwise_then_handoff_approach()
        return True

    def execute_forward_to_goal(self):
        if not self.has_pose():
            rospy.logwarn("No hay odometría suficiente para avanzar al goal.")
            self.publish_zero_cmd_burst()
            return False
        if not self.has_goal():
            rospy.logwarn("No hay dato de /goal_pose.")
            self.publish_zero_cmd_burst()
            return False

        remaining = self.goal_x - self.current_x
        if remaining < 0.0:
            rospy.logwarn(
                "Escenario inválido: x_actual=%.3f ya superó goal_pose=%.3f. No se avanza.",
                self.current_x, self.goal_x
            )
            return False

        y_ref = self.current_y
        z_ref = self.current_z
        yaw_ref = self.current_yaw

        rospy.loginfo("GOAL_POSE recibido: %.3f", self.goal_x)
        rospy.loginfo("DISTANCIA RESTANTE INICIAL: %.3f", remaining)

        while not rospy.is_shutdown():
            if self.check_emergency_land():
                return False

            if not self.has_pose():
                self.publish_cmd(0.0, 0.0, 0.0, 0.0)
                self.rate.sleep()
                continue

            remaining = self.goal_x - self.current_x
            ey = y_ref - self.current_y
            ez = z_ref - self.current_z
            eyaw = self.normalize_angle(yaw_ref - self.current_yaw)

            if remaining < 0.0:
                rospy.logwarn(
                    "x_actual=%.3f superó goal_pose=%.3f. No se retrocede.",
                    self.current_x, self.goal_x
                )
                self.publish_zero_cmd_burst()
                return False

            if remaining <= self.goal_tolerance:
                rospy.loginfo(
                    "GOAL ALCANZADO -> goal_pose=%.3f | x_actual=%.3f | restante=%.3f",
                    self.goal_x, self.current_x, remaining
                )
                x_hold = self.current_x
                y_hold = self.current_y
                z_hold = self.current_z
                yaw_hold = self.current_yaw
                mission_finished = self.do_hold_then_check_mission2(
                    self.goal_hold_time, x_hold, y_hold, z_hold, yaw_hold
                )
                return mission_finished

            no_tube_recent = (
                self.last_average_msg_time is None or
                (rospy.Time.now().to_sec() - self.last_average_msg_time > self.no_detection_timeout) or
                len(self.tube_percents_filt) == 0
            )

            if no_tube_recent:
                vx = self.compute_forward_only_vx(remaining)
                vy = self.sat_tanh(self.kp_hold_y, ey, self.vy_hold_max)
                vz = self.sat_tanh(self.kp_hold_z, ez, self.vz_hold_max)
                wz = self.saturate(self.kp_hold_yaw * eyaw, self.wz_hold_max)

                self.publish_cmd(vx=vx, vy=vy, vz=vz, wz=wz)

                rospy.loginfo_throttle(
                    0.25,
                    "AVANZANDO | goal_pose=%.3f x=%.3f restante=%.3f | ey=%.3f ez=%.3f eyaw=%.3f | cmd=(%.3f, %.3f, %.3f, %.3f)",
                    self.goal_x,
                    self.current_x,
                    remaining,
                    ey,
                    ez,
                    eyaw,
                    vx, vy, vz, wz
                )
                self.rate.sleep()
                continue

            in_range, tube_percent = self.tube_in_range()
            if in_range:
                rospy.loginfo(
                    "TUBO EN RANGO [%.1f, %.1f]: %.2f%% -> HOLD, PUBLICA SOLUCION Y LIBERA CMD_VEL",
                    self.percent_low, self.percent_high, tube_percent
                )
                self.do_hold_then_handoff(self.handoff_hold_time)
                return False

            selected_percent = self.select_relevant_percent()
            if selected_percent is None:
                vx = self.compute_forward_only_vx(remaining)
                vy = self.sat_tanh(self.kp_hold_y, ey, self.vy_hold_max)
                vz = self.sat_tanh(self.kp_hold_z, ez, self.vz_hold_max)
                wz = self.saturate(self.kp_hold_yaw * eyaw, self.wz_hold_max)
                self.publish_cmd(vx=vx, vy=vy, vz=vz, wz=wz)
                self.rate.sleep()
                continue

            percent_rate = self.estimate_selected_percent_rate()
            vx, _, mode = self.compute_visual_vx_pd(selected_percent, percent_rate)

            vy = self.sat_tanh(self.kp_hold_y, ey, self.vy_hold_max)
            vz = self.sat_tanh(self.kp_hold_z, ez, self.vz_hold_max)
            wz = self.saturate(self.kp_hold_yaw * eyaw, self.wz_hold_max)

            self.publish_cmd(vx=vx, vy=vy, vz=vz, wz=wz)

            rospy.loginfo_throttle(
                0.25,
                "CON TUBO | pct=%.2f mode=%s | goal_pose=%.3f x=%.3f restante=%.3f | ey=%.3f ez=%.3f eyaw=%.3f | cmd=(%.3f, %.3f, %.3f, %.3f)",
                selected_percent,
                mode,
                self.goal_x,
                self.current_x,
                remaining,
                ey,
                ez,
                eyaw,
                vx, vy, vz, wz
            )
            self.rate.sleep()

        return False

    def run(self):
        self.start_keyboard_listener()
        while not rospy.is_shutdown():
            if self.check_emergency_land():
                return

            if self.activation_received and not self.executing:
                self.executing = True
                self.activation_received = False

                mission_done = self.execute_forward_to_goal()

                self.publish_zero_cmd_burst()
                rospy.sleep(0.1)

                self.executing = False

                if mission_done:
                    return

            self.rate.sleep()


if __name__ == "__main__":
    node = None
    try:
        node = MissionAdvanceNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        if node is not None:
            node.restore_terminal()
