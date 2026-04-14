#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
import threading
import sys
import select
import termios
import tty

from std_msgs.msg import String, Empty, Float32MultiArray, Int32MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class MissionExecuteSolution:
    STATE_WAITING_SOLUTION = "WAITING_SOLUTION"
    STATE_EXECUTING = "EXECUTING"
    STATE_HOLDING = "HOLDING"

    def __init__(self):
        rospy.init_node("mission_execute_solution")

        self.solution_topic = rospy.get_param("~solution_topic", "/mission_solution")
        self.executor_state_topic = rospy.get_param("~executor_state_topic", "/ejecutor_state")
        self.average_topic = rospy.get_param("~average_topic", "/average_color")
        self.bboxes_topic = rospy.get_param("~bboxes_topic", "/orange_bboxes")
        self.target_tube_bbox_topic = rospy.get_param("~target_tube_bbox_topic", "/target_tube_bbox")
        self.odom_topic = rospy.get_param("~odom_topic", "/bebop/odom")
        self.land_topic = rospy.get_param("~land_topic", "/bebop/land")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/bebop/cmd_vel")

        self.ctrl_rate = rospy.get_param("~ctrl_rate", 30.0)

        # Rango objetivo lateral
        self.percent_low = rospy.get_param("~percent_low", 11.0)
        self.percent_high = rospy.get_param("~percent_high", 13.0)
        self.target_percent = rospy.get_param("~target_percent", 12.0)
        self.exit_hold_time = rospy.get_param("~exit_hold_time", 1.0)

        # Control lateral guiado por porcentaje
        self.ky_far = rospy.get_param("~ky_far", 0.025)
        self.ky_near = rospy.get_param("~ky_near", 0.008)
        self.percent_far = rospy.get_param("~percent_far", 8.0)
        self.vy_max = rospy.get_param("~vy_max", 0.03)
        self.vy_min = rospy.get_param("~vy_min", 0.010)

        # Mantener pose
        self.kp_x = rospy.get_param("~kp_x", 0.8)
        self.kp_z = rospy.get_param("~kp_z", 0.9)
        self.kp_yaw = rospy.get_param("~kp_yaw", 1.0)

        self.vx_max = rospy.get_param("~vx_max", 0.03)
        self.vz_max = rospy.get_param("~vz_max", 0.05)
        self.wz_max = rospy.get_param("~wz_max", 0.18)

        self.max_exec_time = rospy.get_param("~max_exec_time", 12.0)
        self.no_detection_timeout = rospy.get_param("~no_detection_timeout", 1.0)
        self.alpha_percent = rospy.get_param("~alpha_percent", 0.25)

        self.land_pub = rospy.Publisher(self.land_topic, Empty, queue_size=1)
        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=1)
        self.executor_state_pub = rospy.Publisher(self.executor_state_topic, String, queue_size=1, latch=True)

        # Estado máquina
        self.state = self.STATE_WAITING_SOLUTION
        self.solution_received = None
        self.pending_solution = None

        self.emergency_land = False

        # Pose
        self.current_x = None
        self.current_y = None
        self.current_z = None
        self.current_yaw = None
        self.last_odom_msg = None

        # Percepción múltiple alineada
        self.current_percents_raw = []
        self.current_percents_filt = []
        self.current_bboxes = []
        self.current_centers = []
        self.last_average_msg_time = None
        self.last_bboxes_msg_time = None

        # Target bloqueado
        self.target_bbox_ref = None
        self.target_bbox_current = None
        self.target_center_current = None
        self.target_percent_raw = None
        self.target_percent_filt = None

        self.keyboard_thread = None
        self.term_settings = None
        self.rate = rospy.Rate(self.ctrl_rate)

        rospy.Subscriber(self.solution_topic, String, self.solution_callback, queue_size=1)
        rospy.Subscriber(self.average_topic, Float32MultiArray, self.average_callback, queue_size=1)
        rospy.Subscriber(self.bboxes_topic, Int32MultiArray, self.bboxes_callback, queue_size=1)
        rospy.Subscriber(self.target_tube_bbox_topic, Int32MultiArray, self.target_bbox_callback, queue_size=1)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=1)

        rospy.loginfo("Nodo ejecutor en espera de solucion en %s", self.solution_topic)

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
            self.land_pub.publish(Empty())
            rospy.sleep(1.0)
            return True
        return False

    # -------------------------
    # Callbacks
    # -------------------------
    def average_callback(self, msg):
        now = rospy.Time.now().to_sec()
        raw_vals = [float(v) for v in msg.data]
        self.current_percents_raw = raw_vals[:]

        if len(raw_vals) == 0:
            self.current_percents_filt = []
            self.last_average_msg_time = now
            return

        if len(self.current_percents_filt) != len(raw_vals):
            self.current_percents_filt = raw_vals[:]
        else:
            new_filt = []
            for old_v, new_v in zip(self.current_percents_filt, raw_vals):
                filt_v = self.alpha_percent * new_v + (1.0 - self.alpha_percent) * old_v
                new_filt.append(filt_v)
            self.current_percents_filt = new_filt

        self.last_average_msg_time = now

    def bboxes_callback(self, msg):
        now = rospy.Time.now().to_sec()
        data = list(msg.data)

        self.current_bboxes = []
        self.current_centers = []

        usable = len(data) - (len(data) % 4)
        for i in range(0, usable, 4):
            x1 = int(data[i + 0])
            y1 = int(data[i + 1])
            x2 = int(data[i + 2])
            y2 = int(data[i + 3])

            if x1 >= 0 and y1 >= 0 and x2 > x1 and y2 > y1:
                self.current_bboxes.append((x1, y1, x2, y2))
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2
                self.current_centers.append((cx, cy))

        self.last_bboxes_msg_time = now

    def target_bbox_callback(self, msg):
        data = list(msg.data)
        if len(data) >= 4:
            x1 = int(data[0])
            y1 = int(data[1])
            x2 = int(data[2])
            y2 = int(data[3])
            if x1 >= 0 and y1 >= 0 and x2 > x1 and y2 > y1:
                self.target_bbox_ref = (x1, y1, x2, y2)
                rospy.loginfo("TARGET BBOX RECIBIDO: %s", str(self.target_bbox_ref))
                return

        self.target_bbox_ref = None
        rospy.logwarn("TARGET BBOX recibido vacío o inválido.")

    def odom_callback(self, msg):
        self.last_odom_msg = msg
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_z = msg.pose.pose.position.z

        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_yaw = yaw

    def solution_callback(self, msg):
        solution = msg.data.strip().upper()
        if solution in ["DERECHA", "IZQUIERDA"]:
            self.solution_received = solution
            rospy.loginfo("SOLUCION RECIBIDA: %s", solution)

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

    def in_target_range(self, percent_now):
        return self.percent_low <= percent_now <= self.percent_high

    def compute_lateral_speed_from_percent(self, solution, percent_now):
        error = self.target_percent - percent_now
        abs_error = abs(error)

        if abs_error >= self.percent_far:
            vy_mag = self.ky_far * math.sqrt(abs_error)
        else:
            vy_mag = self.ky_near * abs_error

        vy_mag = min(vy_mag, self.vy_max)
        if vy_mag > 0.0:
            vy_mag = max(vy_mag, self.vy_min)

        nominal_sign = -1.0 if solution == "DERECHA" else 1.0

        if error < 0.0:
            sign = nominal_sign
        else:
            sign = -nominal_sign

        return sign * vy_mag, error

    def publish_completed(self):
        rospy.loginfo("PUBLICANDO ESTADO EN %s: COMPLETADO", self.executor_state_topic)
        self.executor_state_pub.publish(String(data="COMPLETADO"))

    def bbox_center(self, bbox):
        x1, y1, x2, y2 = bbox
        return ((x1 + x2) // 2, (y1 + y2) // 2)

    def bbox_distance_score(self, ref_bbox, cand_bbox):
        rcx, rcy = self.bbox_center(ref_bbox)
        ccx, ccy = self.bbox_center(cand_bbox)

        ref_w = max(1.0, float(ref_bbox[2] - ref_bbox[0]))
        ref_h = max(1.0, float(ref_bbox[3] - ref_bbox[1]))
        cand_w = float(cand_bbox[2] - cand_bbox[0])
        cand_h = float(cand_bbox[3] - cand_bbox[1])

        dc = math.hypot(ccx - rcx, ccy - rcy)
        ds = abs(cand_w - ref_w) + abs(cand_h - ref_h)

        return dc + 0.35 * ds

    def update_locked_target_from_candidates(self):
        if self.target_bbox_ref is None:
            return False

        if len(self.current_bboxes) == 0 or len(self.current_percents_filt) == 0:
            return False

        pair_count = min(len(self.current_bboxes), len(self.current_percents_filt))
        if pair_count <= 0:
            return False

        best_idx = None
        best_score = None

        for i in range(pair_count):
            cand_bbox = self.current_bboxes[i]
            score = self.bbox_distance_score(self.target_bbox_ref, cand_bbox)

            if best_score is None or score < best_score:
                best_score = score
                best_idx = i

        if best_idx is None:
            return False

        self.target_bbox_current = self.current_bboxes[best_idx]
        self.target_center_current = self.current_centers[best_idx]
        self.target_percent_raw = self.current_percents_raw[best_idx] if best_idx < len(self.current_percents_raw) else None
        self.target_percent_filt = self.current_percents_filt[best_idx]

        self.target_bbox_ref = self.target_bbox_current
        return True

    def reset_execution_state(self):
        self.pending_solution = None
        self.target_bbox_ref = None
        self.target_bbox_current = None
        self.target_center_current = None
        self.target_percent_raw = None
        self.target_percent_filt = None

    def do_hold_then_complete(self, hold_time):
        if not self.has_pose():
            self.publish_zero_cmd_burst()
            rospy.sleep(0.2)
            self.publish_completed()
            return

        x_ref = self.current_x
        y_ref = self.current_y
        z_ref = self.current_z
        yaw_ref = self.current_yaw

        rospy.loginfo("HOLD %.1f s antes de publicar COMPLETADO...", hold_time)
        hold_start = rospy.Time.now().to_sec()

        while not rospy.is_shutdown():
            if self.check_emergency_land():
                return

            if rospy.Time.now().to_sec() - hold_start >= hold_time:
                break

            if self.has_pose():
                ex = x_ref - self.current_x
                ey = y_ref - self.current_y
                ez = z_ref - self.current_z
                eyaw = self.normalize_angle(yaw_ref - self.current_yaw)

                vx = self.sat_tanh(self.kp_x, ex, self.vx_max)
                vy = self.sat_tanh(1.0, ey, 0.03)
                vz = self.sat_tanh(self.kp_z, ez, self.vz_max)
                wz = self.saturate(self.kp_yaw * eyaw, self.wz_max)

                self.publish_cmd(vx=vx, vy=vy, vz=vz, wz=wz)
            else:
                self.publish_cmd(0.0, 0.0, 0.0, 0.0)

            self.rate.sleep()

        self.publish_zero_cmd_burst()
        rospy.sleep(0.1)
        self.publish_completed()

    def execute_lateral_motion(self, solution):
        if not self.wait_for_pose():
            rospy.logwarn("No hay odometría suficiente para ejecutar la maniobra.")
            return

        if self.target_bbox_ref is None:
            rospy.logwarn("No se recibió target_tube_bbox. No se puede fijar el tubo objetivo.")
            return

        x_ref = self.current_x
        z_ref = self.current_z
        yaw_ref = self.current_yaw

        rospy.loginfo("EJECUTANDO SOLUCION %s con target bloqueado %s", solution, str(self.target_bbox_ref))
        start_time = rospy.Time.now().to_sec()

        while not rospy.is_shutdown():
            if self.check_emergency_land():
                return

            if not self.has_pose():
                self.publish_cmd(0.0, 0.0, 0.0, 0.0)
                self.rate.sleep()
                continue

            perception_stale = (
                self.last_average_msg_time is None or
                self.last_bboxes_msg_time is None or
                (rospy.Time.now().to_sec() - self.last_average_msg_time > self.no_detection_timeout) or
                (rospy.Time.now().to_sec() - self.last_bboxes_msg_time > self.no_detection_timeout)
            )

            if perception_stale:
                rospy.logwarn("CASO EXTREMO: percepción visual desactualizada. Se detiene la maniobra.")
                break

            found_target = self.update_locked_target_from_candidates()
            if not found_target or self.target_percent_filt is None:
                rospy.logwarn("CASO EXTREMO: se perdió el tubo objetivo bloqueado. Se detiene la maniobra.")
                break

            percent_now = self.target_percent_filt

            if self.in_target_range(percent_now):
                rospy.loginfo(
                    "OBJETIVO ALCANZADO sobre el tubo bloqueado: porcentaje = %.2f dentro de [%.2f, %.2f]",
                    percent_now, self.percent_low, self.percent_high
                )
                break

            ex = x_ref - self.current_x
            ez = z_ref - self.current_z
            eyaw = self.normalize_angle(yaw_ref - self.current_yaw)

            vy, error = self.compute_lateral_speed_from_percent(solution, percent_now)

            vx = self.sat_tanh(self.kp_x, ex, self.vx_max)
            vz = self.sat_tanh(self.kp_z, ez, self.vz_max)
            wz = self.saturate(self.kp_yaw * eyaw, self.wz_max)

            self.publish_cmd(vx=vx, vy=vy, vz=vz, wz=wz)

            rospy.loginfo_throttle(
                0.25,
                "EJECUTANDO %s | target_bbox=%s | pct_raw=%.2f pct_filt=%.2f objetivo=[%.2f, %.2f] target=%.2f | err=%.2f | ex=%.3f ez=%.3f eyaw=%.3f | cmd=(%.3f, %.3f, %.3f, %.3f)",
                solution,
                str(self.target_bbox_current),
                self.target_percent_raw if self.target_percent_raw is not None else -1.0,
                percent_now,
                self.percent_low,
                self.percent_high,
                self.target_percent,
                error,
                ex, ez, eyaw,
                vx, vy, vz, wz
            )

            if rospy.Time.now().to_sec() - start_time > self.max_exec_time:
                rospy.logwarn("Timeout en la maniobra lateral.")
                break

            self.rate.sleep()

    def run(self):
        self.start_keyboard_listener()

        while not rospy.is_shutdown():
            if self.check_emergency_land():
                return

            if self.state == self.STATE_WAITING_SOLUTION:
                if self.solution_received is not None:
                    self.pending_solution = self.solution_received
                    self.solution_received = None
                    self.state = self.STATE_EXECUTING

            elif self.state == self.STATE_EXECUTING:
                self.execute_lateral_motion(self.pending_solution)
                self.publish_zero_cmd_burst()
                rospy.sleep(0.2)
                self.state = self.STATE_HOLDING

            elif self.state == self.STATE_HOLDING:
                self.do_hold_then_complete(self.exit_hold_time)
                self.reset_execution_state()
                self.state = self.STATE_WAITING_SOLUTION

            self.rate.sleep()


if __name__ == "__main__":
    node = None
    try:
        node = MissionExecuteSolution()
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        if node is not None:
            node.restore_terminal()
