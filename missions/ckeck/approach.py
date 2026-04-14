#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
import threading
import sys
import select
import termios
import tty

from std_msgs.msg import Empty, String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class ApproachNode:
    def __init__(self):
        rospy.init_node("approach_node")

        # Topics
        self.takeoff_topic = rospy.get_param("~takeoff_topic", "/bebop/takeoff")
        self.land_topic = rospy.get_param("~land_topic", "/bebop/land")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/bebop/cmd_vel")
        self.odom_topic = rospy.get_param("~odom_topic", "/bebop/odom")
        self.activation_topic = rospy.get_param("~activation_topic", "/activacion_dron")

        # -------------------------
        # Variables principales
        # -------------------------
        self.approach_distance = rospy.get_param("~approach_distance", 1.7) # distancia en X
        self.initial_hold_time = 1.0 # hold inicial
        self.pre_rotate_hold_time = 1.0 # hold antes de rotar
        self.rotation_deg = rospy.get_param("~rotation_deg", -90.0) # giro final en yaw
        self.ctrl_rate = rospy.get_param("~ctrl_rate", 30.0)
        self.max_move_time = rospy.get_param("~max_move_time", 30.0)
        self.max_rotate_time = rospy.get_param("~max_rotate_time", 12.0)

        # Control en x
        self.kp_x = rospy.get_param("~kp_x", 0.9)
        self.vx_max = rospy.get_param("~vx_max", 0.06)
        self.vx_min = rospy.get_param("~vx_min", 0.015)
        self.goal_tolerance = rospy.get_param("~goal_tolerance", 0.03)

        # Control en y, z, yaw
        self.kp_y = rospy.get_param("~kp_y", 0.7)
        self.kp_z = rospy.get_param("~kp_z", 0.9)
        self.kp_yaw = rospy.get_param("~kp_yaw", 1.0)

        self.vy_max = rospy.get_param("~vy_max", 0.03)
        self.vz_max = rospy.get_param("~vz_max", 0.05)
        self.wz_max = rospy.get_param("~wz_max", 0.18)
        self.yaw_tolerance_deg = rospy.get_param("~yaw_tolerance_deg", 3.0)

        # Estado
        self.current_x = None
        self.current_y = None
        self.current_z = None
        self.current_yaw = None

        self.x_ref = None
        self.y_ref = None
        self.z_ref = None
        self.yaw_ref = None
        self.x_goal = None
        self.yaw_goal = None

        # Safety teclado
        self.emergency_land = False
        self.keyboard_thread = None
        self.term_settings = None

        # ROS
        self.takeoff_pub = rospy.Publisher(self.takeoff_topic, Empty, queue_size=1)
        self.land_pub = rospy.Publisher(self.land_topic, Empty, queue_size=1)
        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=1)
        self.activation_pub = rospy.Publisher(self.activation_topic, String, queue_size=1, latch=True)

        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=1)

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

    def emergency_land_now(self):
        self.publish_cmd(0.0, 0.0, 0.0, 0.0)
        rospy.sleep(0.1)
        rospy.logwarn("Aterrizando por emergencia...")
        self.land_pub.publish(Empty())
        rospy.sleep(1.0)

    def check_emergency_land(self):
        if self.emergency_land:
            self.emergency_land_now()
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
                rospy.logerr("Timeout esperando odometría.")
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

    def compute_forward_vx(self, remaining):
        if remaining <= 0.0:
            return 0.0
        vx = self.sat_tanh(self.kp_x, remaining, self.vx_max)
        if vx > 0.0:
            vx = max(vx, self.vx_min)
        return min(vx, self.vx_max)

    def compute_hold_cmd(self, x_ref, y_ref, z_ref, yaw_ref):
        ex = x_ref - self.current_x
        ey = y_ref - self.current_y
        ez = z_ref - self.current_z
        eyaw = self.normalize_angle(yaw_ref - self.current_yaw)

        vx = self.sat_tanh(self.kp_x, ex, self.vx_max)
        vy = self.sat_tanh(self.kp_y, ey, self.vy_max)
        vz = self.sat_tanh(self.kp_z, ez, self.vz_max)
        wz = self.saturate(self.kp_yaw * eyaw, self.wz_max)
        return vx, vy, vz, wz

    def do_hold(self, hold_time, x_ref, y_ref, z_ref, yaw_ref, label="HOLD"):
        rospy.loginfo("%s de %.1f s...", label, hold_time)

        hold_start = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            if self.check_emergency_land():
                return False

            if rospy.Time.now().to_sec() - hold_start >= hold_time:
                break

            if self.has_pose():
                vx, vy, vz, wz = self.compute_hold_cmd(x_ref, y_ref, z_ref, yaw_ref)
                self.publish_cmd(vx, vy, vz, wz)
            else:
                self.publish_cmd(0.0, 0.0, 0.0, 0.0)

            self.rate.sleep()

        return True

    def do_final_rotation(self):
        if not self.has_pose():
            rospy.logwarn("No hay pose para ejecutar la rotación final.")
            return False

        yaw_offset_rad = math.radians(self.rotation_deg)
        self.yaw_goal = self.normalize_angle(self.current_yaw + yaw_offset_rad)
        yaw_tolerance = math.radians(self.yaw_tolerance_deg)

        # Mantener únicamente la altura actual durante el giro
        z_ref_rotate = self.current_z

        rospy.loginfo(
            "Iniciando rotación final | yaw_actual=%.2f deg | yaw_goal=%.2f deg",
            math.degrees(self.current_yaw),
            math.degrees(self.yaw_goal)
        )

        rotate_start = rospy.Time.now().to_sec()

        while not rospy.is_shutdown():
            if self.check_emergency_land():
                return False

            if not self.has_pose():
                self.publish_cmd(0.0, 0.0, 0.0, 0.0)
                self.rate.sleep()
                continue

            ez = z_ref_rotate - self.current_z
            eyaw = self.normalize_angle(self.yaw_goal - self.current_yaw)

            if abs(eyaw) <= yaw_tolerance:
                rospy.loginfo(
                    "ROTACIÓN COMPLETADA | yaw_actual=%.2f deg | error=%.2f deg",
                    math.degrees(self.current_yaw),
                    math.degrees(eyaw)
                )
                break

            # Rotar sobre su propio eje: sin corrección en x ni y
            vx = 0.0
            vy = 0.0
            vz = self.sat_tanh(self.kp_z, ez, self.vz_max)
            wz = self.saturate(self.kp_yaw * eyaw, self.wz_max)

            self.publish_cmd(vx, vy, vz, wz)

            rospy.loginfo_throttle(
                0.25,
                "ROTANDO SOBRE EJE | yaw=%.2f deg goal=%.2f deg err=%.2f deg | cmd=(%.3f, %.3f, %.3f, %.3f)",
                math.degrees(self.current_yaw),
                math.degrees(self.yaw_goal),
                math.degrees(eyaw),
                vx, vy, vz, wz
            )

            if rospy.Time.now().to_sec() - rotate_start > self.max_rotate_time:
                rospy.logwarn("Timeout en rotación final.")
                break

            self.rate.sleep()

        self.publish_cmd(0.0, 0.0, 0.0, 0.0)
        rospy.sleep(0.2)
        return True

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

        if not self.wait_for_pose():
            return

        # Capturar referencia al iniciar el hold
        self.x_ref = self.current_x
        self.y_ref = self.current_y
        self.z_ref = self.current_z
        self.yaw_ref = self.current_yaw
        self.x_goal = self.x_ref + self.approach_distance

        rospy.loginfo(
            "Referencia capturada | x=%.3f y=%.3f z=%.3f yaw=%.2f deg | x_goal=%.3f",
            self.x_ref, self.y_ref, self.z_ref, math.degrees(self.yaw_ref), self.x_goal
        )

        if not self.do_hold(self.initial_hold_time, self.x_ref, self.y_ref, self.z_ref, self.yaw_ref, "HOLD inicial"):
            return

        move_start = rospy.Time.now().to_sec()
        rospy.loginfo("Iniciando approach hacia %.3f m en x", self.approach_distance)

        while not rospy.is_shutdown():
            if self.check_emergency_land():
                return

            if not self.has_pose():
                self.publish_cmd(0.0, 0.0, 0.0, 0.0)
                self.rate.sleep()
                continue

            remaining = self.x_goal - self.current_x
            ey = self.y_ref - self.current_y
            ez = self.z_ref - self.current_z
            eyaw = self.normalize_angle(self.yaw_ref - self.current_yaw)

            if remaining <= self.goal_tolerance:
                rospy.loginfo(
                    "APPROACH COMPLETADO | x_goal=%.3f x_actual=%.3f restante=%.3f",
                    self.x_goal, self.current_x, remaining
                )
                break

            vx = self.compute_forward_vx(remaining)
            vy = self.sat_tanh(self.kp_y, ey, self.vy_max)
            vz = self.sat_tanh(self.kp_z, ez, self.vz_max)
            wz = self.saturate(self.kp_yaw * eyaw, self.wz_max)

            self.publish_cmd(vx, vy, vz, wz)

            rospy.loginfo_throttle(
                0.25,
                "APPROACH | x=%.3f goal=%.3f restante=%.3f | ey=%.3f ez=%.3f eyaw=%.3f | cmd=(%.3f, %.3f, %.3f, %.3f)",
                self.current_x, self.x_goal, remaining,
                ey, ez, eyaw,
                vx, vy, vz, wz
            )

            if rospy.Time.now().to_sec() - move_start > self.max_move_time:
                rospy.logwarn("Timeout en approach.")
                break

            self.rate.sleep()

        # Detener dron al llegar
        self.publish_cmd(0.0, 0.0, 0.0, 0.0)
        rospy.sleep(0.2)

        # Hold antes de rotar
        x_hold = self.current_x if self.current_x is not None else self.x_goal
        y_hold = self.current_y if self.current_y is not None else self.y_ref
        z_hold = self.current_z if self.current_z is not None else self.z_ref
        yaw_hold = self.current_yaw if self.current_yaw is not None else self.yaw_ref

        if not self.do_hold(self.pre_rotate_hold_time, x_hold, y_hold, z_hold, yaw_hold, "HOLD antes de rotar"):
            return

        # Rotación final sobre yaw
        if not self.do_final_rotation():
            return

        # Activar naranja.py
        rospy.loginfo("Publicando READY en %s", self.activation_topic)
        self.activation_pub.publish(String(data="READY"))
        rospy.sleep(0.2)

        # Aterrizar
        rospy.loginfo("Aterrizando...")
        self.land_pub.publish(Empty())
        rospy.sleep(2.0)


if __name__ == "__main__":
    node = None
    try:
        node = ApproachNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        if node is not None:
            node.restore_terminal()
