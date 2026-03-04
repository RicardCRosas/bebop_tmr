#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from tf.transformations import euler_from_quaternion
from bebop_msgs.msg import Ardrone3PilotingStateAltitudeChanged


class BebopAdvancedController:

    def __init__(self, pub_cmd_vel, pub_takeoff, pub_land, pub_camera):

        self.pub_cmd_vel = pub_cmd_vel
        self.pub_takeoff = pub_takeoff
        self.pub_land = pub_land
        self.pub_camera = pub_camera

        rospy.Subscriber("/bebop/odom", Odometry, self.odom_callback)
        rospy.Subscriber(
            "/bebop/states/ardrone3/PilotingState/AltitudeChanged",
            Ardrone3PilotingStateAltitudeChanged,
            self.altitude_callback
        )

        # Estados actuales
        self.current_position = None
        self.current_yaw = 0.0
        self.current_altitude = 0.0

        # Referencias iniciales
        self.initial_position = None
        self.initial_altitude = None

        # Target
        self.target_position = None
        self.target_yaw = None

        self.navigation_active = False
        self.hover_active = False
        self.emergency_stop = False

        # 🔥 Ganancias separadas
        self.kp_xy = 0.8
        self.kp_z = 1.2
        self.kp_yaw = 1.5

        self.rate = rospy.Rate(30)

    # =====================================================
    # ODOM
    # =====================================================

    def odom_callback(self, msg):

        self.current_position = msg.pose.pose.position

        q = msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_yaw = yaw

        if self.initial_position is None:
            self.initial_position = self.current_position

    # =====================================================
    # ALTURA
    # =====================================================

    def altitude_callback(self, msg):

        self.current_altitude = msg.altitude

        if self.initial_altitude is None:
            self.initial_altitude = msg.altitude

    # =====================================================
    # POSICIÓN RELATIVA
    # =====================================================

    def get_relative_position(self):

        rx = self.current_position.x - self.initial_position.x
        ry = self.current_position.y - self.initial_position.y
        rz = self.current_altitude - self.initial_altitude

        return rx, ry, rz

    # =====================================================
    # CONTROLADORES EN CASCADA
    # =====================================================

    def xy_position_controller(self):

        rx, ry, _ = self.get_relative_position()

        error_x = self.target_position[0] - rx
        error_y = self.target_position[1] - ry

        vx = self.kp_xy * error_x
        vy = self.kp_xy * error_y

        vx = max(min(vx, 0.4), -0.4)
        vy = max(min(vy, 0.4), -0.4)

        xy_done = math.sqrt(error_x**2 + error_y**2) < 0.05

        return vx, vy, xy_done

    def z_position_controller(self):

        _, _, rz = self.get_relative_position()

        error_z = self.target_position[2] - rz

        vz = self.kp_z * error_z
        vz = max(min(vz, 0.3), -0.3)

        z_done = abs(error_z) < 0.03

        return vz, z_done

    def yaw_controller(self):

        if self.target_yaw is None:
            return 0.0, True

        error = self.normalize_angle(self.target_yaw - self.current_yaw)

        wz = self.kp_yaw * error
        wz = max(min(wz, 0.5), -0.5)

        yaw_done = abs(error) < math.radians(3)

        return wz, yaw_done

    # =====================================================
    # UPDATE PRINCIPAL (CASCADA)
    # =====================================================

    def update(self):

        if self.emergency_stop:
            self.stop()
            return

        if not self.navigation_active or self.target_position is None:
            return

        vx, vy, xy_done = self.xy_position_controller()
        vz, z_done = self.z_position_controller()
        wz, yaw_done = self.yaw_controller()

        self.send_manual_velocity(vx, vy, vz, wz)

        if xy_done and z_done and yaw_done:
            rospy.loginfo("✔ Target alcanzado (cascada)")
            self.navigation_active = False
            self.stop()

    # =====================================================
    # SET TARGET
    # =====================================================

    def set_target(self, x, y, z, yaw_deg=None):

        self.target_position = (x, y, z)

        if yaw_deg is not None:
            self.target_yaw = math.radians(yaw_deg)
        else:
            self.target_yaw = None

        self.navigation_active = True

    # =====================================================
    # UTILIDADES
    # =====================================================

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def send_manual_velocity(self, vx, vy, vz, wz=0.0):

        twist = Twist()
        twist.linear.x = max(min(vx, 1.0), -1.0)
        twist.linear.y = max(min(vy, 1.0), -1.0)
        twist.linear.z = max(min(vz, 1.0), -1.0)
        twist.angular.z = max(min(wz, 1.0), -1.0)

        self.pub_cmd_vel.publish(twist)

    def stop(self):
        self.pub_cmd_vel.publish(Twist())

    def takeoff(self):
        self.pub_takeoff.publish(Empty())

    def land(self):
        self.pub_land.publish(Empty())

    # ---------------MOVIMIENTOS BÁSICOS DE LA CAMARA
    # Controla el movimiento vertical (tilt) de la cámara.
    #   tilt: valor angular (grados o radianes según configuración del dron). En este caso son grados
    # Controla el movimiento horizontal (pan) de la cámara.
    #   pan: valor angular (grados o radianes según configuración del dron). En este caso son grados
    # 
    # =====================================================
    
    def camera_tilt(self, tilt_deg):
        cam = Twist()
        cam.angular.y = tilt_deg
        self.pub_camera.publish(cam)
        print(f'\n Adjusting camera tilt: {tilt_deg} degrees...')

    def camera_pan(self, pan_deg):
        print(f'\n Adjusting camera pan: {pan_deg} degrees...')
        cam = Twist()
        cam.angular.z = pan_deg
        self.pub_camera.publish(cam)
