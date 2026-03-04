#!/usr/bin/env python3
# bebop_advanced_controller.py

# Clase para controlar movimientos básicos del dron Parrot Bebop usando ROS 

# Más que nada es para autonomo de las misiones

# Las velocidades son tienen estos rangos [-1,1]
# Los valores >1 o <-1 se saturan automáticamente y evitan por ende esos valores máximos en lugar del que sobrepasa.
# Esto es % de la velocidad maxima del dron

# linear.x   # Avance y retroceso (x positivo adelante)
# linear.y   # Movimiento lateral (y positivo izquierda)
# linear.z   # Subir o bajar (z positivo arriba)
# angular.x  # Rotación sobre eje X (rara vez usada) 
# angular.y  # Rotación sobre eje Y (rara vez usada)
# angular.z  # Giro (yaw) (positivo giro izquierda)

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

        # Estado actual
        self.current_position = None
        self.current_yaw = 0.0
        self.current_altitude = 0.0

        # Referencias iniciales
        self.initial_position = None
        self.initial_altitude = None

        # Targets
        self.target_x = None
        self.target_y = None
        self.target_z = None
        self.target_yaw = None

        # Flags
        self.navigation_active = False
        self.emergency_stop = False

        # Ganancias (SUAVES para evitar oscilación)
        self.kp_xy = 0.6
        self.kd_xy = 0.15

        self.kp_z = 0.8
        self.kp_yaw = 1.2

        self.prev_error_x = 0
        self.prev_error_y = 0

        self.error_x = 0
        self.error_y = 0
        self.error_z = 0
        self.error_yaw = 0

        self.rate = rospy.Rate(30)

    # =====================================================
    # CALLBACKS
    # =====================================================

    def odom_callback(self, msg):

        self.current_position = msg.pose.pose.position

        orientation_q = msg.pose.pose.orientation
        orientation_list = [
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ]

        (_, _, yaw) = euler_from_quaternion(orientation_list)
        self.current_yaw = yaw

        if self.initial_position is None:
            self.initial_position = self.current_position
            rospy.loginfo("✔ Posición XY inicial fijada")
            rospy.loginfo(self.initial_position)

    def altitude_callback(self, msg):
        self.current_altitude = msg.altitude

        if self.initial_altitude is None:
            self.initial_altitude = msg.altitude
            rospy.loginfo("✔ Altura inicial fijada")
            rospy.loginfo(self.initial_altitude)

    # =====================================================
    # UTIL
    # =====================================================
    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def get_relative_position(self):
        rx = self.current_position.x - self.initial_position.x
        ry = self.current_position.y - self.initial_position.y
        rz = self.current_altitude - self.initial_altitude
        return rx, ry, rz
    

    # =====================================================
    # COMANDOS BÁSICOS
    # =====================================================
    def takeoff(self):
        if not self.emergency_stop:
            self.pub_takeoff.publish(Empty())

    def land(self):
        self.pub_land.publish(Empty())

    def stop(self):
        self.pub_cmd_vel.publish(Twist())

    def send_velocity(self, vx, vy, vz, wz=0.0):
        twist = Twist()
        twist.linear.x = max(min(vx, 1.0), -1.0)
        twist.linear.y = max(min(vy, 1.0), -1.0)
        twist.linear.z = max(min(vz, 1.0), -1.0)
        twist.angular.z = max(min(wz, 1.0), -1.0)

        self.pub_cmd_vel.publish(twist)

    # ---------------------------
    # DETENER MOVIMIENTO
    # ---------------------------
    def reset_twist(self):
        """
        Reinicia todos los componentes del Twist a 0
        para detener cualquier movimiento activo del dron.
        """
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        # Publica el comando de velocidad cero
        self.pub_cmd_vel.publish(twist) 
    
    def publish_twist(self):
        self.pub_cmd_vel.publish(self.twist)

    def wait_for_odometry(self):
        while self.current_position is None and not rospy.is_shutdown():
            rospy.sleep(0.1)  
    
    # =====================================================
    # TARGETS
    # =====================================================

    def set_target_position(self, x, y, z):
        self.target_x = x
        self.target_y = y
        self.target_z = z
        self.target_yaw = None
        self.navigation_active = True

    def set_target_yaw(self, yaw_deg):
        self.target_yaw = math.radians(yaw_deg)
        self.navigation_active = True

    # =====================================================
    # EMERGENCIA
    # =====================================================

    def activate_emergency(self):
        rospy.logwarn("🚨 EMERGENCY STOP ACTIVATED")

        self.emergency_stop = True
        self.navigation_active = False
        self.hover_active = False

        self.stop()
        rospy.sleep(0.1)  # pequeña pausa
        self.pub_land.publish(Empty())

    def clear_emergency(self):
        self.emergency_stop = False
   
    # =====================================================
    # =====================================================
    # =====================================================
    comentario_largo = """
    # MOVIMIENTOS BLOQUEANTES (METROS REALES)
    # =====================================================

    def move_forward(self, distance):

        x0, _, _ = self.get_relative_position()

        while not rospy.is_shutdown():
            rx, _, _ = self.get_relative_position()
            
            dx = rx - x0
            error = distance - dx

            if abs(error) < 0.02:
                break   

             #velocidad proporcional
            kp = 0.8
            speed = max(min(kp * error, 0.5), -0.5)


            self.send_velocity(speed, 0, 0)
            self.rate.sleep()

        self.stop()

    def move_backward(self, distance):
        self.move_forward(-distance)

    def move_left(self, distance):
        _, y0, _ = self.get_relative_position()

        while not rospy.is_shutdown():
            _, ry, _ = self.get_relative_position()
            
            dy = ry - y0
            error = distance - dy

            kp = 0.8
            if abs(error) < 0.02:
                break

            speed = max(min(kp * error, 0.5), -0.5)

            self.send_velocity(0, speed, 0)
            self.rate.sleep()

        self.stop()

    def move_right(self, distance):
        self.move_left(-distance)

    # =====================================================
    # GIRO CORREGIDO (NO DERIVA)
    # =====================================================
    def turn(self, angle_deg):

        yaw0 = self.current_yaw
        target = math.radians(angle_deg)
        #direction = 1 if angle_deg > 0 else -1
        # congelamos posición
        rx0, ry0, _ = self.get_relative_position()

        while not rospy.is_shutdown():
            # error yaw
            current_relative_yaw = self.normalize_angle(self.current_yaw - yaw0)
            error_yaw = self.normalize_angle(target - current_relative_yaw)

            error_yaw = self.normalize_angle(self.current_yaw - yaw0)

            if abs(error_yaw) < math.radians(2): # Menor a 2 grados (que se convierte en radianes)
                break

            # corregir drift XY
            rx, ry, _ = self.get_relative_position()

            error_x = rx0 - rx
            error_y = ry0 - ry

            kp_v = 0.8
            kp_wz = 1.2
            vx = max(min(kp_v * error_x, 0.2), -0.2)
            vy = max(min(kp_v * error_y, 0.2), -0.2)
            wz = max(min(kp_wz * error_yaw, 0.4), -0.4)

            self.send_velocity(vx, vy, 0, wz)
            self.rate.sleep()

        self.stop()

    
    # =====================================================
    # CONTROL DE ALTURA
    # =====================================================

    def maintain_altitude(self, target_altitude, kp=0.8):
        error = target_altitude - self.current_altitude
        return max(min(kp * error, 0.3), -0.3)

    # =====================================================
    # CONTROL DE ALTURA A METROS RELATIVOS
    # =====================================================
    def go_to_height(self, relative_height):

        target = self.initial_altitude + relative_height

        while not rospy.is_shutdown():
            error = target - self.current_altitude

            if abs(error) < 0.05:
                break
            kp = 0.8
            vz = max(min(kp * error, 0.3), -0.3)
            self.send_velocity(0, 0, vz)
            self.rate.sleep()
  
        self.stop()

    # =====================================================
    # HOVER
    # =====================================================
    #def activate_hover(self):
    #    rx, ry, rz = self.get_relative_position()
    #    self.target_position = (rx, ry, rz)
    #    self.hover_active = True
    #    self.navigation_active = False
    #    rospy.loginfo("Target:")
    #    rospy.loginfo(self.target_position)

    #def deactivate_hover(self):
    #    self.target_position = None
    #    self.hover_active = False
    #    self.navigation_active = False
    """
    # =====================================================
    # =====================================================
    # =====================================================
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



    # =====================================================
    # UPDATE (CONTROL CASCADA PROFESIONAL)
    # =====================================================
    def update(self):

        if self.emergency_stop or not self.navigation_active:
            return

        rx, ry, rz = self.get_relative_position()

        vx = vy = vz = wz = 0

        # ================= XY PD =================
        if self.target_x is not None:

            self.error_x = self.target_x - rx
            self.error_y = self.target_y - ry

            d_error_x = self.error_x - self.prev_error_x
            d_error_y = self.error_y - self.prev_error_y

            vx = self.kp_xy * self.error_x + self.kd_xy * d_error_x
            vy = self.kp_xy * self.error_y + self.kd_xy * d_error_y

            self.prev_error_x = self.error_x
            self.prev_error_y = self.error_y

            if abs(self.error_x) < 0.1 and abs(self.error_y) < 0.1:
                vx = 0
                vy = 0

        # ================= Z =================
        if self.target_z is not None:

            self.error_z = self.target_z - rz
            vz = self.kp_z * self.error_z

            if abs(self.error_z) < 0.05:
                vz = 0

        # ================= YAW =================
        if self.target_yaw is not None:

            self.error_yaw = self.normalize_angle(self.target_yaw - self.current_yaw)
            wz = self.kp_yaw * self.error_yaw

            if abs(self.error_yaw) < math.radians(2):
                wz = 0

        # ================= CHECK FIN =================
        done_xy = (self.target_x is None) or (abs(self.target_x - rx) < 0.03)
        done_z = (self.target_z is None) or (abs(self.target_z - rz) < 0.05)
        done_yaw = (self.target_yaw is None) or (abs(self.normalize_angle(self.target_yaw - self.current_yaw)) < math.radians(2))

        if done_xy and done_z and done_yaw:
            rospy.loginfo("✔ Target alcanzado")
            rospy.loginfo(self.get_relative_position())
            self.navigation_active = False
            self.stop()
            return
        
        rospy.loginfo(f"Error x: {self.error_x:.3f}")

        self.send_velocity(vx, vy, vz, wz)
