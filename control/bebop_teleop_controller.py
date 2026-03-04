#!/usr/bin/env python3
# bebop_teleop_controller.py
# Clase para controlar movimientos básicos del dron Parrot Bebop usando ROS para el teleop

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

# Las velocidades son tienen estos rangos [-1,1]
# Los valores >1 o <-1 se saturan automáticamente y evitan por ende esos valores máximos en lugar del que sobrepasa.
# Esto es % de la velocidad maxima del dron

# linear.x   # Avance y retroceso (x positivo adelante)
# linear.y   # Movimiento lateral (y positivo izquierda)
# linear.z   # Subir o bajar (z positivo arriba)
# angular.x  # Rotación sobre eje X (rara vez usada) 
# angular.y  # Rotación sobre eje Y (rara vez usada)
# angular.z  # Giro (yaw) (positivo giro izquierda)

# Tiempo base de duracion de cada comando (en segundos)
sleep = 0.5

class BebopMovements:
    
    def __init__(self, pub_cmd_vel, pub_takeoff, pub_land, pub_camera):
        """
        Inicializa la clase con los publicadores necesarios.
        - pub: publisher para enviar comandos de velocidad (Twist)
        - pub_takeoff: publisher para el despegue (Empty)
        - pub_land: publisher para el aterrizaje (Empty)
        - pub_camera: publisher para el control de cámara (Twist)
        """
        self.pub_cmd_vel = pub_cmd_vel
        self.pub_takeoff = pub_takeoff
        self.pub_land = pub_land
        self.pub_camera = pub_camera
        self.twist = Twist()  # Mensaje de movimiento base (velocidades lineales y angulares)

    
    # ---------------------------
    # Despegue, ya sea que esté en modo automatico o se active desde el modo teleoperado
    # ---------------------------
    def initial_takeoff(self, mode_flag):
        # Solo se ejecuta si el modo es válido
        if mode_flag not in ['automatic', 'teleop'] or rospy.is_shutdown():
            print('\n Invalid state')
            return
        
        rospy.sleep(1)
        print('\n Taking off...')
        
        self.pub_takeoff.publish(Empty()) # Publica mensaje de despegue
        rospy.sleep(3) # Espera a que el dron despegue completamente

        # Realiza pequeños movimientos automáticos tras despegar

        self.up(mode_flag)
        self.up(mode_flag)    # Para que suba a la altura del gate con tiempos
        self.up(mode_flag)

        # gira un poco y creo eso hace que no se empiece a mover de forma autonoma en lo que aun no ajusta la camara. FAKE
        #self.turn_right(mode_flag) 
        #self.turn_left(mode_flag) 
        rospy.sleep(1)
        self.reset_twist() # Detiene movimiento 



    # ---------------------------
    # Aterrizaje, ya sea que esté en modo automatico o se active desde el modo teleoperado.
    # ---------------------------
    def landing(self, mode_flag):
        if mode_flag not in ['automatic', 'teleop'] or rospy.is_shutdown():
            print('\n Invalid state')
            return
        
        self.reset_twist() # Detiene movimiento antes de aterrizar
        print('\n Landing...')
        rospy.sleep(sleep)

        # Envía mensaje de velocidad cero que previamente con reset_twist se asigno, de igual forma se hace dentro de reset_twist
        self.pub_cmd_vel.publish(self.twist)
        rospy.sleep(sleep)

        # Publica mensaje Empty para iniciar aterrizaje
        self.pub_land.publish(Empty())
        
        print('\n Land done')
        self.reset_twist()


    # ---------------------------
    # DETENER MOVIMIENTO
    # ---------------------------
    def reset_twist(self):
        """
        Reinicia todos los componentes del Twist a 0
        para detener cualquier movimiento activo del dron.
        """
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0

        # Publica el comando de velocidad cero
        self.pub_cmd_vel.publish(self.twist) 
        rospy.sleep(sleep)
    
    def publish_twist(self):
        self.pub_cmd_vel.publish(self.twist)


# ------------------------------------------ MOVIMIENTOS BÁSICOS DEL DRON ------------------------------------------

    # Solo se ejecutan en modo automático

    def forward(self, mode_flag):
        #self en método -> Siempre se pasa automáticamente cuando llamas obj.metodo(...), 
        # por lo que al usarlo solo colocas el resto de argumentos
        if mode_flag != 'automatic' or rospy.is_shutdown():
            print('\n Movement interrupted')
            return
        
        rospy.sleep(sleep)
        print('\n Going forward...')

        # Movimiento hacia adelante (eje x positivo) con 100% de velocidad 
        self.twist.linear.x = 1
        self.pub_cmd_vel.publish(self.twist)
        rospy.sleep(sleep)

        #self.twist.linear.x = 1       
        #self.pub.publish(self.twist)
        #rospy.sleep(sleep)

        #self.twist.linear.x = 1       
        #self.pub.publish(self.twist)
        #rospy.sleep(sleep)

        # All these for passing the windows
        # Debido a que a reconocer una ventana avanzaba y como estaban alejadas las ventanas 
        # avanzaba varias veces. y al final hacia un pequeño giro porque al estar en un circulo buscaban
        # alinearse con la posicion de la siguiente ventana e intentar reconocerla.

        # self.twist.linear.x = 1       
        # self.pub.publish(self.twist)
        # rospy.sleep(sleep)
        # self.twist.linear.x = 1
        # self.pub.publish(self.twist)
        # rospy.sleep(sleep)
        # self.twist.linear.x = 1
        # self.pub.publish(self.twist)
        # rospy.sleep(sleep)
        # self.twist.linear.x = 1
        # self.pub.publish(self.twist)
        # rospy.sleep(sleep)
        # self.twist.linear.z = 0.2
        self.pub_cmd_vel.publish(self.twist)
        rospy.sleep(sleep)
        self.reset_twist()

    def left(self, mode_flag):
        if mode_flag != 'automatic' or rospy.is_shutdown():
            print('\n Movement interrupted')
            return
        rospy.sleep(sleep)
        print('\n Moving left...')

        # Movimiento lateral hacia la izquierda (y positivo)
        self.twist.linear.y = 0.6
        self.pub_cmd_vel.publish(self.twist)
        rospy.sleep(sleep)
        self.reset_twist()

    def right(self, mode_flag):
        if mode_flag != 'automatic' or rospy.is_shutdown():
            print('\n Movement interrupted')
            return
        rospy.sleep(sleep)
        print('\n Moving right...')

        # Movimiento lateral hacia la derecha (y negativo)
        self.twist.linear.y = -0.6
        self.pub_cmd_vel.publish(self.twist)
        rospy.sleep(sleep)
        self.reset_twist()

    def backwards(self, mode_flag):
        if mode_flag != 'automatic' or rospy.is_shutdown():
            print('\n Movement interrupted')
            return
        rospy.sleep(sleep)
        print('\n Going backwards...')

        # Movimiento hacia atrás (x negativo)
        self.twist.linear.x = -0.5
        self.pub_cmd_vel.publish(self.twist)
        rospy.sleep(sleep)
        self.reset_twist()

    def up(self, mode_flag):
        if mode_flag != 'automatic' or rospy.is_shutdown():
            print('\n Movement interrupted')
            return
        rospy.sleep(sleep)
        print('\n Going up...')

        # Sube (z positivo)
        self.twist.linear.z = 1
        self.pub_cmd_vel.publish(self.twist)
        rospy.sleep(sleep)
        self.reset_twist()

    def down(self, mode_flag):
        if mode_flag != 'automatic' or rospy.is_shutdown():
            print('\n Movement interrupted')
            return
        rospy.sleep(sleep)
        print('\n Going down...')

        # Baja (z negativo)
        self.twist.linear.z = -0.5
        self.pub_cmd_vel.publish(self.twist)
        rospy.sleep(sleep)
        self.reset_twist()

    # ROTACIONES
    def turn_left(self, mode_flag):
        if mode_flag != 'automatic' or rospy.is_shutdown():
            print('\n Movement interrupted')
            return
        rospy.sleep(sleep)
        print('\n Turning left...')

        # Gira sobre su eje hacia la izquierda (angular.z positivo)
        self.twist.angular.z = 0.5
        self.pub_cmd_vel.publish(self.twist)
        rospy.sleep(sleep)
        self.reset_twist()

    def turn_right(self, mode_flag):
        if mode_flag != 'automatic' or rospy.is_shutdown():
            print('\n Movement interrupted')
            return
        rospy.sleep(sleep)
        print('\n Turning right...')
        
        # Gira hacia la derecha (angular.z negativo)
        self.twist.angular.z = -0.3
        self.pub_cmd_vel.publish(self.twist)
        rospy.sleep(sleep)
        self.reset_twist()


# ------------------------------------------ MOVIMIENTOS BÁSICOS DE LA CAMARA ------------------------------------------
    def camera_pan(self, pan):
        """
        Controla el movimiento horizontal (pan) de la cámara.
        pan: valor angular (grados o radianes según configuración del dron). En este caso son grados
        """
        print(f'\n Adjusting camera pan: {pan} degrees...')
        camera_twist = Twist()
        camera_twist.angular.z = pan
        self.pub_camera.publish(camera_twist)
        rospy.sleep(sleep)

    def camera_tilt(self, tilt):
        """
        Controla el movimiento vertical (tilt) de la cámara.
        tilt: valor angular (grados o radianes según configuración del dron). En este caso son grados
        """
        print(f'\n Adjusting camera tilt: {tilt} degrees...')
        camera_twist = Twist()
        camera_twist.angular.y = tilt
        rospy.sleep(sleep)
        self.pub_camera.publish(camera_twist)
        rospy.sleep(sleep)


""" Teclas:

T - teleop
Y - mission mission_square_1.py
O - ventanas mission_orange_window_modified.py pero no está modificado
2 - aterrizar


"""

    