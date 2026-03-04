#!/usr/bin/env python3
# teleop_node_2.py

# Control manual + disparador de misiones (Supervisor) de un dron Parrot Bebop mediante teclado y ROS.

import os
import sys
import rospy
import select
import termios
import tty
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

# Importa la clase BebopMovements desde la carpeta "scripts/control"
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, '..'))
sys.path.append(project_root)
from control.bebop_teleop_controller import BebopMovements
from control.bebop_advanced_controller_2 import BebopAdvancedController
from bebop_core.mission_supervisor_2 import MissionSupervisor

# ------------------------------
# DICCIONARIOS DE TECLAS
# ------------------------------
# Relaciona teclas con vectores de movimiento (x, y, z, rotación angular)
# Los valores son del % de la vel maxima, osea 1=100%
moveBindings = {
    'w': (1, 0, 0, 0),    # Adelante
    'a': (0, 1, 0, 0),    # Izquierda
    'd': (0, -1, 0, 0),   # Derecha
    's': (-1, 0, 0, 0),   # Atrás
    'q': (0, 0, 0, 1),    # Girar izquierda
    'e': (0, 0, 0, -1),   # Girar derecha
    '+': (0, 0, 1, 0),    # Subir
    '-': (0, 0, -1, 0),   # Bajar
}

# Controles para mover la cámara del dron
cameraBindings = {
    'i': (0, 5),    # Inclinación hacia arriba (tilt)
    'k': (0, -5),   # Inclinación hacia abajo
    'j': (-5, 0),   # Pan izquierda
    'l': (5, 0),    # Pan derecha
}

# Mensaje de ayuda que se muestra en consola
msg = """
---------------------------
Manual Control:
   q    w   e       +: up
   a        d       -: down
        s   
---------------------------
Land: 2
---------------------------
Start Mission_square_1: y
Start Mission_orange_window: o
Start Teleop: t
Emergency: x
---------------------------
Cam control:
---------------------------
   j    i    l       
        k    
---------------------------
CTRL-C to exit and land.
"""

# ===========================================================
# CLASE TELEOP
# ===========================================================

class BebopTeleop:
    def __init__(self):
        # Inicializa el nodo ROS
        rospy.init_node('teleop_control')

        # Publicadores para los distintos comandos del dron
        self.pub_cmd_vel = rospy.Publisher('bebop/cmd_vel', Twist, queue_size=1)          # Movimiento
        self.pub_takeoff = rospy.Publisher('bebop/takeoff', Empty, queue_size=10) # Despegue
        self.pub_land = rospy.Publisher('bebop/land', Empty, queue_size=10)       # Aterrizaje
        self.pub_camera = rospy.Publisher('bebop/camera_control', Twist, queue_size=1) # Cámara

        # Movements (bajo nivel)
        self.teleop_movements = BebopMovements(
            self.pub_cmd_vel,
            self.pub_takeoff,
            self.pub_land,
            self.pub_camera
        )

        # Crea objeto para manejar movimientos de aterrizaje, despegue y los movimientos automaticos 
        self.missions_movements = BebopAdvancedController(
            self.pub_cmd_vel,
            self.pub_takeoff,
            self.pub_land,
            self.pub_camera
        )
        # Supervisor (alto nivel)
        self.supervisor = MissionSupervisor(self.missions_movements)  #(self.teleop_movements)
        
        # Guarda configuración del teclado (para restaurarla luego)
        self.settings = termios.tcgetattr(sys.stdin)

        # Posiciona la cámara al iniciar
        self.init_camera_position()

        print(msg)

    # ----------------------------------------------------------
    # Inicializa la cámara mirando hacia abajo, y luego ajusta
    # ligeramente el ángulo para visualizar al frente.
    # ----------------------------------------------------------
    def init_camera_position(self):
        self.missions_movements.camera_tilt(-90)  # Mira totalmente hacia abajo
        rospy.sleep(1)
        self.missions_movements.camera_tilt(0)   # Ajusta para ver hacia el frente, antes era 10
        rospy.sleep(1)
        rospy.loginfo("Camera Initialized")

    # ----------------------------------------------------------
    # Lee una tecla sin esperar ENTER (modo raw del teclado)
    # ----------------------------------------------------------
    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        try:
            key = sys.stdin.read(1)
        except Exception:
            key = ''
        finally:
            # Restaura configuración normal del teclado
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    # ----------------------------------------------------------
    # BUCLE PRINCIPAL: lee las teclas y actúa según el modo
    # ----------------------------------------------------------
    def run(self):
        rate = rospy.Rate(30)  # 20 Hz

        while not rospy.is_shutdown():
            key = self.getKey()

            # INICIAR MISIÓN
            if key == 'y':
                print("\n--- Starting Mission Square---")
                self.supervisor.start_mission("mission_square_1") #orange_window
            elif key == 'u':
                self.supervisor.start_mission("mission_square_2")
            elif key == 'p':
                self.supervisor.start_mission("mission_square")
            elif key == 'o':
                self.supervisor.start_mission("mission_orange_window_modified")
            elif key == 'm':
                self.supervisor.start_mission("test_mission")
            
            # Activar Teleoperado
            elif key == 't':
                print("\nTELEOP")
                self.supervisor.start_teleop()
            # EMERGENCIA            
            elif key == 'x':
                print("\n--- Emergency stop ---")
                self.supervisor.emergency()

            # Ctrl+C → aterriza y sale
            elif key == '\x03':
                print("\nLanding before exiting...")
                self.supervisor.emergency()
                self.missions_movements.land()
                rospy.signal_shutdown("\nEnded by Ctrl-C")
                break

            # CONTROL TELEOPERADO (SOLO SI NO HAY MISIÓN ACTIVA)
            elif self.supervisor.is_teleop_active():
                if key == '1':
                    self.supervisor.start_teleop()
                    print("\nTakeoff → TELEOP")

                elif key == '2':
                    self.supervisor.request_landing()
                    print("\nLanding requested")

                elif key in moveBindings:
                    # Publica el vector Twist para moverse con los valores obtenidos de moveBindings
                    x, y, z, th = moveBindings[key]
                    self.teleop_movements.twist.linear.x = x
                    self.teleop_movements.twist.linear.y = y
                    self.teleop_movements.twist.linear.z = z
                    self.teleop_movements.twist.angular.z = th
                    self.pub_cmd_vel.publish(self.teleop_movements.twist)

                elif key in cameraBindings:
                    # Control manual de la cámara
                    pan, tilt = cameraBindings[key]
                    if pan != 0:
                        self.teleop_movements.camera_pan(pan)
                    if tilt != 0:
                        self.teleop_movements.camera_tilt(tilt)
                    print(f'\nCamera control: pan {pan}°, tilt {tilt}°')

                else:
                    # Si se presiona una tecla desconocida → detiene el dron en ese punto, mas no lo aterriza
                    self.teleop_movements.reset_twist()
                    self.pub_cmd_vel.publish(self.teleop_movements.twist)

            else:
                pass  # Si está en automático, no se hace nada con teclas

            # ACTUALIZAR SUPERVISOR
            #self.supervisor.update()

            rate.sleep()

# ===========================================================
# EJECUCIÓN PRINCIPAL
# ===========================================================
if __name__ == "__main__":
    teleop = BebopTeleop()
    try:
        teleop.run()
    except rospy.ROSInterruptException:
        pass