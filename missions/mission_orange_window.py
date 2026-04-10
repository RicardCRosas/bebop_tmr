#!/usr/bin/env python3
# mission_orange_window.py

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Empty
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import os
import sys

# =====================================================
# FIX IMPORT PATH
# =====================================================

current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, '..')) #Todo esta parte es para arreglar el tema de imports, para que funcione tanto en simulador como en real sin tener que tocar nada
sys.path.append(project_root)

from control.bebop_teleop_controller import BebopMovements # Importar la clase de movimientos
from perception.window_orange_detector import BebopCameraProcessor # Importar la clase de procesamiento de cámara, que en este caso, es el componente de percepción específico para detectar la ventana naranja

# =====================================================
# MISSION CLASS
# ====================================================
class MissionOrangeWindow:

    def __init__(self):

        rospy.init_node('mission_orange_window') #inicia el nodo de ROS para esta misión
        
        # ========================
        # Publishers                #crea los publishers para interactuar con otros nodos, como el controlador de movimientos y el procesador de cámara
        # ========================
        self.pub_cmd = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1) # Publicador para enviar comandos de movimiento, twist porque incluye velocidades lineales y angulares
        self.pub_takeoff = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1) # Publicador para despegar, empty porque no necesita ningún dato, solo el mensaje de que se quiere despegar
        self.pub_land = rospy.Publisher('/bebop/land', Empty, queue_size=1) # Publicador para aterrizar, empty porque no necesita ningún dato, solo el mensaje de que se quiere aterrizar
        self.pub_camera = rospy.Publisher('/bebop/camera_control', Twist, queue_size=1) # Publicador para controlar la cámara, twist para la orientación de la cámara
        self.status_pub = rospy.Publisher('/mission/status', String, queue_size=1) # Publicador para enviar el estado de la misión, string para indicar si la misión está "in progress" o "done"

        # ========================
        # Movement + Vision
        # ========================

        # Ahora sí pasarlos, preparamos el módulo de movimientos, que es el que se va a encargar de traducir las órdenes de movimiento en comandos específicos para el dron, y el módulo de percepción, que es el que se va a encargar de procesar las imágenes de la cámara para detectar la ventana naranja y extraer la información relevante (como la posición del centro de la ventana en la imagen).
        self.movements = BebopMovements(
            self.pub_cmd, # Publicador para enviar comandos de movimiento
            self.pub_takeoff, # Publicador para despegar
            self.pub_land, # Publicador para aterrizar
            self.pub_camera # Publicador para controlar la cámara
        )

        # Perception module
        self.detector = BebopCameraProcessor()
        self.bridge = CvBridge() # Para convertir entre imágenes de ROS y OpenCV, el CvBridge es una herramienta que permite convertir los mensajes de imagen de ROS (que son del tipo sensor_msgs/Image) a formatos de imagen que OpenCV puede procesar (como arrays de NumPy). Esto es esencial para poder aplicar técnicas de visión por computadora a las imágenes capturadas por la cámara del dron.

        # ROS utilities
        self.image_sub = rospy.Subscriber(
            "/bebop/image_raw", # Subscriber para recibir las imágenes de la cámara del dron, se suscribe al topic de imagen raw del bebop, y cada vez que llega una imagen, se llama al método image_callback para procesarla
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2**24
        )


        # ========================
        # State Variables
        # ========================
        self.latest_image_msg = None   # SOLO guardamos mensaje más reciente, no una cola de imágenes, para evitar acumulación y retrasos
        self.latest_data = None # Últimos datos de detección (como posición del centro de la ventana), que se actualizan cada vez que se procesa una nueva imagen, y se utilizan en la lógica de control para decidir cómo mover el dron en función de dónde se encuentra la ventana en la imagen.
        self.last_detection_time = rospy.Time.now() #deteccniones frescas, no mayores a 0.3s

        self.finished = False #Indica si la misión ha terminado, para dejar de procesar imágenes y ejecutar la lógica de control una vez que se ha pasado la ventana naranja.
        self.center_tolerance = 25 #Valor de tolerancia en pixeles para considerar que el dron está alineado con la ventana, es decir, si el centro de la ventana está dentro de un rango de 25 píxeles a la izquierda o derecha del centro de la imagen, se considera que el dron está alineado y puede avanzar hacia adelante. Si el centro de la ventana está fuera de ese rango, el dron ajustará su posición moviéndose hacia la izquierda o derecha para intentar alinear con la ventana.
        self.forward_counter = 0 #Iniciamos el contador hacia delante en 0, y cada vez que el dron avance hacia delante se irá incrementando en 1, y si el dron tiene que ajustar su posición (movimiento lateral), se reiniciará a 0. Este contador se utiliza para determinar cuándo el dron ha avanzado lo suficiente hacia adelante como para considerar que ha pasado la ventana naranja. En este caso, si el contador supera un valor determinado (en este caso, 20), se considera que el dron ha pasado la ventana y se finaliza la misión.

        self.debug_image = None # Variable para almacenar la última imagen procesada con la información dibujada, que se muestra en una ventana de depuración para visualizar lo que el dron está "viendo" y cómo está interpretando la información de la cámara.


        rospy.loginfo("Mission Orange Window initialized") #Mensaje que indica la inicialización exitosa del nodo.

    # =====================================================
    # IMAGE CALLBACK (LIGERO - SIN PROCESAMIENTO)
    # =====================================================

    def image_callback(self, msg):
        if self.finished:
            return

        # 🔥 SOLO guardar la imagen más reciente
        self.latest_image_msg = msg


    # =====================================================
    # PROCESS IMAGE (FUERA DEL CALLBACK)
    # =====================================================

    def process_latest_image(self):

        if self.latest_image_msg is None:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(self.latest_image_msg, "bgr8") # Convertir el mensaje de imagen de ROS a formato OpenCV, utilizando el CvBridge para convertir el mensaje de imagen (que es del tipo sensor_msgs/Image) a un formato que OpenCV pueda procesar (en este caso, un array de NumPy con formato BGR de 8 bits por canal).
        except Exception as e:
            rospy.logerr(f"CvBridge error: {e}") #Se registra en caso de error en el Cvbridge, lo que podría ocurrir si el mensaje de imagen no se puede convertir correctamente, por ejemplo, si el formato de la imagen no es compatible o si hay algún problema con el mensaje recibido.
            return

        # Reducir resolución
        frame = cv2.resize(frame, (640, 360)) # Reducir la resolución de la imagen para acelerar el procesamiento, esto es especialmente útil para mejorar el rendimiento de la inferencia en tiempo real, ya que procesar imágenes de alta resolución puede ser más lento y consumir más recursos computacionales. Al reducir la resolución a 640x360 píxeles, se mantiene suficiente información visual para detectar la ventana naranja mientras se mejora la velocidad de procesamiento.

        # =========================
        # MEDIR DELAY DE CÁMARA
        # =========================

        msg_time = self.latest_image_msg.header.stamp.to_sec()
        now_time = rospy.Time.now().to_sec()
        camera_delay = now_time - msg_time

        # =========================
        # MEDIR INFERENCIA
        # =========================

        start_time = rospy.Time.now()

        processed_image, data = self.detector.process_image(frame)

        end_time = rospy.Time.now()

        inference_time = (end_time - start_time).to_sec()
        # Todo eso, es para calcular el tiempo que demora el detector en procesar la imagen, en segundos.

        # =========================
        # DIBUJAR INFO EN IMAGEN
        # =========================

        # Dibujar info en pantalla acerca del rendimiento y sincronización.
        cv2.putText(processed_image,
                    f"Inference: {inference_time*1000:.1f} ms",
                    (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2)

        cv2.putText(processed_image,
                    f"ROS time: {rospy.Time.now().to_sec():.2f}",
                    (20, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 0),
                    2)

        self.debug_image = processed_image

        self.latest_data = data
        self.last_detection_time = rospy.Time.now() #momento de la última inferencia válida.


    # =====================================================
    # IMAGE CALLBACK
    # =====================================================

#    def image_callback(self, msg):
#        if self.finished:
#            return
#        try:
#            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
#        except Exception as e:
#            rospy.logerr(f"CvBridge error: {e}")
#            return
         # Reduce resolution for faster inference
#        frame = cv2.resize(frame, (640, 360)) # (480, 270)
#        processed_image, data = self.detector.process_image(frame)
        # Store latest detection
#        self.latest_data = data
#        self.last_detection_time = rospy.Time.now()

        # Optional debug window
#        cv2.imshow("Orange Detection", processed_image)
#        cv2.waitKey(1)


    # =====================================================
    # CONTROL LOGIC
    # =====================================================
    def control_logic(self):

        if self.latest_data is None:
            return

        # If detection is too old → ignore. Timeout detección
        if (rospy.Time.now() - self.last_detection_time).to_sec() > 0.3:
            rospy.loginfo("Detection timeout → searching")
            self.movements.turn_left("automatic")
            self.forward_counter = 0
            return

        data = self.latest_data

        if not data["detected"]:
            rospy.loginfo("Searching window...")
            self.movements.turn_left("automatic")
            self.forward_counter = 0
            return

        cx = data["cx"] # Posición del centro de la ventana en el eje X.
        center_x = data["center_x"] # Centro de la imagen en el eje X

        # ========================
        # ALIGNMENT LOGIC
        # ========================
        # Align horizontally
        if cx < center_x - self.center_tolerance:
            rospy.loginfo("Adjusting left")
            self.movements.left("automatic")
            self.forward_counter = 0

        elif cx > center_x + self.center_tolerance:
            rospy.loginfo("Adjusting right")
            self.movements.right("automatic")
            self.forward_counter = 0

        else:
            rospy.loginfo("Aligned - moving forward")
            self.movements.forward("automatic")
            self.forward_counter += 1

        # Condition to consider window passed
        if self.forward_counter > 20:
            rospy.loginfo("Window passed!")
            self.finish_mission()

    # =====================================================
    # FINISH MISSION
    # =====================================================
    def finish_mission(self):
        self.movements.stop("automatic")
        self.status_pub.publish("done")
        self.finished = True
        rospy.loginfo("Mission Orange Window completed")

    # =====================================================
    # MAIN LOOP
    # =====================================================

    def run(self):

        rate = rospy.Rate(15)  # 🔥 15 Hz reales

        while not rospy.is_shutdown():

            if not self.finished:

                # 1️⃣ Procesar última imagen disponible
                self.process_latest_image()

                # 2️⃣ Ejecutar control
                self.control_logic()

                # Muestra la cámara
                if self.debug_image is not None:
                    cv2.imshow("Orange Detection", self.debug_image)
                    cv2.waitKey(1)

            rate.sleep()


# =====================================================
# MAIN
# =====================================================

if __name__ == "__main__":
    mission = MissionOrangeWindow()
    mission.run()