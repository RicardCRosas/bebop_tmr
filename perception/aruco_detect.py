def image_callback(msg):
    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(msg, "bgr8")

    # Diccionario de parámetros para el detector de ArUco
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50) # Usar un diccionario de marcadores ArUco específico, en este caso, el de 4x4 con 50 marcadores posibles.
    parameters = aruco.DetectorParameters_create() # Crear un objeto de parámetros para configurar el

    # Detectar los marcadores ArUco en la imagen
    corners, ids, _ = aruco.detectMarkers(frame, aruco_dict, parameters=parameters) # Detectar los marcadores ArUco en la imagen utilizando el diccionario y los parámetros definidos. La función devuelve las esquinas de los marcadores detectados, sus IDs y otra información adicional que no se utiliza en este caso.

    if ids is not None:
        aruco.drawDetectedMarkers(frame, corners, ids)
        print("IDs detectados:", ids.flatten()) # Imprimir los IDs de los marcadores detectados en la consola.
    
    cv2.imshow("Aruco Detection", frame) # Mostrar la imagen con los marcadores detectados en una ventana de OpenCV.
    cv2.waitKey(1) # Esperar brevemente para permitir que la ventana se actualice y muestre la imagen procesada.

rospy.init_node('aruco_detector') # Inicializar el nodo de ROS con el nombre 'aruco_detector'.
sub = rospy.Subscriber('/bebop/image_raw', Image, image_callback) # Suscribirse al tópico de imágenes de la cámara del dron (en este caso, '/bebop/image_raw') y especificar que la función 'image_callback' se llame cada vez que se reciba un nuevo mensaje de imagen.
rospy.spin() # Mantener el nodo en ejecución para seguir recibiendo mensajes de imagen y procesándolos con la función de callback.