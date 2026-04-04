# Primera prueba: detección de ArUco con la cámara del dron

## Objetivo
Comprobar que la cámara del Bebop publica en `/bebop/image_raw` y que el nodo `test_aruco_bebop_camera.py` detecta correctamente el marcador ArUco.

## Terminal 1 — levantar cámara del Bebop
```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch bebop_driver bebop_node.launch
```

## Terminal 2 — verificar que la cámara publique
```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rostopic list | grep bebop
rostopic hz /bebop/image_raw
```

## Terminal 3 — correr el test de detección ArUco
```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rosrun bebop_tmr test_aruco_bebop_camera.py
```

## Resultado esperado
- En la Terminal 1 el driver del Bebop inicia sin errores.
- En la Terminal 2 aparece el topic `/bebop/image_raw` y muestra frecuencia.
- En la Terminal 3 se abre una ventana con la imagen de la cámara del dron.
- Si el ArUco está frente a la cámara, el nodo lo detecta y dibuja el contorno en pantalla.