# Pruebas de la misión Whiteboard ArUco — Flujo estable

## Teclas de emergencia
Mientras corre la misión en la terminal:

- `q` = aterrizaje inmediato
- `e` = emergencia dura (`/bebop/reset`)

---

## Recompilar
```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

---

# Cámara y detección

## Terminal 1 — cámara del Bebop
```bash
cd ~
source /opt/ros/noetic/setup.bash
source ~/bebop_ws/devel/setup.bash
rospack find bebop_driver
roslaunch bebop_driver bebop_node.launch
```

## Terminal 2 — verificar imagen
```bash
cd ~
source /opt/ros/noetic/setup.bash
source ~/bebop_ws/devel/setup.bash
rostopic hz /bebop/image_raw
```

## Terminal 3 — test ArUco
```bash
cd ~
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_PACKAGE_PATH=$HOME/catkin_ws/src:$HOME/bebop_ws/src:/opt/ros/noetic/share
rosrun bebop_tmr test_aruco_bebop_camera.py
```

---

# Misión por etapas

## Despegue manual
```bash
cd ~
source /opt/ros/noetic/setup.bash
source ~/bebop_ws/devel/setup.bash
rostopic pub --once /bebop/takeoff std_msgs/Empty "{}"
```

Espera unos segundos a que el dron se estabilice.

---

## Etapa 1 — search_align
```bash
cd ~
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_PACKAGE_PATH=$HOME/catkin_ws/src:$HOME/bebop_ws/src:/opt/ros/noetic/share
rosrun bebop_tmr mission_whiteboard_aruco.py _test_mode:=search_align _show_debug:=true
```

### Resultado esperado
- Detecta el ArUco
- Se centra razonablemente
- Corrige orientación al pizarrón
- No intenta una perfección imposible
- No se acerca todavía

---

## Etapa 2 — approach
```bash
cd ~
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_PACKAGE_PATH=$HOME/catkin_ws/src:$HOME/bebop_ws/src:/opt/ros/noetic/share
rosrun bebop_tmr mission_whiteboard_aruco.py _test_mode:=approach _show_debug:=true
```

### Resultado esperado
- Se alinea
- Si se desalineó, corrige sin avanzar
- Se acerca muy lento
- Se detiene a distancia segura
- No choca

---

## Etapa 3 — pre_draw
```bash
cd ~
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_PACKAGE_PATH=$HOME/catkin_ws/src:$HOME/bebop_ws/src:/opt/ros/noetic/share
rosrun bebop_tmr mission_whiteboard_aruco.py _test_mode:=pre_draw _show_debug:=true
```

### Resultado esperado
- Hace search_align
- Hace approach
- Se acerca un poco más
- Se detiene listo para trazo

---

## Etapa 4 — draw
```bash
cd ~
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_PACKAGE_PATH=$HOME/catkin_ws/src:$HOME/bebop_ws/src:/opt/ros/noetic/share
rosrun bebop_tmr mission_whiteboard_aruco.py _test_mode:=draw _show_debug:=true
```

### Resultado esperado
- Hace todo lo anterior
- Traza la línea
- Se mantiene estable
- No se pega al pizarrón

---

## Etapa 5 — full
```bash
cd ~
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_PACKAGE_PATH=$HOME/catkin_ws/src:$HOME/bebop_ws/src:/opt/ros/noetic/share
rosrun bebop_tmr mission_whiteboard_aruco.py _test_mode:=full _show_debug:=true
```

### Resultado esperado
1. buscar ArUco
2. alinearse al pizarrón
3. aproximarse seguro
4. acercarse un poco más
5. dibujar
6. separarse
7. aterrizar

---

# Parar el dron manualmente

## Aterrizaje normal
```bash
rostopic pub --once /bebop/land std_msgs/Empty "{}"
```

## Emergencia
```bash
rostopic pub --once /bebop/reset std_msgs/Empty "{}"
```