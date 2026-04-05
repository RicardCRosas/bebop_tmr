# Pruebas de la misión Whiteboard ArUco — Modo B completo

## Teclas de emergencia
Mientras corre la misión en la terminal:

- `q` = aterrizaje inmediato
- `e` = emergencia dura (`/bebop/reset`)

> Usar `q` si el dron sigue estable y quieres abortar.  
> Usar `e` solo si hay riesgo real de choque o pérdida de control.

---

## Antes de empezar
Asegúrate de que:
- el Bebop esté encendido
- tu laptop esté conectada a la red del dron
- el workspace `bebop_ws` esté compilado
- el workspace `catkin_ws` esté compilado
- el ArUco esté visible frente a la cámara
- tengas suficiente espacio libre frente al dron
- el dron esté estable antes de empezar cada etapa

---

## Recompilar `catkin_ws`
```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

---

# PRUEBA 1 — Solo cámara del dron

## Terminal 1 — levantar la cámara del Bebop
```bash
cd ~
source /opt/ros/noetic/setup.bash
source ~/bebop_ws/devel/setup.bash
rospack find bebop_driver
roslaunch bebop_driver bebop_node.launch
```

## Terminal 2 — verificar que la cámara publique
```bash
cd ~
source /opt/ros/noetic/setup.bash
source ~/bebop_ws/devel/setup.bash
rostopic list | grep bebop
rostopic hz /bebop/image_raw
```

## Terminal 3 — test de detección ArUco
```bash
cd ~
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_PACKAGE_PATH=$HOME/catkin_ws/src:$HOME/bebop_ws/src:/opt/ros/noetic/share
rosrun bebop_tmr test_aruco_bebop_camera.py
```

---

# PRUEBA 2 — Misión por etapas en vuelo

## Terminal 1 — levantar la cámara del Bebop
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

## Terminal 3 — despegue manual
```bash
cd ~
source /opt/ros/noetic/setup.bash
source ~/bebop_ws/devel/setup.bash
rostopic pub --once /bebop/takeoff std_msgs/Empty "{}"
```

> Esperar unos segundos a que el dron suba y se estabilice antes de correr la etapa.

---

## Etapa 1 — buscar y alinear
### Terminal 4
```bash
cd ~
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_PACKAGE_PATH=$HOME/catkin_ws/src:$HOME/bebop_ws/src:/opt/ros/noetic/share
rosrun bebop_tmr mission_whiteboard_aruco.py _test_mode:=search_align _show_debug:=true
```

### Resultado esperado
- Detecta el ArUco
- Se alinea al centro
- Corrige orientación frente al pizarrón
- NO se aproxima todavía

### Al terminar — aterrizaje manual
```bash
rostopic pub --once /bebop/land std_msgs/Empty "{}"
```

---

## Etapa 2 — aproximación segura
### Terminal 3 — despegar otra vez
```bash
cd ~
source /opt/ros/noetic/setup.bash
source ~/bebop_ws/devel/setup.bash
rostopic pub --once /bebop/takeoff std_msgs/Empty "{}"
```

### Terminal 4
```bash
cd ~
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_PACKAGE_PATH=$HOME/catkin_ws/src:$HOME/bebop_ws/src:/opt/ros/noetic/share
rosrun bebop_tmr mission_whiteboard_aruco.py _test_mode:=approach _show_debug:=true
```

### Resultado esperado
- Se alinea
- Se aproxima MUY lento
- Usa odometría para detenerse
- NO choca
- NO se acerca demasiado

### Al terminar — aterrizaje manual
```bash
rostopic pub --once /bebop/land std_msgs/Empty "{}"
```

---

## Etapa 3 — pre-dibujo
### Terminal 3 — despegar otra vez
```bash
cd ~
source /opt/ros/noetic/setup.bash
source ~/bebop_ws/devel/setup.bash
rostopic pub --once /bebop/takeoff std_msgs/Empty "{}"
```

### Terminal 4
```bash
cd ~
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_PACKAGE_PATH=$HOME/catkin_ws/src:$HOME/bebop_ws/src:/opt/ros/noetic/share
rosrun bebop_tmr mission_whiteboard_aruco.py _test_mode:=pre_draw _show_debug:=true
```

### Resultado esperado
- Se alinea
- Hace la aproximación segura
- Se acerca un poco más
- Se detiene listo para dibujar

### Al terminar — aterrizaje manual
```bash
rostopic pub --once /bebop/land std_msgs/Empty "{}"
```

---

## Etapa 4 — dibujar
### Terminal 3 — despegar otra vez
```bash
cd ~
source /opt/ros/noetic/setup.bash
source ~/bebop_ws/devel/setup.bash
rostopic pub --once /bebop/takeoff std_msgs/Empty "{}"
```

### Terminal 4
```bash
cd ~
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_PACKAGE_PATH=$HOME/catkin_ws/src:$HOME/bebop_ws/src:/opt/ros/noetic/share
rosrun bebop_tmr mission_whiteboard_aruco.py _test_mode:=draw _show_debug:=true
```

### Resultado esperado
- Se alinea
- Se aproxima seguro
- Hace pre-dibujo
- Traza la línea
- Se queda estable

### Al terminar — aterrizaje manual
```bash
rostopic pub --once /bebop/land std_msgs/Empty "{}"
```

---

## Etapa 5 — misión completa
### Terminal 3 — despegar otra vez
```bash
cd ~
source /opt/ros/noetic/setup.bash
source ~/bebop_ws/devel/setup.bash
rostopic pub --once /bebop/takeoff std_msgs/Empty "{}"
```

### Terminal 4
```bash
cd ~
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_PACKAGE_PATH=$HOME/catkin_ws/src:$HOME/bebop_ws/src:/opt/ros/noetic/share
rosrun bebop_tmr mission_whiteboard_aruco.py _test_mode:=full _show_debug:=true
```

### Resultado esperado
La misión completa debe hacer:

1. ajustar cámara
2. buscar ArUco
3. alinearse al pizarrón
4. aproximarse a distancia segura
5. acercarse un poco más
6. dibujar la línea
7. separarse del pizarrón
8. aterrizar

---

# PRUEBA 3 — Ver comandos del dron

## Terminal extra opcional
```bash
cd ~
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_PACKAGE_PATH=$HOME/catkin_ws/src:$HOME/bebop_ws/src:/opt/ros/noetic/share
rostopic echo /bebop/cmd_vel
```

---

# PRUEBA 4 — Ver imagen directa del dron

## Terminal extra opcional
```bash
cd ~
source /opt/ros/noetic/setup.bash
source ~/bebop_ws/devel/setup.bash
rqt_image_view
```

Luego seleccionar:

```text
/bebop/image_raw
```

---

# Orden recomendado
1. `test_aruco_bebop_camera.py`
2. `search_align`
3. `approach`
4. `pre_draw`
5. `draw`
6. `full`

---

# Si algo sale mal
- `q` → aterrizaje inmediato
- `e` → emergencia dura

Y si necesitas parar el dron manualmente desde otra terminal:

## Aterrizaje inmediato
```bash
rostopic pub --once /bebop/land std_msgs/Empty "{}"
```

## Emergencia dura
```bash
rostopic pub --once /bebop/reset std_msgs/Empty "{}"
```