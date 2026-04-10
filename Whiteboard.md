# Pruebas de la misión Whiteboard ArUco

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
- el workspace ya esté compilado
- el ArUco esté visible frente a la cámara (ID 100)
- la misión tiene un timeout interno de 75 segundos (ajustado por velocidad lenta)
- el dron aterrizará automáticamente si se acerca demasiado al pizarrón (Safe Area)

---

## Recompilar
```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## Garantía de Ejecución (Reliability)
Si el código no corre o ROS no encuentra los paquetes, ejecuta esto una sola vez:

### 1. Permisos de archivos
```bash
chmod +x ~/catkin_ws/src/bebop_tmr/missions/*.py
chmod +x ~/catkin_ws/src/bebop_tmr/perception/*.py
chmod +x ~/catkin_ws/src/bebop_tmr/bebop_core/*.py
```

### 2. Compilación y Source
Asegúrate de estar en la raíz de tu workspace:
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

# PRUEBA 1 — Solo cámara del dron
## Terminal 1 — levantar cámara del Bebop
```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch bebop_tmr bebop_node.launch
```

## Terminal 2 — verificar que la cámara publique
```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rostopic list | grep bebop
rostopic hz /bebop/image_raw
```

## Terminal 3 — test de detección ArUco
```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rosrun bebop_tmr test_aruco_bebop_camera.py
```

## Resultado esperado
- La cámara del dron publica en `/bebop/image_raw`
- Se abre una ventana
- El ArUco se detecta
- Aparecen `id`, `err_x`, `err_y`, `area`

---

# PRUEBA 2 — Misión Whiteboard por etapas

## Terminal 1 — cámara del dron
```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch bebop_tmr bebop_node.launch
```

## Terminal 2 — verificar imagen
```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rostopic hz /bebop/image_raw
```

## Terminal 3 — etapa 1: buscar y alinear

### 1. Despegar
```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rostopic pub --once /bebop/takeoff std_msgs/Empty "{}"
```

### 2. Ejecutar etapa
```bash
rosrun bebop_tmr mission_whiteboard_aruco.py _test_mode:=search_align _show_debug:=true
```

### 3. Aterrizar
```bash
rostopic pub --once /bebop/land std_msgs/Empty "{}"
```

## Resultado esperado
- Detecta el ArUco
- Se alinea visualmente
- No falla
- Puedes abortar con `q` o `e`

---

## Terminal 3 — etapa 2: aproximación

### 1. Despegar
```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rostopic pub --once /bebop/takeoff std_msgs/Empty "{}"
```

### 2. Ejecutar etapa
```bash
rosrun bebop_tmr mission_whiteboard_aruco.py _test_mode:=approach _show_debug:=true
```

### 3. Aterrizar
```bash
rostopic pub --once /bebop/land std_msgs/Empty "{}"
```

## Resultado esperado
- Detecta
- Se alinea
- se acerca de forma segura y muy lenta
- si se pega demasiado al pizarrón (Safe Area), aterriza inmediatamente por seguridad

---

## Terminal 3 — etapa 3: punto de inicio del trazo

### 1. Despegar
```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rostopic pub --once /bebop/takeoff std_msgs/Empty "{}"
```

### 2. Ejecutar etapa
```bash
rosrun bebop_tmr mission_whiteboard_aruco.py _test_mode:=draw_start _show_debug:=true
```

### 3. Aterrizar
```bash
rostopic pub --once /bebop/land std_msgs/Empty "{}"
```

## Resultado esperado
- Ya no apunta solo al centro del ArUco
- Se mueve al punto estimado para comenzar a pintar

---

## Terminal 3 — etapa 4: toque del pizarrón

### 1. Despegar
```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rostopic pub --once /bebop/takeoff std_msgs/Empty "{}"
```

### 2. Ejecutar etapa
```bash
rosrun bebop_tmr mission_whiteboard_aruco.py _test_mode:=touch _show_debug:=true
```

### 3. Aterrizar
```bash
rostopic pub --once /bebop/land std_msgs/Empty "{}"
```

## Resultado esperado
- Avanza muy lento
- Confirma toque usando visión + odometría
- si el área crece demasiado (riesgo de choque), aterriza inmediatamente
- si pierde el ArUco cerca del pizarrón, retrocede o aterriza según la fase de seguridad

---

## Terminal 3 — etapa 5: dibujar línea

### 1. Despegar
```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rostopic pub --once /bebop/takeoff std_msgs/Empty "{}"
```

### 2. Ejecutar etapa
```bash
rosrun bebop_tmr mission_whiteboard_aruco.py _test_mode:=draw _show_debug:=true
```

### 3. Aterrizar
```bash
rostopic pub --once /bebop/land std_msgs/Empty "{}"
```

## Resultado esperado
- Toca el pizarrón
- Dibuja línea horizontal
- Retrocede después

---

## Terminal 3 — etapa 6: misión completa

### 1. Despegar
```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rostopic pub --once /bebop/takeoff std_msgs/Empty "{}"
```

### 2. Ejecutar etapa
```bash
rosrun bebop_tmr mission_whiteboard_aruco.py _test_mode:=full _show_debug:=true
```

### 3. Aterrizar
```bash
rostopic pub --once /bebop/land std_msgs/Empty "{}"
```

## Resultado esperado
La misión completa debe hacer:

1. ajustar cámara
2. buscar ArUco
3. alinearse
4. aproximarse sin chocar
5. llegar al punto de inicio del trazo
6. confirmar toque del pizarrón
7. dibujar la línea
8. retroceder
9. girar 90° a la derecha
10. aterrizar

---

# PRUEBA 3 — Ver comandos del dron
## Terminal extra opcional
```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rostopic echo /bebop/cmd_vel
```

## Resultado esperado
- Ver cómo cambian los comandos al mover el ArUco
- Confirmar si la lógica está alineando, acercando o retrocediendo

---

# PRUEBA 4 — Ver imagen directa del dron
## Terminal extra opcional
```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rqt_image_view
```

Luego seleccionar:

```text
/bebop/image_raw
```

---

# Orden recomendado para probar ahorita
1. `test_aruco_bebop_camera.py`
2. `search_align`
3. `approach`
4. `draw_start`
5. `touch`
6. `draw`
7. `full`

---

# Troubleshooting
## Error: `is neither a launch file in package [bebop_driver]`
Si ves este error, es porque estás usando `roslaunch bebop_driver`. Usa siempre `roslaunch bebop_tmr bebop_node.launch` ya que el archivo está en tu propio paquete y configurado específicamente para este proyecto.

## El dron no despega o no se conecta
- Verifica que estés conectado al WiFi del Bebop (Bebop2-XXXX).
- Verifica que el botón de emergencia del Bebop (atrás) no esté parpadeando en rojo. Si parpadea, presiónalo una vez para resetear.
- Si `roslaunch` falla con "ResourceNotFound", asegúrate de haber corrido `source devel/setup.bash` en esa terminal.

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