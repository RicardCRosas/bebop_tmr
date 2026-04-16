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
- la misión tiene un timeout interno de 90 segundos (ajustado por velocidad lenta)
- el dron aterrizará automáticamente si se acerca demasiado al pizarrón (Safe Area)

---

## Recompilar
```bash
cd ~/bebop_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## Garantía de Ejecución (Reliability)
Si el código no corre o ROS no encuentra los paquetes, ejecuta esto una sola vez:

### 1. Permisos de archivos
```bash
chmod +x ~/bebop_ws/src/bebop_tmr/missions/*.py
chmod +x ~/bebop_ws/src/bebop_tmr/perception/*.py
chmod +x ~/bebop_ws/src/bebop_tmr/bebop_core/*.py
```

### 2. Compilación y Source
Asegúrate de estar en la raíz de tu workspace:
```bash
cd ~/bebop_ws
catkin_make
source devel/setup.bash
```

---

# PRUEBA 1 — Solo cámara del dron
## Terminal 1 — levantar cámara del Bebop
```bash
cd ~/bebop_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch bebop_tmr bebop_node.launch
```

## Terminal 2 — verificar que la cámara publique
```bash
cd ~/bebop_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rostopic list | grep bebop
rostopic hz /bebop/image_raw
```

## Terminal 3 — test de detección ArUco
```bash
cd ~/bebop_ws
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
cd ~/bebop_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch bebop_tmr bebop_node.launch
```

## Terminal 2 — verificar imagen
```bash
cd ~/bebop_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rostopic hz /bebop/image_raw
```

## Terminal 3 — etapa 1: buscar y alinear

### 1. Despegar
```bash
cd ~/bebop_ws
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
- Ajusta cámara a una posición
- Busca ArUco (se mantiene flotando sin rotar en yaw)
- Se alinea correctamente usando solo traslación (sin yaw)
- No falla
- Puedes abortar con `q` o `e`

---

## Terminal 3 — etapa 2: aproximación

### 1. Despegar
```bash
cd ~/bebop_ws
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
- Se aproxima de forma segura y lenta
- Una vez alcanzada la distancia segura frente al pizarrón, aterriza automáticamente finalizando la misión.

---

## Terminal 3 — misión completa

### 1. Despegar
```bash
cd ~/bebop_ws
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
La misión completa ahora consiste en:

1. ajustar cámara a una posición
2. buscar ArUco (sin rotar)
3. alinearse y orientarse con pura traslación (sin yaw)
4. aproximarse de forma segura
5. aterrizar una vez alcanzada la distancia segura

---

# PRUEBA 3 — Ver comandos del dron
## Terminal extra opcional
```bash
cd ~/bebop_ws
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
cd ~/bebop_ws
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
4. `full`

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