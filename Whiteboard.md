# Pruebas de la misión Whiteboard ArUco

## Teclas de emergencia

Mientras corre la misión en la terminal:

* `q` = aterrizaje inmediato
* `e` = emergencia dura (`/bebop/reset`)

> Usa `q` si el dron sigue estable y quieres abortar.
> Usa `e` solo si hay riesgo real de choque o pérdida de control.

---

## Antes de empezar

Asegúrate de que:

* el Bebop esté encendido
* tu laptop esté conectada al WiFi del dron
* estás trabajando en `~/bebop_ws`
* el workspace ya está compilado
* el ArUco está visible frente a la cámara (ID 100)
* la misión tiene un timeout interno de 75 segundos
* el dron aterrizará automáticamente si se acerca demasiado al pizarrón (Safe Area)

---

## Recompilar

```bash
cd ~/bebop_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## Garantía de ejecución

Si el código no corre o ROS no encuentra los paquetes, ejecuta esto una sola vez.

### 1. Permisos de archivos

```bash
chmod +x ~/bebop_ws/src/bebop_tmr/missions/*.py
chmod +x ~/bebop_ws/src/bebop_tmr/perception/*.py
chmod +x ~/bebop_ws/src/bebop_tmr/bebop_core/*.py
```

### 2. Compilación y source

Asegúrate de estar en la raíz de tu workspace:

```bash
cd ~/bebop_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

### 3. Verificación rápida del paquete

```bash
rospack find bebop_tmr
```

Resultado esperado:

```text
/home/udlap/bebop_ws/src/bebop_tmr
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

> Esta terminal debe quedarse abierta.
> Si se queda mostrando `SUMMARY` o `PARAMETERS`, es normal.

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

* La cámara del dron publica en `/bebop/image_raw`
* Se abre una ventana
* El ArUco se detecta
* Aparecen `id`, `err_x`, `err_y`, `area`

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

---

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
cd ~/bebop_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rosrun bebop_tmr mission_whiteboard_aruco.py _test_mode:=search_align _show_debug:=true
```

### 3. Aterrizar

```bash
rostopic pub --once /bebop/land std_msgs/Empty "{}"
```

## Resultado esperado

* Detecta el ArUco
* Se alinea visualmente
* No falla
* Puedes abortar con `q` o `e`

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
cd ~/bebop_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rosrun bebop_tmr mission_whiteboard_aruco.py _test_mode:=approach _show_debug:=true
```

### 3. Aterrizar

```bash
rostopic pub --once /bebop/land std_msgs/Empty "{}"
```

## Resultado esperado

* Detecta el ArUco
* Se alinea
* Se acerca de forma segura y muy lenta
* Si se pega demasiado al pizarrón (Safe Area), aterriza inmediatamente por seguridad

---

## Terminal 3 — etapa 3: punto de inicio del trazo

### 1. Despegar

```bash
cd ~/bebop_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rostopic pub --once /bebop/takeoff std_msgs/Empty "{}"
```

### 2. Ejecutar etapa

```bash
cd ~/bebop_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rosrun bebop_tmr mission_whiteboard_aruco.py _test_mode:=draw_start _show_debug:=true
```

### 3. Aterrizar

```bash
rostopic pub --once /bebop/land std_msgs/Empty "{}"
```

## Resultado esperado

* Ya no apunta solo al centro del ArUco
* Se mueve al punto estimado para comenzar a pintar

---

## Terminal 3 — etapa 4: toque del pizarrón

### 1. Despegar

```bash
cd ~/bebop_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rostopic pub --once /bebop/takeoff std_msgs/Empty "{}"
```

### 2. Ejecutar etapa

```bash
cd ~/bebop_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rosrun bebop_tmr mission_whiteboard_aruco.py _test_mode:=touch _show_debug:=true
```

### 3. Aterrizar

```bash
rostopic pub --once /bebop/land std_msgs/Empty "{}"
```

## Resultado esperado

* Avanza muy lento
* Confirma toque usando visión + odometría
* Si el área crece demasiado (riesgo de choque), aterriza inmediatamente
* Si pierde el ArUco cerca del pizarrón, retrocede o aterriza según la fase de seguridad

---

## Terminal 3 — etapa 5: dibujar línea

### 1. Despegar

```bash
cd ~/bebop_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rostopic pub --once /bebop/takeoff std_msgs/Empty "{}"
```

### 2. Ejecutar etapa

```bash
cd ~/bebop_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rosrun bebop_tmr mission_whiteboard_aruco.py _test_mode:=draw _show_debug:=true
```

### 3. Aterrizar

```bash
rostopic pub --once /bebop/land std_msgs/Empty "{}"
```

## Resultado esperado

* Toca el pizarrón
* Dibuja línea horizontal
* Retrocede después

---

## Terminal 3 — etapa 6: misión completa

### 1. Despegar

```bash
cd ~/bebop_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rostopic pub --once /bebop/takeoff std_msgs/Empty "{}"
```

### 2. Ejecutar etapa

```bash
cd ~/bebop_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
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
cd ~/bebop_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rostopic echo /bebop/cmd_vel
```

## Resultado esperado

* Ver cómo cambian los comandos al mover el ArUco
* Confirmar si la lógica está alineando, acercando o retrocediendo

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
4. `draw_start`
5. `touch`
6. `draw`
7. `full`

---

# Troubleshooting

## Error: `is neither a launch file in package [bebop_driver]`

Si ves este error, normalmente estabas intentando lanzar algo como:

```bash
roslaunch bebop_driver bebop_node.launch
```

No uses eso. Usa siempre:

```bash
roslaunch bebop_tmr bebop_node.launch
```

## Error: `Couldn't find executable named mission_whiteboard_aruco.py`

Verifica estas tres cosas:

```bash
cd ~/bebop_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
chmod +x ~/bebop_ws/src/bebop_tmr/missions/mission_whiteboard_aruco.py
```

Luego vuelve a probar:

```bash
rosrun bebop_tmr mission_whiteboard_aruco.py _test_mode:=search_align _show_debug:=true
```

## El dron no despega o no se conecta

* Verifica que estás conectado al WiFi del Bebop (`Bebop2-XXXX`)
* Verifica que el botón de emergencia del Bebop no esté parpadeando en rojo
* Si `roslaunch` falla con `ResourceNotFound`, asegúrate de haber corrido `source devel/setup.bash` en esa terminal
* Verifica que `/bebop/image_raw` realmente publique antes de intentar la misión

Comando útil:

```bash
rostopic hz /bebop/image_raw
```

## Verificar que ROS esté usando el workspace correcto

```bash
rospack find bebop_tmr
```

Debe apuntar a:

```text
/home/udlap/bebop_ws/src/bebop_tmr
```

Si apunta a otro lugar, estás compilando o ejecutando desde el workspace equivocado.

---

# Si algo sale mal

* `q` → aterrizaje inmediato
* `e` → emergencia dura

Y si necesitas parar el dron manualmente desde otra terminal:

## Aterrizaje inmediato

```bash
rostopic pub --once /bebop/land std_msgs/Empty "{}"
```

## Emergencia dura

```bash
rostopic pub --once /bebop/reset std_msgs/Empty "{}"
```
