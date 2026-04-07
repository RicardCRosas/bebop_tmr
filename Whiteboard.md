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
- `mission_supervisor.py` tenga timeout de 45 segundos
- el ArUco esté visible frente a la cámara

---

## Recompilar
```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
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
roslaunch bebop_driver bebop_node.launch
```

## Terminal 2 — verificar imagen
```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rostopic hz /bebop/image_raw
```

## Terminal 3 — etapa 1: buscar y alinear
```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rosrun bebop_tmr mission_whiteboard_aruco.py _test_mode:=search_align _show_debug:=true
```

## Resultado esperado
- Detecta el ArUco
- Se alinea visualmente
- No falla
- Puedes abortar con `q` o `e`

---

## Terminal 3 — etapa 2: aproximación
```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rosrun bebop_tmr mission_whiteboard_aruco.py _test_mode:=approach _show_debug:=true
```

## Resultado esperado
- Detecta
- Se alinea
- Se acerca de forma segura
- No se pega demasiado al pizarrón

---

## Terminal 3 — etapa 3: punto de inicio del trazo
```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rosrun bebop_tmr mission_whiteboard_aruco.py _test_mode:=draw_start _show_debug:=true
```

## Resultado esperado
- Ya no apunta solo al centro del ArUco
- Se mueve al punto estimado para comenzar a pintar

---

## Terminal 3 — etapa 4: toque del pizarrón
```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rosrun bebop_tmr mission_whiteboard_aruco.py _test_mode:=touch _show_debug:=true
```

## Resultado esperado
- Avanza muy lento
- Confirma toque usando visión + odometría
- No empuja demasiado
- Si pierde el ArUco cerca del pizarrón, retrocede

---

## Terminal 3 — etapa 5: dibujar línea
```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rosrun bebop_tmr mission_whiteboard_aruco.py _test_mode:=draw _show_debug:=true
```

## Resultado esperado
- Toca el pizarrón
- Dibuja línea horizontal
- Retrocede después

---

## Terminal 3 — etapa 6: misión completa
```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rosrun bebop_tmr mission_whiteboard_aruco.py _test_mode:=full _show_debug:=true
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