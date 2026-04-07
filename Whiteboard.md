# Misión Whiteboard — Guía de ejecución paso a paso

## Teclas de emergencia (activas mientras corre la misión)
| Tecla | Acción |
|-------|--------|
| `q` | Aterrizaje normal inmediato |
| `e` | Reset de emergencia (motores apagados en el aire — **solo si va a chocar**) |

---

## Verificaciones antes de cada prueba

```bash
# Confirmar que la odometría llega correctamente
rostopic echo /bebop/odom | head -20
```
Debes ver `position.x`, `position.y`, `position.z` cambiando al mover el drone.

```bash
# Confirmar imagen
rostopic hz /bebop/image_raw
```
Debe mostrar ~30 Hz.

```bash
# Confirmar que el ArUco es visible
rosrun bebop_tmr test_aruco_bebop_camera.py
```
Abre ventana con la cámara del drone. El ArUco debe aparecer recuadrado en verde.

---

## Flujo de estados

```
SET_CAMERA
   │  ajusta tilt de cámara 18°, espera 1.5 s
   ▼
SEARCH_ARUCO
   │  gira az=0.15 buscando ArUco ID=100
   │  espera 8 frames estables antes de avanzar
   ▼
ALIGN_YAW          ← solo az, sin traslación
   │  corrige orientación visual del drone respecto al pizarrón
   │  tolera ±0.18 rad — no busca perfección
   ▼
ALIGN_CENTER       ← solo ly o lz, UN eje a la vez, sin avanzar
   │  centra el ArUco en imagen (tolera ±60 px X, ±45 px Y)
   │  espera 0.6 s estable antes de continuar
   ▼
APPROACH_BOARD     ← solo lx cuando está alineado; pausa al corregir deriva
   │  avanza hasta que area >= approach_area_threshold (≈1 m del pizarrón)
   │  freno de emergencia si area >= area_abort_threshold (≈0.5 m)
   │  si pierde ArUco > 1.5 s → SAFE_RETREAT
   │
   │  [SAFE_RETREAT si pierde ArUco]
   │     retrocede 0.40 m con odometría → vuelve a SEARCH_ARUCO
   ▼
MOVE_TO_DRAW_START ← odometría pura, sin ArUco
   │  fase DOWN:  baja 0.15 m  (lz negativo)
   │  fase RIGHT: mueve 0.28 m a la derecha (ly negativo)
   ▼
TOUCH_BOARD        ← open-loop
   │  lx=0.07 durante 1.0 s para hacer contacto con el pizarrón
   ▼
DRAW_LINE          ← open-loop
   │  lx=0.02 (presión) + ly=-0.14 (trazo) durante 2.0 s
   ▼
BACK_OFF           ← odometría
   │  retrocede 0.55 m reales
   ▼
ROTATE_RIGHT_90    ← odometría yaw
   │  gira -90° con PD sobre /bebop/odom yaw
   ▼
DONE → publica "done"
```

---

## Preparación del entorno

### Terminal 1 — conectar y levantar el Bebop
```bash
cd ~
source /opt/ros/noetic/setup.bash
source ~/bebop_ws/devel/setup.bash
roslaunch bebop_driver bebop_node.launch
```
Esperar a ver `[bebop_driver] Connected` antes de continuar.

### Terminal 2 — compilar el paquete (solo si hubo cambios de código)
```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

---

## Pruebas por etapa (`test_mode`)

Siempre despega manualmente primero:
```bash
rostopic pub --once /bebop/takeoff std_msgs/Empty "{}"
```
Espera ~3 segundos a que el drone se estabilice.

---

### Etapa 1 — `search_align` (solo búsqueda y alineación)
```bash
cd ~
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_PACKAGE_PATH=$HOME/catkin_ws/src:$HOME/bebop_ws/src:/opt/ros/noetic/share
rosrun bebop_tmr mission_whiteboard_aruco.py _test_mode:=search_align _show_debug:=true
```

**Resultado esperado:**
- Ajusta cámara
- Gira buscando el ArUco
- Corrige yaw (orientación frente al pizarrón)
- Centra el ArUco en imagen
- Para y publica "done"
- **No avanza hacia el pizarrón**

**Si algo falla:**
- `ArUco no detectado` → revisar iluminación, distancia y diccionario (DICT_5X5_250, ID 100)
- `gira sin parar` → el ArUco no es visible para la cámara con el tilt actual; ajustar `_cam_tilt`
- `correcciones muy grandes` → aumentar `_tol_x` y `_tol_y`

---

### Etapa 2 — `approach` (se acerca al pizarrón)
```bash
rosrun bebop_tmr mission_whiteboard_aruco.py _test_mode:=approach _show_debug:=true
```

**Resultado esperado:**
- Hace todo lo de etapa 1
- Avanza recto hacia el pizarrón
- Se detiene a ~1 m (cuando el ArUco ocupa el área objetivo)
- **No choca**
- Para y publica "done"

**Si algo falla:**
- `choca` → reducir `_approach_area_threshold` (ej. 13000) o reducir `_approach_speed`
- `para demasiado lejos` → aumentar `_approach_area_threshold` (ej. 20000)
- `va en diagonal` → revisar que `APPROACH_BOARD` no manda lx y ly simultáneamente
- `pierde ArUco y va a SAFE_RETREAT` → aumentar `_approach_loss_timeout` (ej. 2.5)

**Calibración del umbral de área:**
Con `show_debug=true` verás el valor de `area=XXXX` en la imagen. Apunta el valor
cuando el drone esté a la distancia deseada (≈1 m del pizarrón) y úsalo como
`_approach_area_threshold`.

---

### Etapa 3 — `draw_start` (se posiciona en punto de inicio)
```bash
rosrun bebop_tmr mission_whiteboard_aruco.py _test_mode:=draw_start _show_debug:=true
```

**Resultado esperado:**
- Hace todo lo de etapa 2
- Baja 0.15 m (sin buscar el ArUco)
- Mueve 0.28 m a la derecha
- Para en la posición de inicio del trazo, abajo y a la derecha del ArUco
- Publica "done"

**Ajuste de posición:**
- Si el plumón queda demasiado lejos del pizarrón → aumentar `_draw_right` o reducir `_draw_down`
- Si tapa el ArUco → reducir `_draw_right`
- Los valores son en metros; ajustar de 0.05 en 0.05

---

### Etapa 4 — `touch` (hace contacto)
```bash
rosrun bebop_tmr mission_whiteboard_aruco.py _test_mode:=touch _show_debug:=true
```

**Resultado esperado:**
- Hace todo lo anterior
- Empuja suavemente hacia adelante (lx=0.07) durante 1 s
- El plumón hace contacto con el pizarrón
- Para y publica "done"

**Ajuste:**
- Si el plumón no llega → aumentar `_touch_time` (ej. 1.5)
- Si choca duro → reducir `_touch_speed` (ej. 0.05) o `_touch_time`

---

### Etapa 5 — `draw` (traza la línea)
```bash
rosrun bebop_tmr mission_whiteboard_aruco.py _test_mode:=draw _show_debug:=true
```

**Resultado esperado:**
- Hace todo lo anterior
- Se mueve lateralmente a la derecha durante 2 s con presión hacia adelante
- Se ve la línea en el pizarrón
- Se aleja del pizarrón
- Publica "done"

**Ajuste:**
- Línea muy corta → aumentar `_draw_time` o `_draw_lat_speed` (más negativo)
- Línea muy larga → reducir `_draw_time`
- No hace contacto → aumentar `_draw_fwd_bias` (ej. 0.03)

---

### Etapa 6 — `full` (misión completa con rotación)
```bash
rosrun bebop_tmr mission_whiteboard_aruco.py _test_mode:=full _show_debug:=true
```

**Resultado esperado:**
1. Ajusta cámara
2. Busca ArUco
3. Corrige yaw
4. Centra en imagen
5. Se acerca recto al pizarrón
6. Se posiciona abajo-derecha del ArUco
7. Hace contacto
8. Traza línea horizontal
9. Retrocede 0.55 m
10. Gira -90° (a la derecha)
11. Publica "done"

---

## Parar el drone manualmente

```bash
# Aterrizaje normal
rostopic pub --once /bebop/land std_msgs/Empty "{}"

# Emergencia (apaga motores — solo si va a chocar)
rostopic pub --once /bebop/reset std_msgs/Empty "{}"
```

---

## Parámetros de calibración principales

| Parámetro | Default | Cuándo ajustar |
|-----------|---------|----------------|
| `_cam_tilt` | 18.0° | Si no ve el ArUco al buscar |
| `_approach_area_threshold` | 17000 | Si para muy lejos o muy cerca en approach |
| `_area_abort_threshold` | 32000 | Si el freno de emergencia actúa muy pronto |
| `_approach_speed` | 0.10 | Si avanza muy rápido/lento |
| `_draw_down` | 0.15 m | Si el punto de inicio del trazo queda muy alto/bajo |
| `_draw_right` | 0.28 m | Si el punto de inicio queda muy cerca/lejos del ArUco |
| `_touch_time` | 1.0 s | Si no hace contacto o choca |
| `_draw_time` | 2.0 s | Si la línea es muy corta/larga |
| `_backoff_distance` | 0.55 m | Distancia de separación tras el trazo |

---

## Verificar odometría en vivo durante la misión

```bash
# En otra terminal mientras corre la misión:
rostopic echo /bebop/odom/pose/pose
```
Verás las coordenadas X, Y, Z actualizándose. Útil para verificar que
los movimientos odométricos funcionan correctamente.

---

## Notas importantes

- **La odometría del Bebop (`/bebop/odom`) da posición XY real** — confirmado con pruebas reales (notas 18 feb y `mission_square_1.py`).
- **El ArUco solo se usa como referencia de alineación inicial.** Los estados MOVE_TO_DRAW_START, BACK_OFF y ROTATE_RIGHT_90 no lo necesitan.
- **APPROACH nunca manda lx y ly/lz al mismo tiempo.** Si hay deriva, para, corrige, y reanuda.
- **`already_close` es un flag permanente** — si alguna vez el área supera el umbral, el drone no volverá a intentar acercarse aunque re-detecte el ArUco desde lejos.
- Diccionario: `DICT_5X5_250`, ID `100`.
