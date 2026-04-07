#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mission_whiteboard_aruco.py
============================
Misión autónoma: dibujar una línea horizontal en el pizarrón.

ARQUITECTURA DE ESTADOS
────────────────────────
SET_CAMERA          → ajusta cámara a tilt para ver el pizarrón
SEARCH_ARUCO        → gira lentamente hasta detectar ArUco ID=100 con estabilidad
ALIGN_YAW           → corrige SOLO yaw (az) — sin traslación
ALIGN_CENTER        → centra lateral y vertical — sin avanzar, sin diagonal
APPROACH_BOARD      → avanza SOLO lx, correctivo hace pausa (no diagonal)
                      · si área >= area_abort → freno de emergencia
                      · si already_close → para y avanza a MOVE_TO_DRAW_START
                      · si pierde ArUco > approach_loss_timeout → SAFE_RETREAT
SAFE_RETREAT        → retrocede retreat_distance m con odometría → SEARCH_ARUCO
MOVE_TO_DRAW_START  → odometría: fase DOWN (baja draw_down m)
                                  fase RIGHT (mueve draw_right m a la derecha)
TOUCH_BOARD         → open-loop: empuje lx=touch_speed durante touch_time s
DRAW_LINE           → open-loop: lx=fwd_bias + ly=lat_speed durante draw_time s
BACK_OFF            → odometría: retrocede backoff_distance m
ROTATE_RIGHT_90     → odometría yaw: gira -90° con PD
DONE                → publica "done"

ODOMETRÍA CONFIRMADA
─────────────────────
El topic /bebop/odom publica posición XY real (confirmado en notas 18 feb y
misión mission_square_1.py que ya lo usa correctamente).

SEGURIDAD
─────────
· already_close: flag permanente. Una vez que el área >= approach_area_threshold
  el dron no intenta acercarse más aunque re-detecte el ArUco desde lejos.
· area_abort_threshold: freno de emergencia independiente del estado.
· Alineación no busca perfección — tolerancias amplias.
· APPROACH nunca manda lx y ly/lz al mismo tiempo.
· Teclado: q=land, e=reset

CALIBRACIÓN POR ETAPAS (test_mode)
────────────────────────────────────
search_align  → para después de ALIGN_CENTER
approach      → para después de APPROACH_BOARD
draw_start    → para después de MOVE_TO_DRAW_START
touch         → para después de TOUCH_BOARD
draw          → para después de DRAW_LINE
full          → misión completa (default)
"""

import os
import sys
import math
import threading
import select
import termios
import tty

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image
from std_msgs.msg import String, Empty
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from tf.transformations import euler_from_quaternion

current_dir  = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, '..'))
if project_root not in sys.path:
    sys.path.append(project_root)

from control.bebop_teleop_controller import BebopMovements
from perception.aruco_whiteboard_detector import ArucoWhiteboardDetector


class MissionWhiteboardAruco:

    def __init__(self):
        rospy.init_node('mission_whiteboard_aruco')

        # ── Publishers ────────────────────────────────────────────────────
        self.pub_cmd     = rospy.Publisher('/bebop/cmd_vel',        Twist,  queue_size=1)
        self.pub_takeoff = rospy.Publisher('/bebop/takeoff',        Empty,  queue_size=1)
        self.pub_land    = rospy.Publisher('/bebop/land',           Empty,  queue_size=1)
        self.pub_reset   = rospy.Publisher('/bebop/reset',          Empty,  queue_size=1)
        self.pub_camera  = rospy.Publisher('/bebop/camera_control', Twist,  queue_size=1)
        self.status_pub  = rospy.Publisher('/mission/status',       String, queue_size=1)

        self.movements = BebopMovements(
            self.pub_cmd, self.pub_takeoff, self.pub_land, self.pub_camera
        )
        self.detector = ArucoWhiteboardDetector()
        self.bridge   = CvBridge()

        # ── Suscriptores ──────────────────────────────────────────────────
        self.image_sub = rospy.Subscriber(
            '/bebop/image_raw', Image, self.image_callback,
            queue_size=1, buff_size=2**24
        )
        self.odom_sub = rospy.Subscriber(
            '/bebop/odom', Odometry, self.odom_callback, queue_size=1
        )

        # ── Visión ────────────────────────────────────────────────────────
        self.latest_image_msg    = None
        self.latest_data         = None
        self.debug_image         = None
        self.last_detection_time = None
        self.last_seen_area      = 0.0

        # Filtro exponencial (suaviza errores de visión frame a frame)
        self.alpha       = 0.35
        self.f_error_x   = None
        self.f_error_y   = None
        self.f_yaw_error = None
        self.f_area      = None

        self.stable_count = 0   # frames consecutivos con detección
        self.lost_count   = 0   # frames consecutivos sin detección

        # ── Odometría ────────────────────────────────────────────────────
        self.odom_x   = None
        self.odom_y   = None
        self.odom_z   = None
        self.odom_yaw = 0.0

        # Referencias internas para movimientos odométricos
        self._ref_x      = None
        self._ref_y      = None
        self._ref_z      = None
        self._ref_yaw    = None
        self._target_yaw = None
        self._draw_phase = None   # sub-fase MOVE_TO_DRAW_START: 'down' | 'right'

        # Flag permanente: ya estuvimos lo suficientemente cerca del pizarrón
        self.already_close = False

        # ── Máquina de estados ────────────────────────────────────────────
        self.finished         = False
        self.state            = 'SET_CAMERA'
        self.state_start_time = rospy.Time.now()
        self.start_time       = rospy.Time.now()

        # ── Teclado de emergencia ─────────────────────────────────────────
        self.abort_land  = False
        self.abort_reset = False
        self.term_settings = None
        self._start_keyboard()

        # ═════════════════════════════════════════════════════════════════
        # PARÁMETROS — todos sobreescribibles desde el launch
        # ═════════════════════════════════════════════════════════════════

        self.test_mode  = rospy.get_param('~test_mode',    'full')
        self.show_debug = rospy.get_param('~show_debug',   True)
        self.img_w      = rospy.get_param('~image_width',   640)
        self.img_h      = rospy.get_param('~image_height',  360)

        # Cámara
        self.cam_tilt        = rospy.get_param('~cam_tilt',        18.0)
        self.cam_settle_time = rospy.get_param('~cam_settle_time',   1.5)

        # Detección mínima estable antes de actuar
        self.min_stable = rospy.get_param('~min_stable', 8)

        # ─ ALIGN_YAW ─────────────────────────────────────────────────────
        self.tol_yaw  = rospy.get_param('~tol_yaw',   0.18)   # rad visual
        self.v_yaw    = rospy.get_param('~v_yaw',      0.12)   # velocidad corrección yaw

        # ─ ALIGN_CENTER ──────────────────────────────────────────────────
        self.tol_x      = rospy.get_param('~tol_x',      60)    # px — tolerancia amplia
        self.tol_y      = rospy.get_param('~tol_y',      45)    # px
        self.v_lat      = rospy.get_param('~v_lat',      0.10)  # velocidad lateral
        self.v_vert     = rospy.get_param('~v_vert',     0.10)  # velocidad vertical
        self.align_hold = rospy.get_param('~align_hold',  0.6)  # s quieto antes de avanzar

        # ─ APPROACH_BOARD ────────────────────────────────────────────────
        # Área en px² cuando el ArUco mide ~130px de lado (≈1.0–1.2 m distancia)
        self.approach_area_threshold = rospy.get_param('~approach_area_threshold', 17000)
        # Área para freno de emergencia (≈0.5 m distancia)
        self.area_abort_threshold    = rospy.get_param('~area_abort_threshold',    32000)
        # Velocidad de avance — solo lx, NUNCA diagonal
        self.approach_speed          = rospy.get_param('~approach_speed',          0.10)
        # Tolerancias de deriva durante approach (más amplias que en alineación)
        self.approach_tol_x          = rospy.get_param('~approach_tol_x',          80)
        self.approach_tol_y          = rospy.get_param('~approach_tol_y',          60)
        self.approach_tol_yaw        = rospy.get_param('~approach_tol_yaw',         0.25)
        # Tiempo máximo sin ArUco antes de SAFE_RETREAT
        self.approach_loss_timeout   = rospy.get_param('~approach_loss_timeout',    1.5)

        # ─ SAFE_RETREAT ──────────────────────────────────────────────────
        self.retreat_distance = rospy.get_param('~retreat_distance', 0.40)  # m

        # ─ MOVE_TO_DRAW_START (odometría) ────────────────────────────────
        self.draw_down  = rospy.get_param('~draw_down',  0.15)   # m bajar
        self.draw_right = rospy.get_param('~draw_right', 0.28)   # m a la derecha

        # ─ TOUCH_BOARD (open-loop) ───────────────────────────────────────
        self.touch_speed = rospy.get_param('~touch_speed', 0.07)
        self.touch_time  = rospy.get_param('~touch_time',  1.0)

        # ─ DRAW_LINE (open-loop) ─────────────────────────────────────────
        self.draw_fwd_bias  = rospy.get_param('~draw_fwd_bias',   0.02)
        self.draw_lat_speed = rospy.get_param('~draw_lat_speed',  -0.14)  # negativo=derecha
        self.draw_time      = rospy.get_param('~draw_time',        2.0)

        # ─ BACK_OFF (odometría) ──────────────────────────────────────────
        self.backoff_distance = rospy.get_param('~backoff_distance', 0.55)

        # ─ ROTATE_RIGHT_90 (odometría yaw) ──────────────────────────────
        self.rotate_tol = rospy.get_param('~rotate_tol', 0.04)   # rad

        # Ganancias para controladores odométricos
        self.kp_odom   = rospy.get_param('~kp_odom',    0.80)
        self.max_odom  = rospy.get_param('~max_odom',   0.20)
        self.min_odom  = rospy.get_param('~min_odom',   0.05)
        self.kp_yaw_od = rospy.get_param('~kp_yaw_od',  1.00)
        self.max_rot   = rospy.get_param('~max_rot',    0.35)

        self.max_mission_time = rospy.get_param('~max_mission_time', 60.0)

        rospy.loginfo(f'[WB] MissionWhiteboardAruco | test_mode={self.test_mode}')
        rospy.loginfo(f'[WB] approach_area={self.approach_area_threshold}  abort={self.area_abort_threshold}')

    # ═════════════════════════════════════════════════════════════════
    # TECLADO DE EMERGENCIA
    # ═════════════════════════════════════════════════════════════════

    def _keyboard_loop(self):
        while not rospy.is_shutdown() and not self.finished:
            if select.select([sys.stdin], [], [], 0.05)[0]:
                key = sys.stdin.read(1).lower()
                if key == 'q':
                    rospy.logwarn('[WB] EMERGENCIA q → aterrizaje')
                    self.abort_land = True
                    break
                elif key == 'e':
                    rospy.logerr('[WB] EMERGENCIA e → reset')
                    self.abort_reset = True
                    break

    def _start_keyboard(self):
        if not sys.stdin.isatty():
            rospy.logwarn('[WB] stdin no es TTY — emergencia por teclado no disponible')
            return
        try:
            self.term_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
            threading.Thread(target=self._keyboard_loop, daemon=True).start()
            rospy.loginfo("[WB] Teclado activo: 'q'=land  'e'=reset")
        except Exception as ex:
            rospy.logwarn(f'[WB] No se pudo activar teclado: {ex}')

    def _restore_terminal(self):
        if self.term_settings and sys.stdin.isatty():
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.term_settings)
            except Exception:
                pass

    def _check_emergency(self):
        if self.abort_reset:
            self.stop()
            self.pub_reset.publish(Empty())
            self.status_pub.publish('failed')
            self.finished = True
            return True
        if self.abort_land:
            self.stop()
            self.pub_land.publish(Empty())
            self.status_pub.publish('failed')
            self.finished = True
            return True
        return False

    # ═════════════════════════════════════════════════════════════════
    # CALLBACKS
    # ═════════════════════════════════════════════════════════════════

    def image_callback(self, msg):
        if not self.finished:
            self.latest_image_msg = msg

    def odom_callback(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.odom_z = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.odom_yaw = yaw

    # ═════════════════════════════════════════════════════════════════
    # PROCESAMIENTO DE IMAGEN
    # ═════════════════════════════════════════════════════════════════

    def process_image(self):
        if self.latest_image_msg is None:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(self.latest_image_msg, 'bgr8')
        except Exception as ex:
            rospy.logerr(f'[WB] CvBridge: {ex}')
            return

        frame = cv2.resize(frame, (self.img_w, self.img_h))
        processed, data = self.detector.process_image(frame)

        if data['detected']:
            yaw_err = self._visual_yaw(data)
            self.f_error_x   = self._smooth(self.f_error_x,   data['error_x'])
            self.f_error_y   = self._smooth(self.f_error_y,   data['error_y'])
            self.f_yaw_error = self._smooth(self.f_yaw_error, yaw_err)
            self.f_area      = self._smooth(self.f_area,      data['area'])

            self.last_detection_time = rospy.Time.now()
            self.last_seen_area      = float(data['area'])
            self.stable_count       += 1
            self.lost_count          = 0

            # Flag permanente de proximidad
            if data['area'] >= self.approach_area_threshold:
                self.already_close = True
        else:
            self.stable_count = 0
            self.lost_count  += 1

        self.debug_image = processed
        self.latest_data = data

    def _smooth(self, prev, curr):
        if curr is None:
            return prev
        if prev is None:
            return float(curr)
        return (1.0 - self.alpha) * float(prev) + self.alpha * float(curr)

    def _visual_yaw(self, data):
        """Estima yaw_error visual usando asimetría de esquinas del ArUco.
        No requiere calibración de cámara."""
        pts = data.get('corners')
        if pts is None or len(pts) != 4:
            return 0.0
        pts = np.array(pts, dtype=np.float32)
        tl, tr, br, bl = pts
        top_angle = math.atan2(tr[1] - tl[1], tr[0] - tl[0])
        lh  = np.linalg.norm(bl - tl)
        rh  = np.linalg.norm(br - tr)
        avg = max((lh + rh) / 2.0, 1e-6)
        return 0.8 * ((lh - rh) / avg) + 0.7 * top_angle

    # ═════════════════════════════════════════════════════════════════
    # HELPERS DE ESTADO
    # ═════════════════════════════════════════════════════════════════

    def set_state(self, new):
        if self.state == new:
            return
        rospy.loginfo(f'[WB] {self.state} → {new}')
        self.state            = new
        self.state_start_time = rospy.Time.now()
        self.stop()
        # Limpiar referencias para estados odométricos
        self._ref_x = self._ref_y = self._ref_z = None
        self._ref_yaw = self._target_yaw = None
        self._draw_phase = None

    def elapsed(self):
        return (rospy.Time.now() - self.state_start_time).to_sec()

    def detection_fresh(self):
        if self.last_detection_time is None:
            return False
        return (rospy.Time.now() - self.last_detection_time).to_sec() < self.approach_loss_timeout

    def odom_ok(self):
        return self.odom_x is not None

    def _norm(self, a):
        return math.atan2(math.sin(a), math.cos(a))

    def twist(self, lx=0.0, ly=0.0, lz=0.0, az=0.0):
        msg = Twist()
        msg.linear.x  = lx
        msg.linear.y  = ly
        msg.linear.z  = lz
        msg.angular.z = az
        self.pub_cmd.publish(msg)

    def stop(self):
        self.movements.reset_twist()

    def finish(self):
        self.stop()
        self.status_pub.publish('done')
        self.finished = True
        rospy.loginfo('[WB] Misión completada ✔')

    def fail(self, reason='unknown'):
        rospy.logwarn(f'[WB] Misión fallida: {reason}')
        self.stop()
        self.status_pub.publish('failed')
        self.finished = True

    def stop_after(self, stage):
        order  = {'search_align': 1, 'approach': 2, 'draw_start': 3,
                  'touch': 4, 'draw': 5, 'full': 6}
        stages = {'ALIGN_OK': 1, 'APPROACH_OK': 2, 'DRAW_START_OK': 3,
                  'TOUCH_OK': 4, 'DRAW_OK': 5, 'FULL_OK': 6}
        return stages.get(stage, 999) >= order.get(self.test_mode, 6)

    def _odom_speed(self, err):
        """Velocidad proporcional clampeada para movimientos odométricos."""
        return max(min(self.kp_odom * abs(err), self.max_odom), self.min_odom)

    # ═════════════════════════════════════════════════════════════════
    # ESTADOS: VISIÓN ACTIVA
    # ═════════════════════════════════════════════════════════════════

    def handle_set_camera(self):
        cam = Twist()
        cam.angular.y = self.cam_tilt
        self.pub_camera.publish(cam)
        if self.elapsed() >= self.cam_settle_time:
            self.set_state('SEARCH_ARUCO')

    def handle_search_aruco(self):
        data = self.latest_data
        if data is None or not data['detected']:
            self.twist(az=0.15)
            return
        if self.stable_count < self.min_stable:
            self.stop()
            return
        self.set_state('ALIGN_YAW')

    def handle_align_yaw(self):
        """Corrige SOLO el yaw visual. Sin traslación."""
        data = self.latest_data
        if data is None or not data['detected']:
            if self.lost_count > 15:
                self.set_state('SEARCH_ARUCO')
            return

        yaw_err = self.f_yaw_error if self.f_yaw_error is not None else self._visual_yaw(data)

        if abs(yaw_err) > self.tol_yaw:
            # Velocidad proporcional al error pero limitada
            scale = min(abs(yaw_err) / self.tol_yaw, 1.0)
            wz    = math.copysign(self.v_yaw * scale, yaw_err)
            self.twist(az=wz)
            return

        self.stop()
        self.set_state('ALIGN_CENTER')

    def handle_align_center(self):
        """Centra lateral y vertical. UN eje a la vez, sin avanzar."""
        data = self.latest_data
        if data is None or not data['detected']:
            if self.lost_count > 15:
                self.set_state('SEARCH_ARUCO')
            return

        err_x = self.f_error_x if self.f_error_x is not None else data['error_x']
        err_y = self.f_error_y if self.f_error_y is not None else data['error_y']

        if abs(err_x) > self.tol_x:
            # Solo lateral
            self.twist(ly=self.v_lat if err_x < 0 else -self.v_lat)
            return

        if abs(err_y) > self.tol_y:
            # Solo vertical
            self.twist(lz=self.v_vert if err_y < 0 else -self.v_vert)
            return

        # Alineado — esperar un momento antes de avanzar para confirmar estabilidad
        self.stop()
        if self.elapsed() < self.align_hold:
            return

        if self.stop_after('ALIGN_OK'):
            self.finish()
            return

        self.set_state('APPROACH_BOARD')

    def handle_approach_board(self):
        """Avanza SOLO lx. Correcciones de deriva hacen una pausa, nunca diagonal.
        El area_abort_threshold actúa como freno de emergencia independiente."""
        data = self.latest_data

        # ── Freno de emergencia por área ─────────────────────────────────
        if data and data['detected']:
            area = self.f_area if self.f_area is not None else data['area']
            if area >= self.area_abort_threshold:
                rospy.logwarn(f'[WB] ABORT AREA {area:.0f} → demasiado cerca, para')
                self.stop()
                self.already_close = True
                if self.stop_after('APPROACH_OK'):
                    self.finish()
                    return
                self.set_state('MOVE_TO_DRAW_START')
                return

        # ── Ya estuvimos cerca alguna vez → no acercarse más ─────────────
        if self.already_close:
            self.stop()
            if self.stop_after('APPROACH_OK'):
                self.finish()
                return
            self.set_state('MOVE_TO_DRAW_START')
            return

        # ── Pérdida de ArUco ─────────────────────────────────────────────
        if data is None or not data['detected']:
            if self.detection_fresh():
                self.stop()   # hover breve esperando recuperar
            else:
                rospy.logwarn('[WB] ArUco perdido > timeout → SAFE_RETREAT')
                self.set_state('SAFE_RETREAT')
            return

        # ── Detección válida → verificar deriva antes de avanzar ─────────
        err_x   = self.f_error_x   if self.f_error_x   is not None else data['error_x']
        err_y   = self.f_error_y   if self.f_error_y   is not None else data['error_y']
        yaw_err = self.f_yaw_error if self.f_yaw_error is not None else self._visual_yaw(data)

        if abs(err_x) > self.approach_tol_x:
            # Para lx, corrige lateral
            self.twist(ly=self.v_lat if err_x < 0 else -self.v_lat)
            return

        if abs(err_y) > self.approach_tol_y:
            # Para lx, corrige vertical
            self.twist(lz=self.v_vert if err_y < 0 else -self.v_vert)
            return

        if abs(yaw_err) > self.approach_tol_yaw:
            # Para lx, corrige yaw
            scale = min(abs(yaw_err) / self.tol_yaw, 1.0)
            wz    = math.copysign(self.v_yaw * scale, yaw_err)
            self.twist(az=wz)
            return

        # Alineado — avanzar recto, SOLO lx
        self.twist(lx=self.approach_speed)

    def handle_safe_retreat(self):
        """Retrocede retreat_distance m con odometría, luego vuelve a SEARCH_ARUCO."""
        if not self.odom_ok():
            return
        if self._ref_x is None:
            self._ref_x = self.odom_x
            self._ref_y = self.odom_y
            rospy.logwarn(f'[WB] SAFE_RETREAT desde ({self._ref_x:.3f}, {self._ref_y:.3f})')

        dist = math.sqrt((self.odom_x - self._ref_x)**2 + (self.odom_y - self._ref_y)**2)
        err  = self.retreat_distance - dist

        if err > 0.03:
            self.twist(lx=-self._odom_speed(err))
        else:
            self.stop()
            rospy.loginfo(f'[WB] SAFE_RETREAT OK ({dist:.3f} m) → SEARCH_ARUCO')
            self.set_state('SEARCH_ARUCO')

    # ═════════════════════════════════════════════════════════════════
    # ESTADOS: ODOMETRÍA (sin ArUco)
    # ═════════════════════════════════════════════════════════════════

    def handle_move_to_draw_start(self):
        """Odometría pura: baja draw_down m, luego mueve draw_right m a la derecha."""
        if not self.odom_ok():
            rospy.logwarn_throttle(1.0, '[WB] Esperando odometría...')
            return

        if self._draw_phase is None:
            self._ref_z      = self.odom_z
            self._draw_phase = 'down'
            rospy.loginfo(f'[WB] DRAW_START fase DOWN: z0={self._ref_z:.3f}')

        if self._draw_phase == 'down':
            dz  = abs(self.odom_z - self._ref_z)
            err = self.draw_down - dz
            if err > 0.02:
                self.twist(lz=-self._odom_speed(err))   # negativo = bajar
            else:
                self.stop()
                self._ref_y      = self.odom_y
                self._draw_phase = 'right'
                rospy.loginfo(f'[WB] DRAW_START fase RIGHT: y0={self._ref_y:.3f}')
            return

        if self._draw_phase == 'right':
            dy  = abs(self.odom_y - self._ref_y)
            err = self.draw_right - dy
            if err > 0.02:
                self.twist(ly=-self._odom_speed(err))   # negativo = derecha en Bebop
            else:
                self.stop()
                rospy.loginfo('[WB] DRAW_START OK → TOUCH_BOARD')
                if self.stop_after('DRAW_START_OK'):
                    self.finish()
                    return
                self.set_state('TOUCH_BOARD')

    def handle_back_off(self):
        """Retrocede backoff_distance m con odometría."""
        if not self.odom_ok():
            return
        if self._ref_x is None:
            self._ref_x = self.odom_x
            self._ref_y = self.odom_y
            rospy.loginfo(f'[WB] BACK_OFF desde ({self._ref_x:.3f}, {self._ref_y:.3f})')

        dist = math.sqrt((self.odom_x - self._ref_x)**2 + (self.odom_y - self._ref_y)**2)
        err  = self.backoff_distance - dist

        if err > 0.03:
            self.twist(lx=-self._odom_speed(err))
        else:
            self.stop()
            rospy.loginfo(f'[WB] BACK_OFF OK ({dist:.3f} m) → ROTATE_RIGHT_90')
            self.set_state('ROTATE_RIGHT_90')

    def handle_rotate_right_90(self):
        """Gira -90° con PD sobre odom_yaw."""
        if not self.odom_ok():
            return
        if self._ref_yaw is None:
            self._ref_yaw    = self.odom_yaw
            self._target_yaw = self._norm(self.odom_yaw - math.pi / 2.0)
            rospy.loginfo(
                f'[WB] ROTATE: {math.degrees(self._ref_yaw):.1f}° '
                f'→ {math.degrees(self._target_yaw):.1f}°'
            )

        err = self._norm(self._target_yaw - self.odom_yaw)

        if abs(err) > self.rotate_tol:
            speed = max(min(self.kp_yaw_od * err, self.max_rot), -self.max_rot)
            self.twist(az=speed)
        else:
            self.stop()
            rospy.loginfo(f'[WB] ROTATE OK → DONE (yaw={math.degrees(self.odom_yaw):.1f}°)')
            self.set_state('DONE')

    # ═════════════════════════════════════════════════════════════════
    # ESTADOS: OPEN-LOOP (contacto físico)
    # ═════════════════════════════════════════════════════════════════

    def handle_touch_board(self):
        if self.elapsed() < self.touch_time:
            self.twist(lx=self.touch_speed)
        else:
            self.stop()
            rospy.loginfo('[WB] TOUCH OK → DRAW_LINE')
            if self.stop_after('TOUCH_OK'):
                self.finish()
                return
            self.set_state('DRAW_LINE')

    def handle_draw_line(self):
        if self.elapsed() < self.draw_time:
            self.twist(lx=self.draw_fwd_bias, ly=self.draw_lat_speed)
        else:
            self.stop()
            rospy.loginfo('[WB] DRAW OK → BACK_OFF')
            if self.stop_after('DRAW_OK'):
                self.finish()
                return
            self.set_state('BACK_OFF')

    def handle_done(self):
        self.finish()

    # ═════════════════════════════════════════════════════════════════
    # MÁQUINA DE ESTADOS PRINCIPAL
    # ═════════════════════════════════════════════════════════════════

    def control_logic(self):
        if self.finished:
            return
        if (rospy.Time.now() - self.start_time).to_sec() > self.max_mission_time:
            self.fail('timeout')
            return

        s = self.state
        if   s == 'SET_CAMERA':          self.handle_set_camera()
        elif s == 'SEARCH_ARUCO':        self.handle_search_aruco()
        elif s == 'ALIGN_YAW':           self.handle_align_yaw()
        elif s == 'ALIGN_CENTER':        self.handle_align_center()
        elif s == 'APPROACH_BOARD':      self.handle_approach_board()
        elif s == 'SAFE_RETREAT':        self.handle_safe_retreat()
        elif s == 'MOVE_TO_DRAW_START':  self.handle_move_to_draw_start()
        elif s == 'TOUCH_BOARD':         self.handle_touch_board()
        elif s == 'DRAW_LINE':           self.handle_draw_line()
        elif s == 'BACK_OFF':            self.handle_back_off()
        elif s == 'ROTATE_RIGHT_90':     self.handle_rotate_right_90()
        elif s == 'DONE':                self.handle_done()
        else:                            self.fail(f'estado_desconocido_{s}')

    # ═════════════════════════════════════════════════════════════════
    # LOOP PRINCIPAL
    # ═════════════════════════════════════════════════════════════════

    def run(self):
        self.start_time = rospy.Time.now()
        rate = rospy.Rate(15)
        rospy.loginfo('[WB] Misión iniciada.')

        while not rospy.is_shutdown() and not self.finished:
            if self._check_emergency():
                break
            self.process_image()
            self.control_logic()

            if self.show_debug and self.debug_image is not None:
                cv2.imshow('whiteboard_debug', self.debug_image)
                cv2.waitKey(1)

            rate.sleep()

        self.stop()
        if self.show_debug:
            cv2.destroyAllWindows()
        self._restore_terminal()


if __name__ == '__main__':
    mission = MissionWhiteboardAruco()
    mission.run()
