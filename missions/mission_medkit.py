#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Mission 3: Find the cry for help sound and deliver a med-kit
─────────────────────────────────────────────────────────────
TMR 2026 / IMAV 2025

Descripción:
  - Dentro de la arena hay hasta 5 cajas negras (57cm × 78cm × 36cm) en el suelo.
  - Una de ellas contiene una bocina con sonido de auxilio (≥ 70 dB a 1m).
  - El MAV debe localizar esa caja y lanzar un med-kit (20cm × 10cm × 5cm, 30g) dentro.

Scoring:
  - Caja no encontrada:   0 pts
  - Caja encontrada:      3 pts
  - Med-kit entregado:     2 pts

Estado: STUB — Requiere implementación de:
  1. Detección de audio direccional (micrófono + análisis de frecuencia/intensidad)
  2. Detección visual de cajas negras (percepción por cámara)
  3. Mecanismo de entrega del med-kit (servo/electroimán/gripper)

NOTA: El equipo puede optar por no buscar la caja. En ese caso, se le dará
la ubicación aproximada, pero solo se puntúa si la entrega es exitosa.
"""

import os
import sys
import rospy
from std_msgs.msg import String, Empty
from geometry_msgs.msg import Twist

# =====================================================
# FIX IMPORT PATH
# =====================================================
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, '..'))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from control.bebop_teleop_controller import BebopMovements


class MissionMedkit:
    """
    Stub de la Misión 3: buscar sonido de auxilio y entregar med-kit.

    TODO: Integrar módulos de:
      - Audio: micrófono direccional para localizar la fuente de sonido.
      - Percepción: detección visual de cajas negras en el suelo.
      - Actuador: mecanismo de liberación del med-kit.
    """

    def __init__(self):
        rospy.init_node('mission_medkit')

        # Publishers
        self.pub_cmd = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1)
        self.pub_takeoff = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1)
        self.pub_land = rospy.Publisher('/bebop/land', Empty, queue_size=1)
        self.pub_camera = rospy.Publisher('/bebop/camera_control', Twist, queue_size=1)
        self.status_pub = rospy.Publisher('/mission/status', String, queue_size=1)

        # Movements
        self.movements = BebopMovements(
            self.pub_cmd,
            self.pub_takeoff,
            self.pub_land,
            self.pub_camera
        )

        # State
        self.finished = False

        # Parámetros
        self.max_mission_time = rospy.get_param("~max_mission_time", 60.0)
        self.use_known_location = rospy.get_param("~use_known_location", False)
        # Si se opta por no buscar, se puede pasar la ubicación aproximada
        self.target_x = rospy.get_param("~target_x", 0.0)
        self.target_y = rospy.get_param("~target_y", 0.0)

        rospy.loginfo("=" * 50)
        rospy.loginfo("Mission 3: Find Cry for Help & Deliver Med-Kit")
        rospy.logwarn("⚠️  STUB — Funcionalidad limitada")
        rospy.logwarn("   Requiere: módulo de audio, detección de cajas, mecanismo de entrega")
        rospy.loginfo("=" * 50)

    def finish_mission(self):
        """Publicar done y terminar."""
        self.movements.stop("automatic") if hasattr(self.movements, 'stop') else None
        self.status_pub.publish("done")
        self.finished = True
        rospy.loginfo("Mission Medkit completed (stub)")

    def fail_mission(self, reason="unknown"):
        """Publicar failed y terminar."""
        rospy.logwarn(f"Mission Medkit failed: {reason}")
        self.status_pub.publish("failed")
        self.finished = True

    def run(self):
        """
        Ejecutar el stub de la misión.

        Flujo esperado cuando se implemente:
          1. Despegar (si no viene del orquestador con dron ya en vuelo)
          2. Ajustar cámara para buscar cajas negras en el suelo
          3. Si use_known_location: navegar a la posición dada
             Si no: escanear la arena buscando cajas + sonido
          4. Localizar la caja con sonido (análisis de audio direccional)
          5. Posicionarse sobre la caja
          6. Liberar el med-kit
          7. Verificar entrega (visión)
          8. Publicar "done"
        """
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()

        rospy.loginfo("Mission Medkit: Stub ejecutándose...")
        rospy.loginfo("   (Este stub simula la misión con pausa de 5 segundos)")

        while not rospy.is_shutdown() and not self.finished:
            elapsed = (rospy.Time.now() - start_time).to_sec()

            if elapsed >= self.max_mission_time:
                self.fail_mission("timeout")
                break

            # ── STUB: simular ejecución ──
            if elapsed >= 5.0:
                rospy.loginfo("Stub: simulación completada")
                self.finish_mission()
                break

            rate.sleep()


if __name__ == "__main__":
    try:
        mission = MissionMedkit()
        mission.run()
    except rospy.ROSInterruptException:
        pass
