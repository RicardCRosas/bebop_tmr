#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
============================================================================
  Competition Orchestrator — Torneo Mexicano de Robótica (TMR) 2026
  Categoría: Drones Autónomos
  Basado en el Rulebook IMAV 2025 (con adaptaciones TMR 2026)
============================================================================

  Misiones (tomadas del IMAV 2025):
  ──────────────────────────────────────────────────────────────────────
  Mission 1 — Enter the room
      Opciones de entrada:
        • Free passage (1 m de ancho)           →  0 pts
        • Wide tunnel   (ventana 1.5 m × 1.5 m) →  1 pt
        • Medium tunnel (ventana 1.0 m × 1.0 m) →  2 pts
        • Small tunnel  (ventana 0.5 m × 0.5 m) →  3 pts

  Mission 2 — Draw on the whiteboard
      • Sin dibujo                              →  0 pts
      • Línea continua (mín. 20 cm)             →  3 pts
      • Cada 20 cm adicionales de línea         → +1 pt

  Mission 3 — Find the cry for help sound and deliver a med-kit
      • Caja no encontrada                      →  0 pts
      • Caja encontrada                         →  3 pts
      • Med-kit entregado                       →  2 pts

  Mission 4 — Land on a moving platform (SIN HUMO — adaptación TMR 2026)
      • Sin aterrizaje                          →  0 pts
      • Aterrizaje (plataforma estática)        →  2 pts
      • Aterrizaje con plataforma en movimiento → +3 pts

  Multiplicadores:
      • Single MAV Onboard Processing           → Final Score × 1.5
      • Swarm Solution Offboard Processing      → Final Score × 2.0
      • Swarm Solution Onboard Processing       → Final Score × 3.0

  Presentación: Final Score + 3 × (0…10)*
  ──────────────────────────────────────────────────────────────────────

  Scripts mapeados en el proyecto:
      Mission 1 → missions/mission_orange_window.py
      Mission 2 → missions/mission_whiteboard_aruco.py
      Mission 3 → missions/mission_medkit.py          (stub — requiere hardware de audio/entrega)
      Mission 4 → missions/mission_4.py               (HelipadDetector — aterrizaje autónomo)

  Uso:
      python3 competition_orchestrator.py --combo 1,2,3,4
      python3 competition_orchestrator.py --combo 1,2 --tunnel medium
      python3 competition_orchestrator.py --combo 4 --platform moving
      python3 competition_orchestrator.py --combo 1,2,3,4 --multiplier onboard
"""

import os
import sys
import time
import signal
import subprocess
import argparse
import datetime

import rospy
from std_msgs.msg import String, Empty

# ============================================================================
# PATHS
# ============================================================================
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
MISSIONS_DIR = os.path.abspath(os.path.join(CURRENT_DIR, '..'))
PROJECT_ROOT = os.path.abspath(os.path.join(MISSIONS_DIR, '..'))

# Asegurar imports del proyecto
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

# ============================================================================
# MISSION REGISTRY — Mapeo a scripts reales del proyecto
# ============================================================================
MISSION_REGISTRY = {
    "mission_1": {
        "name": "Mission 1: Enter the Room",
        "script": os.path.join(MISSIONS_DIR, "mission_orange_window.py"),
        "description": "Navegar y atravesar el túnel/ventana para entrar al área de navegación.",
        "implemented": True,
    },
    "mission_point_to_point": {
        "name": "Mission Intermediate: Point to Point",
        "script": os.path.join(MISSIONS_DIR, "ckeck", "mission_point_to_point.py"),
        "description": "Navegación point to point en la arena (entre Window y Whiteboard).",
        "implemented": True,
    },
    "mission_2": {
        "name": "Mission 2: Draw on the Whiteboard",
        "script": os.path.join(MISSIONS_DIR, "mission_whiteboard_aruco.py"),
        "description": "Detectar el ArUco, acercarse al pizarrón y dibujar una línea horizontal continua.",
        "implemented": True,
    },
    "mission_3": {
        "name": "Mission 3: Find Cry for Help & Deliver Med-Kit",
        "script": os.path.join(MISSIONS_DIR, "mission_medkit.py"),
        "description": "Localizar la caja negra con sonido de auxilio y entregar el med-kit.",
        "implemented": False,  # Stub — requiere hardware de audio y mecanismo de entrega
    },
    "mission_4": {
        "name": "Mission 4: Land on Platform (Sin Humo — TMR 2026)",
        "script": os.path.join(MISSIONS_DIR, "mission_4.py"),
        "description": "Aterrizar sobre la plataforma (estática o en movimiento). Sin humo para TMR 2026.",
        "implemented": True,
    },
}

# ============================================================================
# SCORING TABLES (IMAV 2025 / TMR 2026)
# ============================================================================
SCORING = {
    "mission_1": {
        "free_path": 0,
        "wide":      1,   # Ventana 1.5 m × 1.5 m
        "medium":    2,   # Ventana 1.0 m × 1.0 m
        "small":     3,   # Ventana 0.5 m × 0.5 m
    },
    "mission_2": {
        "no_drawing":       0,
        "continuous_line":  3,   # Mín. 20 cm
        "per_extra_20cm":   1,   # +1 por cada 20 cm adicionales
    },
    "mission_3": {
        "box_not_found":    0,
        "box_found":        3,
        "medkit_delivered":  2,
    },
    "mission_4": {
        "no_landing":       0,
        "landing_static":   2,
        "landing_moving":   3,   # +3 extra sobre landing_static → total 5
    },
}

MULTIPLIERS = {
    "none":              1.0,
    "onboard":           1.5,   # Single MAV Onboard Processing
    "swarm_offboard":    2.0,   # Swarm Solution Offboard Processing
    "swarm_onboard":     3.0,   # Swarm Solution Onboard Processing
}

# Tiempo máximo (configurable, el reglamento dice 15-20 min según equipos inscritos)
DEFAULT_MAX_TIME_SECONDS = 20 * 60  # 20 minutos
PREPARATION_TIME_SECONDS = 120      # 2 minutos de preparación antes de arrancar


# ============================================================================
# ORCHESTRATOR CLASS
# ============================================================================
class CompetitionOrchestrator:
    """
    Orquestador de misiones para la competencia TMR 2026.

    Ejecuta las misiones seleccionadas como subprocesos independientes,
    monitoreando su estado por el tópico /mission/status. Entre cada misión
    hace una pausa para que el equipo prepare el dron.

    Lleva el conteo de puntaje según el sistema de scoring del IMAV 2025.
    """

    def __init__(self, sequence, tunnel_difficulty="medium", platform_mode="static",
                 multiplier="none", max_time=DEFAULT_MAX_TIME_SECONDS,
                 skip_preparation=False):
        """
        :param sequence: Lista de keys de misión, e.g. ["mission_1", "mission_2"]
        :param tunnel_difficulty: Dificultad de Mission 1: "free_path", "wide", "medium", "small"
        :param platform_mode: Modo de Mission 4: "static", "moving"
        :param multiplier: Multiplicador de scoring: "none", "onboard", "swarm_offboard", "swarm_onboard"
        :param max_time: Tiempo total máximo en segundos
        :param skip_preparation: Si True, no esperar 2 min de preparación
        """
        rospy.init_node('competition_orchestrator', anonymous=True)

        self.sequence = sequence
        self.tunnel_difficulty = tunnel_difficulty
        self.platform_mode = platform_mode
        self.multiplier = multiplier
        self.max_time = max_time
        self.skip_preparation = skip_preparation

        # Estado
        self.current_mission_idx = 0
        self.mission_completed = False
        self.mission_failed = False
        self.competition_start_time = None
        self.aborted = False

        # Puntaje
        self.scores = {}
        self.total_raw_score = 0

        # Subscriber para status de misiones
        self.status_sub = rospy.Subscriber(
            "/mission/status",
            String,
            self._status_callback
        )

        # Publisher para emergencia (aterrizar)
        self.pub_land = rospy.Publisher('/bebop/land', Empty, queue_size=1)

        # Manejar Ctrl+C limpiamente
        signal.signal(signal.SIGINT, self._signal_handler)

        # Validar secuencia
        self._validate_sequence()

        # Log de configuración
        rospy.loginfo("=" * 60)
        rospy.loginfo("🏆 COMPETITION ORCHESTRATOR — TMR 2026")
        rospy.loginfo("=" * 60)
        rospy.loginfo(f"  Secuencia:        {self.sequence}")
        rospy.loginfo(f"  Túnel (M1):       {self.tunnel_difficulty}")
        rospy.loginfo(f"  Plataforma (M4):  {self.platform_mode}")
        rospy.loginfo(f"  Multiplicador:    {self.multiplier} (×{MULTIPLIERS[self.multiplier]})")
        rospy.loginfo(f"  Tiempo máximo:    {self.max_time // 60} min {self.max_time % 60} s")
        rospy.loginfo("=" * 60)

        # Mostrar advertencias de stubs
        for mk in self.sequence:
            info = MISSION_REGISTRY.get(mk)
            if info and not info["implemented"]:
                rospy.logwarn(
                    f"⚠️  {info['name']} usa un STUB. "
                    f"Requiere implementación de hardware para ser funcional."
                )

    # ──────────────────────────────────────────────────────
    # CALLBACKS
    # ──────────────────────────────────────────────────────

    def _status_callback(self, msg):
        """Callback del tópico /mission/status publicado por cada misión."""
        status = msg.data.strip().lower()
        if status == "done":
            self.mission_completed = True
        elif status == "failed":
            self.mission_failed = True

    def _signal_handler(self, sig, frame):
        """Manejar Ctrl+C para aterrizaje de emergencia."""
        rospy.logerr("\n🛑 CTRL+C detectado — Enviando aterrizaje de emergencia...")
        self.aborted = True
        try:
            self.pub_land.publish(Empty())
        except Exception:
            pass
        self._print_score_summary()
        rospy.signal_shutdown("User abort")
        sys.exit(0)

    # ──────────────────────────────────────────────────────
    # VALIDATION
    # ──────────────────────────────────────────────────────

    def _validate_sequence(self):
        """Validar que todos los scripts existan y sean ejecutables."""
        for mk in self.sequence:
            info = MISSION_REGISTRY.get(mk)
            if info is None:
                rospy.logwarn(f"⚠️  '{mk}' no está registrado en MISSION_REGISTRY. Se omitirá.")
                continue
            if not os.path.exists(info["script"]):
                rospy.logwarn(
                    f"⚠️  Script de {info['name']} no encontrado: {info['script']}"
                )

    # ──────────────────────────────────────────────────────
    # TIME MANAGEMENT
    # ──────────────────────────────────────────────────────

    def _elapsed_competition_time(self):
        """Segundos transcurridos desde que el cronómetro empezó."""
        if self.competition_start_time is None:
            return 0.0
        return (rospy.Time.now() - self.competition_start_time).to_sec()

    def _remaining_time(self):
        """Segundos restantes de la competencia."""
        return max(0.0, self.max_time - self._elapsed_competition_time())

    def _time_exceeded(self):
        """True si se acabó el tiempo."""
        return self._elapsed_competition_time() >= self.max_time

    def _format_time(self, seconds):
        """Formatea segundos en MM:SS."""
        m = int(seconds) // 60
        s = int(seconds) % 60
        return f"{m:02d}:{s:02d}"

    # ──────────────────────────────────────────────────────
    # SCORING
    # ──────────────────────────────────────────────────────

    def _record_score(self, mission_key, points, detail=""):
        """Registrar puntaje de una misión."""
        self.scores[mission_key] = {
            "points": points,
            "detail": detail,
        }
        self.total_raw_score += points
        rospy.loginfo(f"📊 Score para {mission_key}: {points} pts ({detail})")

    def _calculate_mission_1_score(self, completed):
        """Calcular puntaje de Misión 1 según la dificultad del túnel elegido."""
        if not completed:
            self._record_score("mission_1", 0, "No completada")
            return

        pts = SCORING["mission_1"].get(self.tunnel_difficulty, 0)
        self._record_score("mission_1", pts, f"Túnel: {self.tunnel_difficulty}")

    def _calculate_mission_2_score(self, completed):
        """Calcular puntaje de Misión 2."""
        if not completed:
            self._record_score("mission_2", 0, "No completada")
            return

        # El puntaje base es 3 si se logró línea continua mín. 20cm.
        # Los puntos extra por longitud se registran manualmente o por el script.
        pts = SCORING["mission_2"]["continuous_line"]
        self._record_score("mission_2", pts, "Línea continua ≥ 20cm (puntos extra se evalúan por jueces)")

    def _calculate_mission_3_score(self, completed):
        """Calcular puntaje de Misión 3."""
        if not completed:
            self._record_score("mission_3", 0, "No completada")
            return

        # Si completó → caja encontrada + medkit entregado
        pts = SCORING["mission_3"]["box_found"] + SCORING["mission_3"]["medkit_delivered"]
        self._record_score("mission_3", pts, "Caja encontrada + Med-kit entregado")

    def _calculate_mission_4_score(self, completed):
        """Calcular puntaje de Misión 4 según modo de plataforma."""
        if not completed:
            self._record_score("mission_4", 0, "No completada")
            return

        pts = SCORING["mission_4"]["landing_static"]
        detail = "Aterrizaje en plataforma estática"

        if self.platform_mode == "moving":
            pts += SCORING["mission_4"]["landing_moving"]
            detail = "Aterrizaje en plataforma en movimiento"

        self._record_score("mission_4", pts, detail)

    def _auto_score_mission(self, mission_key, completed):
        """Calcular puntaje automático basado en si la misión se completó."""
        if mission_key == "mission_point_to_point":
            self._record_score(mission_key, 0, "Misión de navegación intermedia (sin puntos directos en TMR)")
            return

        scorers = {
            "mission_1": self._calculate_mission_1_score,
            "mission_2": self._calculate_mission_2_score,
            "mission_3": self._calculate_mission_3_score,
            "mission_4": self._calculate_mission_4_score,
        }
        scorer = scorers.get(mission_key)
        if scorer:
            scorer(completed)

    def _print_score_summary(self):
        """Imprimir resumen final de puntaje."""
        mult = MULTIPLIERS.get(self.multiplier, 1.0)
        final_score = self.total_raw_score * mult

        rospy.loginfo("")
        rospy.loginfo("=" * 60)
        rospy.loginfo("📋 RESUMEN DE PUNTAJE — TMR 2026")
        rospy.loginfo("=" * 60)

        for mk, data in self.scores.items():
            info = MISSION_REGISTRY.get(mk, {})
            name = info.get("name", mk)
            rospy.loginfo(f"  {name}: {data['points']} pts — {data['detail']}")

        rospy.loginfo("-" * 60)
        rospy.loginfo(f"  Puntaje bruto:       {self.total_raw_score} pts")
        rospy.loginfo(f"  Multiplicador:       ×{mult} ({self.multiplier})")
        rospy.loginfo(f"  PUNTAJE FINAL:       {final_score:.1f} pts")
        rospy.loginfo(f"  (+ evaluación de presentación: 3 × [0..10] por jueces)")
        rospy.loginfo("=" * 60)
        rospy.loginfo(f"  Tiempo total usado:  {self._format_time(self._elapsed_competition_time())}")
        rospy.loginfo("=" * 60)

    # ──────────────────────────────────────────────────────
    # EXECUTION
    # ──────────────────────────────────────────────────────

    def _preparation_phase(self):
        """Fase de preparación de 2 minutos antes de iniciar el cronómetro."""
        if self.skip_preparation:
            rospy.loginfo("⏩ Saltando fase de preparación (--skip-prep).")
            return

        rospy.loginfo("")
        rospy.loginfo("=" * 60)
        rospy.loginfo("⏱️  FASE DE PREPARACIÓN — 2 minutos")
        rospy.loginfo("   Instálense en la estación de control.")
        rospy.loginfo("   Cuando estén listos, el cronómetro empezará automáticamente.")
        rospy.loginfo("=" * 60)

        prep_start = rospy.Time.now()
        rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - prep_start).to_sec()
            remaining = max(0, PREPARATION_TIME_SECONDS - elapsed)

            if remaining <= 0:
                break

            if int(remaining) % 30 == 0 or remaining <= 10:
                rospy.loginfo(f"  ⏱️  Preparación: {self._format_time(remaining)} restantes")

            rate.sleep()

        rospy.loginfo("✅ Preparación terminada. ¡Comenzando competencia!")

    def _wait_between_missions(self, next_mission_key):
        """
        Pausa entre misiones para que el equipo recoja el dron y lo prepare.
        Según reglamento: un miembro puede entrar a la arena para tomar el dron.
        """
        next_info = MISSION_REGISTRY.get(next_mission_key, {})
        next_name = next_info.get("name", next_mission_key)

        rospy.loginfo("")
        rospy.loginfo("─" * 60)
        rospy.loginfo(f"⏸️  PAUSA — Preparar dron para: {next_name}")
        rospy.loginfo(f"   Tiempo restante de competencia: {self._format_time(self._remaining_time())}")
        rospy.loginfo("   Presiona ENTER cuando estén listos (o espera 30s)...")
        rospy.loginfo("─" * 60)

        # Espera con timeout de 30 segundos (o hasta que el usuario presione Enter)
        pause_start = rospy.Time.now()
        pause_timeout = 30.0  # Max 30 segundos entre misiones

        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - pause_start).to_sec()
            if elapsed >= pause_timeout:
                rospy.loginfo("⏩ Timeout de pausa — continuando automáticamente.")
                break

            if self._time_exceeded():
                rospy.logwarn("⏰ ¡Tiempo de competencia agotado!")
                return False

            rate.sleep()

        return True

    def _run_mission(self, mission_key):
        """
        Ejecutar una misión individual como subproceso.

        Returns:
            bool: True si la misión se completó exitosamente, False si falló.
        """
        info = MISSION_REGISTRY.get(mission_key)

        if info is None:
            rospy.logwarn(f"'{mission_key}' no registrado. Omitiendo.")
            return False

        script_path = info["script"]

        if not os.path.exists(script_path):
            rospy.logwarn(f"Script no encontrado: {script_path}. Omitiendo {info['name']}.")
            return False

        # Verificar tiempo restante
        if self._time_exceeded():
            rospy.logwarn("⏰ Tiempo agotado — no se puede iniciar misión.")
            return False

        rospy.loginfo("")
        rospy.loginfo("=" * 60)
        rospy.loginfo(f"🚀 INICIANDO: {info['name']}")
        rospy.loginfo(f"   {info['description']}")
        rospy.loginfo(f"   Script: {os.path.basename(script_path)}")
        rospy.loginfo(f"   Tiempo restante: {self._format_time(self._remaining_time())}")
        if not info["implemented"]:
            rospy.logwarn(f"   ⚠️  STUB — funcionalidad limitada")
        rospy.loginfo("=" * 60)

        # Resetear flags de estado
        self.mission_completed = False
        self.mission_failed = False

        # Construir argumentos del subproceso
        cmd = [sys.executable, script_path]

        # Pasar parámetros específicos por misión si aplica
        # (los scripts usan rospy.get_param, aquí se podrían agregar _args de ROS)

        # Lanzar subproceso
        try:
            proc = subprocess.Popen(cmd)
        except Exception as e:
            rospy.logerr(f"Error al lanzar {script_path}: {e}")
            return False

        rate = rospy.Rate(10)  # 10 Hz de polling
        mission_start = rospy.Time.now()

        while not rospy.is_shutdown() and not self.aborted:
            # ── Misión publicó "done"
            if self.mission_completed:
                rospy.loginfo(f"✅ {info['name']} — ¡COMPLETADA!")
                elapsed = (rospy.Time.now() - mission_start).to_sec()
                rospy.loginfo(f"   Duración: {self._format_time(elapsed)}")
                self._cleanup_process(proc)
                return True

            # ── Misión publicó "failed"
            if self.mission_failed:
                rospy.logerr(f"❌ {info['name']} — FALLÓ")
                self._cleanup_process(proc)
                return False

            # ── Proceso terminó sin publicar status
            if proc.poll() is not None:
                exit_code = proc.returncode
                if exit_code == 0:
                    rospy.logwarn(
                        f"⚠️  {info['name']} terminó (exit 0) sin publicar 'done'. "
                        f"Se asume completada."
                    )
                    return True
                else:
                    rospy.logerr(
                        f"❌ {info['name']} terminó con error (exit {exit_code})."
                    )
                    return False

            # ── Verificar tiempo total de competencia
            if self._time_exceeded():
                rospy.logwarn(f"⏰ Tiempo de competencia agotado durante {info['name']}.")
                self._cleanup_process(proc)
                return False

            rate.sleep()

        # Si llegamos aquí, es porque se abortó
        self._cleanup_process(proc)
        return False

    def _cleanup_process(self, proc):
        """Terminar subproceso limpiamente si sigue vivo."""
        if proc.poll() is None:
            rospy.loginfo("   Terminando subproceso de misión...")
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                rospy.logwarn("   Subproceso no respondió — forzando kill.")
                proc.kill()
                proc.wait()

    # ──────────────────────────────────────────────────────
    # MAIN RUN
    # ──────────────────────────────────────────────────────

    def run(self):
        """Ejecutar la secuencia completa de competencia."""

        # ── Fase de preparación (2 minutos)
        self._preparation_phase()

        # ── Iniciar cronómetro de competencia
        self.competition_start_time = rospy.Time.now()
        start_wall = datetime.datetime.now().strftime("%H:%M:%S")
        rospy.loginfo(f"\n⏱️  CRONÓMETRO INICIADO a las {start_wall}")
        rospy.loginfo(f"   Tiempo máximo: {self._format_time(self.max_time)}\n")

        # ── Ejecutar misiones en secuencia
        for idx, mission_key in enumerate(self.sequence):
            if rospy.is_shutdown() or self.aborted:
                break

            if self._time_exceeded():
                rospy.logwarn("⏰ Tiempo agotado — no se ejecutarán más misiones.")
                break

            # Pausa entre misiones (excepto antes de la primera)
            if idx > 0:
                can_continue = self._wait_between_missions(mission_key)
                if not can_continue:
                    break

            # Ejecutar misión
            completed = self._run_mission(mission_key)

            # Registrar puntaje
            self._auto_score_mission(mission_key, completed)

            # Breve espera post-misión para cleanup de ROS
            if not rospy.is_shutdown():
                rospy.sleep(2.0)

        # ── Resumen final
        rospy.loginfo("")
        rospy.loginfo("🏆 SECUENCIA DE COMPETENCIA FINALIZADA")
        self._print_score_summary()


# ============================================================================
# CLI ENTRY POINT
# ============================================================================
def main():
    parser = argparse.ArgumentParser(
        description="Orquestador de competencia TMR 2026 — Drones Autónomos (IMAV 2025)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Ejemplos:
  # Ejecutar todas las misiones
  python3 competition_orchestrator.py --combo 1,2,3,4

  # Solo misiones 1 y 2, túnel medio
  python3 competition_orchestrator.py --combo 1,2 --tunnel medium

  # Solo aterrizaje en plataforma en movimiento
  python3 competition_orchestrator.py --combo 4 --platform moving

  # Con multiplicador de procesamiento onboard
  python3 competition_orchestrator.py --combo 1,2,3,4 --multiplier onboard

  # Saltar fase de preparación (para pruebas)
  python3 competition_orchestrator.py --combo 1,2 --skip-prep
        """,
    )

    parser.add_argument(
        '--combo', type=str, default='1,p2p,2,4',
        help="Misiones a ejecutar separadas por coma. Ej: '1,2' o '1,p2p,2,4'"
    )
    parser.add_argument(
        '--tunnel', type=str, default='medium',
        choices=['free_path', 'wide', 'medium', 'small'],
        help="Dificultad del túnel en Misión 1 (default: medium)"
    )
    parser.add_argument(
        '--platform', type=str, default='static',
        choices=['static', 'moving'],
        help="Modo de plataforma en Misión 4 (default: static)"
    )
    parser.add_argument(
        '--multiplier', type=str, default='none',
        choices=['none', 'onboard', 'swarm_offboard', 'swarm_onboard'],
        help="Multiplicador de scoring (default: none)"
    )
    parser.add_argument(
        '--max-time', type=int, default=DEFAULT_MAX_TIME_SECONDS,
        help=f"Tiempo máximo de competencia en segundos (default: {DEFAULT_MAX_TIME_SECONDS})"
    )
    parser.add_argument(
        '--skip-prep', action='store_true',
        help="Saltar los 2 minutos de preparación (para pruebas)"
    )
    parser.add_argument(
        '--list-missions', action='store_true',
        help="Mostrar misiones disponibles y salir"
    )

    args, _ = parser.parse_known_args()

    # ── Mostrar misiones y salir
    if args.list_missions:
        print("\n📋 Misiones disponibles:")
        print("=" * 60)
        for key, info in MISSION_REGISTRY.items():
            status = "✅ Implementada" if info["implemented"] else "⚠️  Stub"
            print(f"  {key}: {info['name']}")
            print(f"         {info['description']}")
            print(f"         Script: {os.path.basename(info['script'])} [{status}]")
            print()
        return

    # ── Parsear secuencia
    seq_list = []
    for m in args.combo.split(','):
        m = m.strip()
        if m:
            if m.lower() in ['p2p', 'nav', 'point_to_point']:
                key = 'mission_point_to_point'
            elif m.isdigit():
                key = f"mission_{m}"
            else:
                key = m

            if key in MISSION_REGISTRY:
                seq_list.append(key)
            else:
                print(f"⚠️  Misión '{m}' no reconocida. Misiones válidas: 1, p2p, 2, 3, 4")

    if not seq_list:
        print("❌ No se especificaron misiones válidas.")
        return

    # ── Ejecutar
    try:
        orchestrator = CompetitionOrchestrator(
            sequence=seq_list,
            tunnel_difficulty=args.tunnel,
            platform_mode=args.platform,
            multiplier=args.multiplier,
            max_time=args.max_time,
            skip_preparation=args.skip_prep,
        )
        orchestrator.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Competencia interrumpida por ROS.")


if __name__ == "__main__":
    main()
