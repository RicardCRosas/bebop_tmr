#!/usr/bin/env python3
# mission_supervisor_2.py

import rospy
import subprocess
from enum import Enum
from std_msgs.msg import String


class GlobalState(Enum):
    IDLE = 0
    TAKEOFF = 1
    MISSION = 2
    TELEOP = 3
    LANDING = 4

class MissionSupervisor:

    def __init__(self, movements):

        self.movements = movements
        self.state = GlobalState.IDLE

        self.current_process = None
        self.current_mission = None

        self.mission_start_time = None
        self.max_mission_time = 20.0  # Timeout global

        # Suscriptor al estado de misión
        rospy.Subscriber("/mission/status", String, self.mission_status_callback)
        
        #self.state_pub = rospy.Publisher("/supervisor/state", String, queue_size=1)
        self.state_pub = rospy.Publisher(
            "/supervisor/state",
            String,
            queue_size=1,
            latch=True
        )

        self.timer = rospy.Timer(rospy.Duration(0.05), self.timer_callback)

        self.last_state = self.state 

        self.state_start_time = None

        rospy.loginfo("Mission Supervisor Ready")


    # =====================================================
    # INTERFAZ EXTERNA (desde teleop)
    # =====================================================

    def start_mission(self, mission_name):
        rospy.logwarn(f"Current state before mission: {self.state}")
        if self.state != GlobalState.IDLE:
            rospy.logwarn("Cannot start mission: not in IDLE")
            return
        self.current_mission = mission_name
        self.state = GlobalState.TAKEOFF # comentado para pruebas
        #self.state = GlobalState.MISSION # Quitar despues de las pruebas de controller con odometria

    def start_teleop(self):
        if self.state != GlobalState.IDLE:
            rospy.logwarn("Cannot start teleop: not in IDLE")
            return
        self.state = GlobalState.TAKEOFF

    def request_landing(self):
        if self.state in [GlobalState.MISSION, GlobalState.TELEOP, GlobalState.TAKEOFF]:
            self.state = GlobalState.LANDING

    def abort_mission(self):  
        rospy.logwarn("Mission aborted")

        if self.state == GlobalState.MISSION:
            self.stop_mission()
            self.force_zero_velocity()
            self.state = GlobalState.LANDING

    # 🔥 EMERGENCY = evento que fuerza LANDING
    def emergency(self):
        rospy.logwarn("🚨 EMERGENCY STOP ACTIVATED")
        rospy.logwarn("Press 'z' to deactivate the emergency stop")

        # 1 Matar misión inmediatamente
        self.stop_mission()

        # 2 Cortar movimiento. Forzar velocidad cero
        self.force_zero_velocity()

        # 3 Limpiar misión
        self.current_mission = None
        self.mission_start_time = None

        # 4 Forzar transición directa a LANDING
        self.state_start_time = None
        self.state = GlobalState.LANDING


    def is_teleop_active(self):
        return self.state == GlobalState.TELEOP


    # =====================================================
    # FSM 
    # =====================================================

    def update(self):

        if self.state == GlobalState.IDLE:
            pass

        elif self.state == GlobalState.TAKEOFF:
            self.handle_takeoff()

        elif self.state == GlobalState.MISSION:
            self.check_timeout()

        elif self.state == GlobalState.TELEOP:
            pass

        elif self.state == GlobalState.LANDING:
            self.handle_landing()


        self.publish_state()  # 🔥 Siempre publica estado actual


    # =====================================================
    # STATE HANDLERS
    # =====================================================

    def handle_takeoff(self):
        if self.state_start_time is None:
            rospy.loginfo("TAKEOFF")
            self.movements.takeoff()
            self.state_start_time = rospy.Time.now()
            return
        
        elapsed = (rospy.Time.now() - self.state_start_time).to_sec()   

        if elapsed > 3.0:

            if self.current_mission:
                self.launch_mission()
                self.mission_start_time = rospy.Time.now()
                self.state = GlobalState.MISSION
            else:
                self.state = GlobalState.TELEOP

            self.state_start_time = None


    def handle_landing(self):
        if self.state_start_time is None:
            rospy.loginfo("LANDING")
            self.force_zero_velocity() # Asegurar velocidad cero al aterrizar
            self.movements.land()
            self.state_start_time = rospy.Time.now()
            return

        elapsed = (rospy.Time.now() - self.state_start_time).to_sec()

        if elapsed > 3.0:
            self.current_mission = None
            self.mission_start_time = None
            self.state = GlobalState.IDLE
            self.state_start_time = None


    
    # =====================================================
    # MISSION MANAGEMENT
    # =====================================================

    def launch_mission(self):
        rospy.loginfo(f"Launching mission: {self.current_mission}")

        self.current_process = subprocess.Popen(
            ["rosrun", "bebop_tmr", f"{self.current_mission}.py"]
        )


    def stop_mission(self):
        if self.current_process:
            try:
                self.current_process.kill()  # kill inmediato
            except Exception:
                pass
            self.current_process = None

    def force_zero_velocity(self):
        self.movements.reset_twist()

    def mission_status_callback(self, msg):

        # Solo reaccionar si realmente hay misión corriendo
        if self.state != GlobalState.MISSION:
            return

        if msg.data == "done":
            rospy.loginfo("Mission reported DONE")
            self.stop_mission()
            self.force_zero_velocity()
            self.state = GlobalState.LANDING
            #self.state = GlobalState.IDLE

        elif msg.data == "failed":
            rospy.logwarn("Mission reported FAILED → EMERGENCY")
            self.stop_mission()
            self.force_zero_velocity()
            self.state = GlobalState.LANDING


    def check_timeout(self):
        if not self.mission_start_time:
            return

        elapsed = (rospy.Time.now() - self.mission_start_time).to_sec()

        if elapsed > self.max_mission_time:
            rospy.logwarn("MISSION TIMEOUT")
            self.stop_mission()
            self.force_zero_velocity()
            self.state = GlobalState.LANDING

    def publish_state(self):
        if self.state != self.last_state:
            rospy.loginfo(f"STATE → {self.state.name}")
            self.state_pub.publish(self.state.name)
            self.last_state = self.state

    def timer_callback(self, event):
        self.update()


# =====================================================
# TEST NODE (solo para validación independiente)
# VERSIÓN CORRECTA PARA VALIDAR SUPERVISOR SOLO
# =====================================================

#if __name__ == "__main__":

#    rospy.init_node("mission_supervisor_test")

    # ============================
    # Mock de movimientos
    # ============================

#    class DummyMovements:
#        def initial_takeoff(self, mode):
#            rospy.loginfo("[MOCK] Takeoff")

#        def landing(self, mode):
#            rospy.loginfo("[MOCK] Landing")

#    movements = DummyMovements()
#    supervisor = MissionSupervisor(movements)

#    rate = rospy.Rate(10)  # 10 Hz

#    rospy.loginfo("Supervisor test running...")

    # Lanzar misión automáticamente después de 2 segundos
#    rospy.sleep(2)
#    supervisor.start_mission("mission_square")

#    while not rospy.is_shutdown():
#        supervisor.update()
#        rate.sleep()


