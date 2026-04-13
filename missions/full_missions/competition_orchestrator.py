#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Orchestrator for Full Missions (Torneo Mexicano de Robótica 2026)
This script allows executing combinations of missions sequentially.
Based on the IMAV 2025 rulebook adapted for TMR 2026.

Available Missions:
1. Enter the room (mission_orange_window.py)
2. Draw on the whiteboard (mission_whiteboard_aruco.py)
3. Find the cry for help sound and deliver a med-kit (mission_medkit.py) - To be implemented
4. Land on a moving platform with smoke (mission_platform_landing.py) - To be implemented
"""

import os
import sys
import time
import subprocess
import rospy
from std_msgs.msg import String

# Definimos que misiones existen actualmente y su ruta
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
MISSIONS_DIR = os.path.abspath(os.path.join(CURRENT_DIR, '..'))

AVAILABLE_MISSIONS = {
    "mission_1": os.path.join(MISSIONS_DIR, "mission_orange_window.py"),
    "mission_2": os.path.join(MISSIONS_DIR, "mission_whiteboard_aruco.py"),
    "mission_3": os.path.join(MISSIONS_DIR, "mission_medkit.py"), # Placeholder
    "mission_4": os.path.join(MISSIONS_DIR, "mission_platform_landing.py") # Placeholder
}

class MissionOrchestrator:
    def __init__(self, sequence):
        """
        :param sequence: List of mission keys (e.g., ["mission_1", "mission_2"])
        """
        rospy.init_node('competition_orchestrator', anonymous=True)
        self.sequence = sequence
        self.current_mission_idx = 0
        self.mission_completed = False
        self.mission_failed = False
        
        # Suscribirnos al tópico para saber cuando una misión termina
        self.status_sub = rospy.Subscriber(
            "/mission/status", 
            String, 
            self.status_callback
        )
        
        rospy.loginfo("Mission Orchestrator Initialized.")
        rospy.loginfo(f"Planned Sequence: {self.sequence}")

    def status_callback(self, msg):
        status = msg.data.lower()
        if status == "done":
            self.mission_completed = True
        elif status == "failed":
            self.mission_failed = True

    def run(self):
        for mission_key in self.sequence:
            script_path = AVAILABLE_MISSIONS.get(mission_key)
            
            if not script_path or not os.path.exists(script_path):
                rospy.logwarn(f"Script for {mission_key} not found at {script_path}. Skipping.")
                continue
                
            rospy.loginfo(f"==================================================")
            rospy.loginfo(f"🚀 STARTING {mission_key.upper()}...")
            rospy.loginfo(f"Script: {script_path}")
            rospy.loginfo(f"==================================================")
            
            self.mission_completed = False
            self.mission_failed = False
            
            # Lanzar la misión como subproceso
            # Use sys.executable (e.g. python3) to execute the python script
            proc = subprocess.Popen([sys.executable, script_path])
            
            rate = rospy.Rate(10)
            
            # Esperar a que la misión publique "done" o termine el proceso natural
            while not rospy.is_shutdown():
                # Revisar si publico éxito
                if self.mission_completed:
                    rospy.loginfo(f"✅ {mission_key} finished successfully!")
                    break
                
                # Revisar si publico fallo
                if self.mission_failed:
                    rospy.logerr(f"❌ {mission_key} failed! Aborting sequence.")
                    proc.terminate() # Terminar el subproceso actual
                    return # Cancelar el resto
                
                # Revisar si el proceso se cayo/termino inesperadamente
                if proc.poll() is not None:
                    rospy.logwarn(f"⚠️ {mission_key} process terminated without publishing 'done'.")
                    break
                    
                rate.sleep()
                
            # Limpieza: Asegurarnos de cerrar el subproceso antes de la siguiente misión
            if proc.poll() is None:
                rospy.loginfo("Terminating the previous mission process to free resources...")
                proc.terminate()
                proc.wait() # Esperar a que cierre
                
            rospy.loginfo("Waiting 3 seconds before next mission...")
            time.sleep(3.0)
            
        rospy.loginfo("🏆 ORCHESTRATOR SEQUENCE COMPLETED!")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Run combinations of TMR missions.")
    parser.add_argument('--combo', type=str, default='1,2', help="Comma-separated mission numbers (e.g., '1,2' or '1,2,3,4')")
    
    args, unknown = parser.parse_known_args()
    
    # Parse requested sequence
    seq_list = [f"mission_{m.strip()}" for m in args.combo.split(',') if m.strip()]
    
    try:
        orchestrator = MissionOrchestrator(seq_list)
        orchestrator.run()
    except rospy.ROSInterruptException:
        pass
