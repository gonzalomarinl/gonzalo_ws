#!/usr/bin/env python3
import rclpy
import argparse
import sys
import math
import yaml
import os
import time
import cv2
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, DurabilityPolicy

# --- IMPORTAMOS TU MÉDICO DE PLANTAS ---
try:
    from sancho_navigation.plant_doctor import analyze_plant
except ImportError:
    try:
        from plant_doctor import analyze_plant
    except:
        print("⚠️ ADVERTENCIA: No se encuentra 'plant_doctor.py'. La IA no funcionará.")
        def analyze_plant(img, name, mode): pass

# --- CLASE PARA GESTIONAR LA CÁMARA ---
class CameraNode(Node):
    def __init__(self, camera_topic):
        super().__init__('camera_subscriber')
        self.bridge = CvBridge()
        self.latest_image = None
        self.image_received = False
        
        # Suscripción corregida según la lista de tópicos real
        print(f"📷 Suscribiéndose al tópico real: {camera_topic}")
        self.subscription = self.create_subscription(
            Image,
            camera_topic, 
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_received = True
        except Exception as e:
            self.get_logger().error(f'Error al convertir imagen: {e}')

    def get_image(self):
        return self.latest_image if self.image_received else None

# --- CLASE DE MISIÓN ---
class GreenhouseMission:
    def __init__(self, mode, camera_node):
        self.mode = mode
        self.camera_node = camera_node
        self.save_dir = os.path.expanduser('~/gonzalo_ws/plant_photos_raw')
        
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
        
        print(f"📂 Carpeta de capturas: {self.save_dir}")

    def perform_detection_logic(self, waypoint_idx):
        print(f"🛑 EJECUTANDO DETECCIÓN {waypoint_idx}...", flush=True)
        
        # Aumentamos los intentos y el timeout para dar tiempo al hardware real
        for _ in range(20): 
            rclpy.spin_once(self.camera_node, timeout_sec=0.2)
            
        frame = self.camera_node.get_image()
        
        if frame is None:
            print("⚠️ CÁMARA NO DISPONIBLE. Verificando buffer...", flush=True)
            import numpy as np
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
        else:
            print("📸 Captura tomada con éxito.", flush=True)

        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"test_captura_{waypoint_idx}_{timestamp}.jpg"
        full_path = os.path.join(self.save_dir, filename)
        cv2.imwrite(full_path, frame)
        
        # Llamada al doctor (con el modelo ya corregido para state_dict)
        analyze_plant(full_path, f"Test_Punto_{waypoint_idx}", self.mode)
        
        print(f"✅ Análisis del Punto {waypoint_idx} finalizada.\n", flush=True)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', type=str, default='sim', choices=['sim', 'real'])
    args, unknown = parser.parse_known_args()
    
    rclpy.init()
    
    # CAMBIO CRÍTICO: El tópico real es /sancho_camera/image_raw
    camera_topic = '/sancho_camera/image_raw' if args.mode == 'real' else '/camera/image_raw'
    
    camera_node = CameraNode(camera_topic)
    mission = GreenhouseMission(args.mode, camera_node)

    print("\n--- 🛠️ MODO SÓLO DETECCIÓN ACTIVADO (TEMPORAL) ---")
    
    try:
        for i in range(1, 6):
            mission.perform_detection_logic(i)
            print("⏳ Esperando 3 segundos para la siguiente prueba...")
            time.sleep(3.0)
            
    except KeyboardInterrupt:
        print("\nPrueba interrumpida.")

    print("🏁 PRUEBA FINALIZADA.", flush=True)
    camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()