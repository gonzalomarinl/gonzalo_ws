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

# --- IMPORTACIÓN CORREGIDA ---
try:
    # Intentamos importar el módulo instalado
    import sancho_navigation.plant_doctor as doctor
    analyze_plant = doctor.analyze_plant
except ImportError:
    try:
        # Intentamos importación local directa
        import plant_doctor
        analyze_plant = plant_doctor.analyze_plant
    except Exception as e:
        print(f"❌ ERROR CRÍTICO: No se pudo cargar plant_doctor.py: {e}")
        def analyze_plant(img, name, mode): return False, "IA no disponible"

class CameraNode(Node):
    def __init__(self, camera_topic):
        super().__init__('camera_subscriber')
        self.bridge = CvBridge()
        self.latest_image = None
        self.image_received = False
        self.subscription = self.create_subscription(Image, camera_topic, self.listener_callback, 10)

    def listener_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_received = True
        except Exception as e:
            self.get_logger().error(f'Error: {e}')

    def get_image(self):
        return self.latest_image if self.image_received else None

class GreenhouseMission:
    def __init__(self, mode, camera_node):
        self.mode = mode
        self.camera_node = camera_node
        self.save_dir = os.path.expanduser('~/gonzalo_ws/plant_photos_raw')
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

    def perform_detection_logic(self, waypoint_idx):
        print(f"\n🔍 --- INICIANDO ANÁLISIS EN PUNTO {waypoint_idx} ---", flush=True)
        
        # Sincronización de cámara
        for _ in range(20): 
            rclpy.spin_once(self.camera_node, timeout_sec=0.1)
            
        frame = self.camera_node.get_image()
        
        if frame is None:
            print("❌ ERROR: Cámara no responde.")
            return

        # Guardar imagen temporalmente para la IA
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"raw_pt_{waypoint_idx}_{timestamp}.jpg"
        full_path = os.path.join(self.save_dir, filename)
        cv2.imwrite(full_path, frame)
        print(f"📸 Captura realizada.")

        # --- LLAMADA A LA IA Y RESULTADO ---
        # analyze_plant devuelve (es_anomalia, diagnostico)
        alerta, resultado = analyze_plant(full_path, f"Punto_{waypoint_idx}", self.mode)
        
        print(f"🧠 DIAGNÓSTICO IA: {resultado.upper()}")
        
        # --- LIMPIEZA ---
        if os.path.exists(full_path):
            os.remove(full_path) # Borramos la imagen pesada original
            print(f"🗑️ Imagen temporal borrada.")
        
        print(f"✅ Punto {waypoint_idx} procesado.\n")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', type=str, default='sim', choices=['sim', 'real'])
    args, unknown = parser.parse_known_args()
    rclpy.init()
    
    # Tópico real detectado en tus capturas
    camera_topic = '/sancho_camera/image_raw' if args.mode == 'real' else '/camera/image_raw'
    
    camera_node = CameraNode(camera_topic)
    mission = GreenhouseMission(args.mode, camera_node)

    print(f"🚀 Iniciando modo: {args.mode.upper()}")
    try:
        for i in range(1, 4):
            mission.perform_detection_logic(i)
            time.sleep(2)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()