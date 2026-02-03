#!/usr/bin/env python3
import rclpy
import argparse
import os
import time
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# IMPORTACIÓN SEGURA
try:
    import sancho_navigation.plant_doctor as doctor
except ImportError:
    import plant_doctor as doctor

class CameraNode(Node):
    def __init__(self, camera_topic):
        super().__init__('camera_subscriber')
        self.bridge = CvBridge()
        self.latest_image = None
        self.subscription = self.create_subscription(Image, camera_topic, self.listener_callback, 10)

    def listener_callback(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def get_image(self):
        return self.latest_image

class GreenhouseMission:
    def __init__(self, mode, camera_node):
        self.mode = mode
        self.camera_node = camera_node
        self.save_dir = os.path.expanduser('~/gonzalo_ws/plant_photos_raw')
        os.makedirs(self.save_dir, exist_ok=True)

    def perform_detection_logic(self, waypoint_idx):
        print(f"\n🔍 ANALIZANDO PUNTO {waypoint_idx}...")
        for _ in range(20): rclpy.spin_once(self.camera_node, timeout_sec=0.1)
        
        frame = self.camera_node.get_image()
        if frame is None:
            print("❌ Error: No hay imagen."); return

        temp_path = os.path.join(self.save_dir, f"temp_{waypoint_idx}.jpg")
        cv2.imwrite(temp_path, frame)
        
        # LLAMADA A LA IA
        alerta, resultado = doctor.analyze_plant(temp_path, f"Punto_{waypoint_idx}", self.mode)
        print(f"🧠 RESULTADO: {resultado.upper()}")
        
        if os.path.exists(temp_path): os.remove(temp_path) # BORRADO TRAS ANÁLISIS

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', type=str, default='sim')
    args, _ = parser.parse_known_args()
    rclpy.init()
    
    # Tópico real de Sancho
    camera_topic = '/sancho_camera/image_raw' if args.mode == 'real' else '/camera/image_raw'
    
    node = CameraNode(camera_topic)
    mission = GreenhouseMission(args.mode, node)

    try:
        for i in range(1, 4):
            mission.perform_detection_logic(i)
            time.sleep(2)
    except KeyboardInterrupt: pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()