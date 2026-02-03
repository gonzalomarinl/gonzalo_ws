#!/usr/bin/env python3
import rclpy
import argparse
import os
import time
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# IMPORTACIÓN SEGURA PARA EVITAR ERROR DE ATRIBUTO
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
        print(f"\n🔍 --- INICIANDO ANÁLISIS EN PUNTO {waypoint_idx} ---")
        # Esperamos a que lleguen frames frescos
        for _ in range(25): 
            rclpy.spin_once(self.camera_node, timeout_sec=0.1)
        
        frame = self.camera_node.get_image()
        if frame is None:
            print("❌ Error: No se reciben imágenes del tópico /sancho_camera/image_raw"); return

        # Guardamos la imagen temporalmente
        temp_path = os.path.join(self.save_dir, f"temp_pt_{waypoint_idx}.jpg")
        cv2.imwrite(temp_path, frame)
        print("📸 Captura realizada.")
        
        # LLAMADA A LA IA
        alerta, resultado, confianza = doctor.analyze_plant(temp_path, f"Punto_{waypoint_idx}", self.mode)
        
        # MOSTRAR RESULTADO AL OPERARIO
        if alerta:
            print(f"🚨 ¡ALERTA DETECTADA!: {resultado.upper()} ({confianza:.1f}%)")
        else:
            print(f"✅ PLANTA SANA ({confianza:.1f}%)")
        
        # BORRADO DE LA IMAGEN CAPTURADA PARA NO LLENAR EL DISCO
        if os.path.exists(temp_path):
            os.remove(temp_path)
            print("🗑️ Imagen temporal borrada.")
        
        print(f"✅ Procesamiento del Punto {waypoint_idx} completado.")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', type=str, default='sim')
    args, _ = parser.parse_known_args()
    rclpy.init()
    
    # Tópico real corregido
    camera_topic = '/sancho_camera/image_raw' if args.mode == 'real' else '/camera/image_raw'
    
    node = CameraNode(camera_topic)
    mission = GreenhouseMission(args.mode, node)

    print(f"🚀 Modo {args.mode.upper()} activo. Iniciando secuencia de prueba...")
    try:
        # Hacemos 3 capturas de prueba
        for i in range(1, 4):
            mission.perform_detection_logic(i)
            time.sleep(3)
    except KeyboardInterrupt: pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()