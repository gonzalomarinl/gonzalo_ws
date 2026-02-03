#!/usr/bin/env python3
import rclpy
import argparse
import sys
import math
import yaml
import os
import time
import cv2
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.qos import QoSProfile, DurabilityPolicy

# --- IMPORTACIÓN SEGURA DEL DOCTOR ---
try:
    import sancho_navigation.plant_doctor as doctor
except ImportError:
    try:
        import plant_doctor as doctor
    except:
        print("⚠️ ADVERTENCIA: No se encuentra 'plant_doctor.py'.")
        doctor = None

# --- CLASE CÁMARA ---
class CameraNode(Node):
    def __init__(self, camera_topic):
        super().__init__('camera_subscriber')
        self.bridge = CvBridge()
        self.latest_image = None
        self.image_received = False
        print(f"📷 Suscribiéndose al tópico: {camera_topic}")
        self.subscription = self.create_subscription(Image, camera_topic, self.listener_callback, 10)

    def listener_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_received = True
        except Exception as e:
            self.get_logger().error(f'Error cámara: {e}')

    def get_image(self):
        return self.latest_image if self.image_received else None

# --- CLASE VISUALIZADOR ---
class RouteVisualizer(Node):
    def __init__(self, route_poses):
        super().__init__('mission_visualizer')
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(MarkerArray, 'route_markers', qos)
        self.route = route_poses
        
    def publish_markers(self):
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "mission_route"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0 # Verde
        
        points = [pose.pose.position for pose in self.route]
        marker.points = points
        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

# --- CLASE DE MISIÓN ---
class GreenhouseMission:
    def __init__(self, mode, camera_node):
        self.mode = mode
        self.camera_node = camera_node
        self.config_path = os.path.expanduser('~/gonzalo_ws/src/sancho_navigation/config/greenhouse_config.yaml')
        self.save_dir = os.path.expanduser('~/gonzalo_ws/plant_photos_raw')
        os.makedirs(self.save_dir, exist_ok=True)
        
        self.config = self.load_config()
        if not self.config:
            print("❌ ERROR: No se pudo cargar greenhouse_config.yaml")
            sys.exit(1)

    def load_config(self):
        if not os.path.exists(self.config_path): return None
        with open(self.config_path, 'r') as f:
            data = yaml.safe_load(f)
            return data['greenhouse_geometry']

    def generate_snake_route(self):
        c = self.config
        p1, p2 = c['p1_start'], c['p1_end']
        row_sep, num_rows = c['row_separation'], c['num_rows']
        step_dist, lane_offset = c['step_distance'], c['lane_offset']

        full_route = []
        dx, dy = p2[0]-p1[0], p2[1]-p1[1]
        length = math.sqrt(dx**2 + dy**2)
        ux, uy = dx/length, dy/length
        nx, ny = -uy, ux
        row_yaw = math.atan2(dy, dx)
        
        for i in range(num_rows):
            cx, cy = p1[0] + (nx*row_sep*i), p1[1] + (ny*row_sep*i)
            # Lado A y Lado B con sus respectivos orientaciones (yaw)
            for side in [lane_offset, -lane_offset]:
                yaw = row_yaw - (math.pi/2.0) if side > 0 else row_yaw + (math.pi/2.0)
                q = Quaternion(z=math.sin(yaw/2.0), w=math.cos(yaw/2.0))
                
                side_points = []
                curr = 0.0
                while curr <= length:
                    pose = PoseStamped()
                    pose.header.frame_id = 'map'
                    pose.pose.position.x = cx + (nx*side) + (ux*curr)
                    pose.pose.position.y = cy + (ny*side) + (uy*curr)
                    pose.pose.orientation = q
                    side_points.append(pose)
                    curr += step_dist
                
                if side < 0: side_points.reverse()
                full_route.extend(side_points)
        return full_route

    def perform_detection_logic(self, waypoint_idx):
        print(f"\n🔍 --- INICIANDO ANÁLISIS EN PUNTO {waypoint_idx} ---")
        
        # 1. TIEMPO DE ESTABILIZACIÓN: Esperamos a que el robot deje de vibrar
        print("⏳ Estabilizando robot...")
        time.sleep(2.0) 

        # 2. Refrescar buffer de cámara
        for _ in range(30): 
            rclpy.spin_once(self.camera_node, timeout_sec=0.1)
        
        frame = self.camera_node.get_image()
        if frame is None:
            print("❌ Error: Sin imagen. Usando respaldo negro."); frame = np.zeros((480,640,3), np.uint8)

        # 3. Guardar y Procesar
        temp_path = os.path.join(self.save_dir, f"temp_nav_pt_{waypoint_idx}.jpg")
        cv2.imwrite(temp_path, frame)
        
        if doctor:
            alerta, resultado, confianza = doctor.analyze_plant(temp_path, f"Punto_{waypoint_idx}", self.mode)
            if alerta: print(f"🚨 ¡ALERTA!: {resultado.upper()} ({confianza:.1f}%)")
            else: print(f"✅ PLANTA SANA ({confianza:.1f}%)")
        
        # 4. LIMPIEZA
        if os.path.exists(temp_path): os.remove(temp_path)
        print(f"✅ Punto {waypoint_idx} completado.")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', type=str, default='sim', choices=['sim', 'real'])
    args, _ = parser.parse_known_args()
    rclpy.init()
    
    # Tópicos unificados
    camera_topic = '/astra_camera/camera/color/image_raw' if args.mode == 'real' else '/camera/image_raw'
    
    cam_node = CameraNode(camera_topic)
    mission = GreenhouseMission(args.mode, cam_node)
    route = mission.generate_snake_route()
    
    viz_node = RouteVisualizer(route)
    viz_node.publish_markers()

    navigator = BasicNavigator()
    print("⏳ Esperando a Nav2...")
    navigator.waitUntilNav2Active()

    print(f"🚀 Ruta lista: {len(route)} puntos. ¡Iniciando navegación!")

    for i, goal_pose in enumerate(route):
        print(f"📍 Navegando a punto {i+1}/{len(route)}...")
        viz_node.publish_markers()
        navigator.goToPose(goal_pose)

        while not navigator.isTaskComplete():
            rclpy.spin_once(cam_node, timeout_sec=0.1)

        if navigator.getResult() == TaskResult.SUCCEEDED:
            mission.perform_detection_logic(i+1)
            # Tiempo largo de espera tras cada análisis (opcional, ajusta según necesidad)
            print("⏳ Pausa post-análisis...")
            time.sleep(2.0)
        else:
            print(f"❌ Fallo al llegar al punto {i+1}. Saltando...")

    print("🏁 MISIÓN FINALIZADA.")
    navigator.lifecycleShutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()