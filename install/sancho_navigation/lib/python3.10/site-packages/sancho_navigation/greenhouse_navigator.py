#!/usr/bin/env python3
import rclpy
import argparse
import sys
import math
import yaml
import os
import time
import cv2
import threading
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.qos import QoSProfile, DurabilityPolicy

# --- IMPORTAMOS TU MÉDICO DE PLANTAS ---
# Asegúrate de que plant_doctor.py está en la misma carpeta o instalado como paquete
try:
    from sancho_navigation.plant_doctor import analyze_plant
except ImportError:
    # Fallback por si ejecutas localmente sin instalar el paquete completo
    try:
        from plant_doctor import analyze_plant
    except:
        print("⚠️ ADVERTENCIA: No se encuentra 'plant_doctor.py'. La IA no funcionará.")
        def analyze_plant(img, name, mode): pass

# --- CLASE PARA GESTIONAR LA CÁMARA (NUEVO) ---
class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.bridge = CvBridge()
        self.latest_image = None
        self.image_received = False
        
        # Suscripción al topic de la cámara de Gazebo
        # Ajusta '/camera/image_raw' si tu robot usa otro topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        try:
            # Convertimos de mensaje ROS a imagen OpenCV
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_received = True
        except Exception as e:
            self.get_logger().error(f'Error al convertir imagen: {e}')

    def get_image(self):
        return self.latest_image if self.image_received else None

# --- CLASE VISUALIZADOR (BOLAS VERDES) ---
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
        marker.color.r = 0.0 
        marker.color.g = 1.0 # VERDE
        marker.color.b = 0.0
        
        points = []
        for pose_stamped in self.route:
            p = pose_stamped.pose.position
            points.append(Point(x=p.x, y=p.y, z=0.2))
            
        marker.points = points
        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

# --- CLASE DE MISIÓN ---
class GreenhouseMission:
    def __init__(self, mode, camera_node):
        self.mode = mode
        self.camera_node = camera_node # Referencia al nodo de cámara
        self.config_path = os.path.expanduser('~/gonzalo_ws/src/sancho_navigation/config/greenhouse_config.yaml')
        self.save_dir = os.path.expanduser('~/gonzalo_ws/plant_photos_raw')
        
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
        
        print(f"📂 Cargando configuración desde: {self.config_path}")
        self.config = self.load_config()
        
        if not self.config:
            print("❌ ERROR CRÍTICO: No se pudo cargar greenhouse_config.yaml.")
            sys.exit(1)

    def load_config(self):
        try:
            if not os.path.exists(self.config_path): return None
            with open(self.config_path, 'r') as f:
                data = yaml.safe_load(f)
                return data['greenhouse_geometry']
        except Exception as e:
            print(f"Error leyendo YAML: {e}")
            return None

    def get_quaternion_from_yaw(self, yaw):
        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def generate_snake_route(self):
        c = self.config
        p1 = c['p1_start']
        p2 = c['p1_end']
        row_sep = c['row_separation']
        num_rows = c['num_rows']
        step_dist = c['step_distance']
        lane_offset = c['lane_offset']

        full_route = []
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        length = math.sqrt(dx**2 + dy**2)
        if length == 0: return []

        ux = dx / length
        uy = dy / length
        nx = -uy
        ny = ux
        row_yaw = math.atan2(dy, dx)
        
        yaw_A = row_yaw - (math.pi / 2.0)
        yaw_B = row_yaw + (math.pi / 2.0)

        print(f"📐 Generando ruta: {num_rows} filas | Longitud: {length:.2f}m")

        for i in range(num_rows):
            cx = p1[0] + (nx * row_sep * i)
            cy = p1[1] + (ny * row_sep * i)
            # LADO A
            start_A_x = cx + (nx * lane_offset)
            start_A_y = cy + (ny * lane_offset)
            points_A = []
            curr = 0.0
            while curr <= length:
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = start_A_x + (ux * curr)
                pose.pose.position.y = start_A_y + (uy * curr)
                pose.pose.orientation = self.get_quaternion_from_yaw(yaw_A)
                points_A.append(pose)
                curr += step_dist
            full_route.extend(points_A)
            # LADO B
            start_B_x = cx - (nx * lane_offset)
            start_B_y = cy - (ny * lane_offset)
            points_B = []
            curr = 0.0
            while curr <= length:
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = start_B_x + (ux * curr)
                pose.pose.position.y = start_B_y + (uy * curr)
                pose.pose.orientation = self.get_quaternion_from_yaw(yaw_B)
                points_B.append(pose)
                curr += step_dist
            points_B.reverse()
            full_route.extend(points_B)
        return full_route

    def perform_detection_logic(self, waypoint_idx):
        print(f"🛑 PARADA {waypoint_idx}: Iniciando protocolo de inspección...", flush=True)
        
        # 1. Obtener imagen de la cámara
        frame = self.camera_node.get_image()
        
        if frame is None:
            print("⚠️ CÁMARA NO DISPONIBLE. Usando imagen negra de prueba.", flush=True)
            # Crear imagen negra si falla la cámara para no romper el programa
            import numpy as np
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
        else:
            print("📸 Captura de cámara recibida correctamente.", flush=True)

        # 2. Guardar la imagen cruda (evidencia)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"captura_pt{waypoint_idx}_{timestamp}.jpg"
        full_path = os.path.join(self.save_dir, filename)
        cv2.imwrite(full_path, frame)
        
        # 3. LLAMAR AL DOCTOR (Aquí es donde ocurre la magia de la IA)
        # Esto llamará a tu script plant_doctor.py, que tiene el time.sleep(10)
        # simulando el procesamiento de la red neuronal.
        analyze_plant(full_path, f"Punto_{waypoint_idx}", self.mode)
        
        print(f"✅ Inspección del Punto {waypoint_idx} finalizada. Continuando ruta.\n", flush=True)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', type=str, default='sim', choices=['sim', 'real'])
    args, unknown = parser.parse_known_args()
    
    rclpy.init()
    
    # 1. Iniciamos el nodo de cámara en un hilo separado para que siempre tenga la última foto
    camera_node = CameraNode()
    cam_thread = threading.Thread(target=rclpy.spin, args=(camera_node,), daemon=True)
    cam_thread.start()
    print("📷 Nodo de cámara iniciado y escuchando...")

    # 2. Preparar Misión
    mission = GreenhouseMission(args.mode, camera_node)
    route = mission.generate_snake_route()
    
    # 3. Visualizador
    viz_node = RouteVisualizer(route)
    viz_node.publish_markers() # Publicar una vez
    print("👀 Visualizador de ruta activado (Bolas Verdes).")

    # 4. Nav2
    navigator = BasicNavigator()
    print("⏳ Esperando a Nav2...", flush=True)
    navigator.waitUntilNav2Active()

    print(f"✅ Ruta calculada: {len(route)} paradas. ¡Despegamos!", flush=True)
    
    # 5. EJECUCIÓN
    for i, goal_pose in enumerate(route):
        print(f"🚀 [{i+1}/{len(route)}] Yendo al objetivo...", flush=True)
        
        # Refrescamos visualización
        viz_node.publish_markers()
        
        navigator.goToPose(goal_pose)

        while not navigator.isTaskComplete():
            pass

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            # --- AQUÍ OCURRE LA DETECCIÓN REAL ---
            mission.perform_detection_logic(i+1)
        else:
            print(f"❌ Fallo en punto {i+1}. Abortando.")
            navigator.lifecycleShutdown()
            viz_node.destroy_node()
            camera_node.destroy_node()
            exit(1)

    print("🏁 ¡MISIÓN COMPLETADA!", flush=True)
    navigator.lifecycleShutdown()
    viz_node.destroy_node()
    camera_node.destroy_node()
    exit(0)

if __name__ == '__main__':
    main()