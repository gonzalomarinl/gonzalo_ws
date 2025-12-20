#!/usr/bin/env python3
import rclpy
import argparse
import sys
import math
import yaml
import os
import time
# Quitamos threading para evitar el conflicto
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from visualization_msgs.msg import Marker, MarkerArray
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration

# --- CLASE VISUALIZADOR (SIMPLIFICADA) ---
class RouteVisualizer(Node):
    def __init__(self, route_poses):
        super().__init__('mission_visualizer')
        # QoS Durability Transient Local: Para que el mensaje se quede "pegado" en RViz
        # aunque publiquemos solo una vez.
        from rclpy.qos import QoSProfile, DurabilityPolicy
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

# --- CLASE DE MISIÓN (INTACTA) ---
class GreenhouseMission:
    def __init__(self, mode):
        self.mode = mode
        self.config_path = os.path.expanduser('~/gonzalo_ws/src/sancho_navigation/config/greenhouse_config.yaml')
        
        print(f"📂 Cargando configuración desde: {self.config_path}")
        self.config = self.load_config()
        
        if not self.config:
            print("❌ ERROR CRÍTICO: No se pudo cargar greenhouse_config.yaml.")
            sys.exit(1)
        else:
            print("✅ Configuración cargada correctamente.")

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

    def perform_detection(self, waypoint_idx):
        msg = f"📸 [MODO {self.mode.upper()}] Punto {waypoint_idx}: Capturando..."
        print(msg, flush=True)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', type=str, default='sim', choices=['sim', 'real'])
    args, unknown = parser.parse_known_args()
    
    rclpy.init()
    
    # 1. Preparar Misión
    mission = GreenhouseMission(args.mode)
    route = mission.generate_snake_route()
    
    # 2. INICIO VISUALIZADOR (Sin Hilos / Threads)
    viz_node = RouteVisualizer(route)
    # Publicamos una vez al inicio
    viz_node.publish_markers()
    print("👀 Visualizador de ruta activado (Bolas Verdes).")

    # 3. Preparar Nav2
    navigator = BasicNavigator()
    print("⏳ Esperando a Nav2...", flush=True)
    navigator.waitUntilNav2Active()

    print(f"✅ Ruta calculada: {len(route)} paradas. ¡Despegamos!", flush=True)
    
    # 4. EJECUCIÓN
    for i, goal_pose in enumerate(route):
        print(f"🚀 [{i+1}/{len(route)}] Navegando...", flush=True)
        
        # [Truco] Refrescamos las bolas verdes en cada vuelta por si RViz se ha reiniciado
        viz_node.publish_markers() 
        
        navigator.goToPose(goal_pose)

        while not navigator.isTaskComplete():
            pass

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            mission.perform_detection(i+1)
        else:
            print(f"❌ Fallo en punto {i+1}. Abortando.")
            navigator.lifecycleShutdown()
            # Limpieza antes de salir
            viz_node.destroy_node()
            exit(1)

    print("🏁 ¡MISIÓN COMPLETADA!", flush=True)
    navigator.lifecycleShutdown()
    viz_node.destroy_node()
    exit(0)

if __name__ == '__main__':
    main()