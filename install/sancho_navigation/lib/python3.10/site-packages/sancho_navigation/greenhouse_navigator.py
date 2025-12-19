#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseWithCovarianceStamped
import yaml
import math
import os
import time

class GreenhouseNavigator(Node):
    def __init__(self):
        super().__init__('greenhouse_navigator')

        # --- 1. CARGAR RUTA ---
        self.base_path = os.path.expanduser('~/gonzalo_ws/src/sancho_navigation')
        self.route_file = os.path.join(self.base_path, 'config', 'my_route.yaml')
        self.waypoints = []

        if os.path.exists(self.route_file):
            with open(self.route_file, 'r') as f:
                data = yaml.safe_load(f)
                if data:
                    for p in data:
                        # Guardamos x, y, yaw, nombre
                        self.waypoints.append([p['x'], p['y'], p['yaw'], p['name']])
            self.get_logger().info(f"✅ Ruta cargada: {len(self.waypoints)} puntos.")
        else:
            self.get_logger().error(f"❌ No encuentro el archivo: {self.route_file}")
            return 

        # --- 2. CLIENTES DE NAVEGACIÓN ---
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # Publicador para forzar la posición inicial y arreglar el fallo de localización
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        
        self.current_wp_index = 0

        # --- 3. SECUENCIA DE ARRANQUE ---
        self.get_logger().info("⏳ Esperando a que Nav2 esté listo...")
        self.nav_to_pose_client.wait_for_server()
        
        # FIX CRÍTICO: Sincronizar Gazebo con RViz
        self.set_initial_pose()
        
        # Esperamos un poco para que el robot se asiente en el mapa
        time.sleep(2.0)
        
        self.get_logger().info("🚀 Iniciando patrulla simple...")
        self.send_next_goal()

    def set_initial_pose(self):
        """Le dice a Nav2 dónde está el robot realmente para que no se pierda"""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # COORDENADAS DE SPAWN (Las de tu nav2_params.yaml)
        msg.pose.pose.position.x = -2.0  
        msg.pose.pose.position.y = -0.5
        msg.pose.pose.orientation.w = 1.0 # Orientación neutra
        
        self.get_logger().info(f"📌 Forzando localización en x={msg.pose.pose.position.x}, y={msg.pose.pose.position.y}")
        
        # Publicamos varias veces para asegurar que AMCL lo recibe
        for _ in range(5):
            self.initial_pose_pub.publish(msg)
            time.sleep(0.2)

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

    def send_next_goal(self):
        if self.current_wp_index >= len(self.waypoints):
            self.get_logger().info("🏁 RUTA COMPLETADA. Trabajo terminado.")
            return

        # Obtener datos del siguiente punto
        wp = self.waypoints[self.current_wp_index]
        x, y, yaw_deg, label = wp[0], wp[1], wp[2], wp[3]

        # Crear mensaje de meta
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        
        # Convertir grados a cuaternión (orientación)
        q = self.get_quaternion_from_euler(0.0, 0.0, math.radians(yaw_deg))
        goal_msg.pose.pose.orientation.x = q[0]
        goal_msg.pose.pose.orientation.y = q[1]
        goal_msg.pose.pose.orientation.z = q[2]
        goal_msg.pose.pose.orientation.w = q[3]

        self.get_logger().info(f"📍 Yendo a: {label} (x={x:.2f}, y={y:.2f})...")
        
        future = self.nav_to_pose_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_accepted_callback)

    def goal_accepted_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("⚠️ Meta rechazada. Saltando punto...")
            self.current_wp_index += 1
            self.send_next_goal()
            return
        
        res_future = goal_handle.get_result_async()
        res_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        status = future.result().status
        
        if status == 4: # STATUS_SUCCEEDED
            self.get_logger().info("✅ Llegada confirmada.")
            self.perform_detection()
        else:
            self.get_logger().warn(f"⚠️ Fallo en navegación (Status: {status}). Saltando...")
            self.current_wp_index += 1
            self.send_next_goal()

    def perform_detection(self):
        # LÓGICA SIMPLE: PARAR Y SIMULAR DETECCIÓN
        self.get_logger().info("🔍 Ejecutando sistema de detección (10 segundos)...")
        
        # Pausa simple (bloqueante, pero segura para scripts sencillos)
        time.sleep(10.0) 
        
        self.get_logger().info("📸 Detección terminada. Avanzando...")
        self.current_wp_index += 1
        self.send_next_goal()

def main(args=None):
    rclpy.init(args=args)
    node = GreenhouseNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Deteniendo nodo...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()