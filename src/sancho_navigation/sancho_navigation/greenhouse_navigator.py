#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import yaml
import math
import os
import time
import subprocess
import cv2

class GreenhouseNavigator(Node):
    def __init__(self):
        super().__init__('greenhouse_navigator')

        # --- 1. CONFIGURACIÓN DE RUTAS ---
        self.base_path = os.path.expanduser('~/gonzalo_ws/src/sancho_navigation')
        self.route_file = os.path.join(self.base_path, 'config', 'my_route.yaml')
        self.temp_img_path = os.path.join(self.base_path, 'temp_scan.jpg')
        self.doctor_script = os.path.join(self.base_path, 'sancho_navigation', 'plant_doctor.py')

        # --- 2. COMUNICACIONES ROS 2 ---
        # Publicador para la posición inicial (Coincide con nav2_params.yaml)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        
        # Publicador de velocidad directa (Para el giro de calibración inicial)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Cliente de acción para navegación
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Suscripción a cámara para capturar la imagen antes del análisis
        self.bridge = CvBridge()
        self.latest_cv_image = None
        self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)

        # --- 3. CARGA DE WAYPOINTS ---
        self.waypoints = []
        if self.load_route():
            self.get_logger().info(f"✅ Ruta cargada con {len(self.waypoints)} puntos.")
        else:
            return

        self.current_wp_index = 0

        # --- 4. SECUENCIA DE ARRANQUE ---
        self.get_logger().info("⏳ Esperando a que los servidores de Nav2 estén listos...")
        self.nav_to_pose_client.wait_for_server()
        
        # Ejecutamos la calibración antes de movernos
        self.start_calibration_sequence()

    def load_route(self):
        if not os.path.exists(self.route_file):
            self.get_logger().error(f"❌ No se encuentra el archivo de ruta: {self.route_file}")
            return False
        with open(self.route_file, 'r') as f:
            data = yaml.safe_load(f)
            if data:
                for p in data:
                    self.waypoints.append({
                        'x': p['x'], 
                        'y': p['y'], 
                        'yaw': math.radians(p['yaw']), 
                        'name': p['name']
                    })
                return True
        return False

    def camera_callback(self, msg):
        try:
            self.latest_cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error en cámara: {e}")

    def start_calibration_sequence(self):
        """Fase de pre-vuelo: Inyección de pose y giro de 360º para AMCL"""
        self.get_logger().info("🚀 INICIANDO FASE DE CALIBRACIÓN...")

        # 1. Inyectar posición inicial (según nav2_params.yaml)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = -2.0  
        msg.pose.pose.position.y = -0.5
        msg.pose.pose.orientation.w = 1.0
        
        for _ in range(5):
            self.initial_pose_pub.publish(msg)
            time.sleep(0.1)
        self.get_logger().info("📌 Posición inicial inyectada (-2.0, -0.5).")

        # 2. Giro de 360 grados (Tu idea para localizarse)
        self.get_logger().info("🔄 Girando 360º para ajustar localización (15s)...")
        twist = Twist()
        twist.angular.z = 0.4  # Velocidad de giro lenta
        
        start_time = time.time()
        while (time.time() - start_time) < 15.0:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
            
        # Detener el robot
        self.cmd_vel_pub.publish(Twist())
        self.get_logger().info("✅ Localización convergida. Iniciando patrulla.")
        
        # 3. Comenzar navegación
        self.send_next_goal()

    def send_next_goal(self):
        if self.current_wp_index >= len(self.waypoints):
            self.get_logger().info("🏁 MISIÓN FINALIZADA.")
            return

        wp = self.waypoints[self.current_wp_index]
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = wp['x']
        goal_msg.pose.pose.position.y = wp['y']
        
        # Orientación
        goal_msg.pose.pose.orientation.z = math.sin(wp['yaw'] / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(wp['yaw'] / 2.0)

        self.get_logger().info(f"📍 Yendo a {wp['name']}...")
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("⚠️ Meta rechazada por Nav2. Reintentando siguiente...")
            self.current_wp_index += 1
            self.send_next_goal()
            return

        self.get_logger().info("⛽ En camino...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        if status == 4: # STATUS_SUCCEEDED
            self.get_logger().info("🎯 ¡Llegamos al waypoint!")
            self.execute_external_detection()
        else:
            self.get_logger().warn(f"⚠️ Error en navegación (Status {status}).")
            self.current_wp_index += 1
            self.send_next_goal()

    def execute_external_detection(self):
        """Lanza el script de IA y detiene la navegación hasta que termine"""
        wp_name = self.waypoints[self.current_wp_index]['name']
        
        # 1. Capturar y guardar imagen actual
        if self.latest_cv_image is not None:
            cv2.imwrite(self.temp_img_path, self.latest_cv_image)
            self.get_logger().info(f"📸 Foto capturada para análisis en {wp_name}.")
        else:
            self.get_logger().error("❌ No hay señal de cámara para el análisis.")

        # 2. Llamada al script externo plant_doctor.py (Modularidad)
        self.get_logger().info(f"🧬 Iniciando análisis médico en {wp_name}...")
        try:
            # Esta llamada bloquea el flujo del navegador durante los 10s de la detección
            subprocess.run([
                "python3", 
                self.doctor_script, 
                "--image", self.temp_img_path, 
                "--name", wp_name,
                "--mode", self.mode #modo de simulación o real
            ], check=True)
        except Exception as e:
            self.get_logger().error(f"❌ Fallo al ejecutar plant_doctor.py: {e}")

        # 3. Una vez terminado el script externo, avanzamos al siguiente punto
        self.current_wp_index += 1
        self.send_next_goal()

def main(args=None):
    rclpy.init(args=args)
    navigator = GreenhouseNavigator()
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()