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

        # 1. Definir rutas primero
        self.base_path = os.path.expanduser('~/gonzalo_ws/src/sancho_navigation')
        self.route_file = os.path.join(self.base_path, 'config', 'my_route.yaml')
        self.temp_img_path = os.path.join(self.base_path, 'temp_scan.jpg')
        self.doctor_script = os.path.join(self.base_path, 'sancho_navigation', 'plant_doctor.py')

        # 2. INICIALIZAR CLIENTES (Esto corrige tu error de AttributeError)
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()
        self.latest_cv_image = None
        self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)

        # 3. Cargar Waypoints
        self.waypoints = []
        if not self.load_route():
            return
        self.current_wp_index = 0

        # 4. Esperar a Nav2 con seguridad
        self.get_logger().info("⏳ Conectando con servidores de Nav2...")
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("❌ ERROR: Servidores de Nav2 no disponibles. Revisa el terminal de Navigation Launch.")
            return

        self.start_calibration_sequence()

    def load_route(self):
        if not os.path.exists(self.route_file):
            self.get_logger().error(f"❌ No existe: {self.route_file}")
            return False
        with open(self.route_file, 'r') as f:
            data = yaml.safe_load(f)
            if data:
                for p in data:
                    self.waypoints.append({'x': p['x'], 'y': p['y'], 'yaw': math.radians(p['yaw']), 'name': p['name']})
                self.get_logger().info(f"✅ Ruta cargada: {len(self.waypoints)} puntos.")
                return True
        return False

    def camera_callback(self, msg):
        try: self.latest_cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except: pass

    def start_calibration_sequence(self):
        self.get_logger().info("🚀 INICIANDO CALIBRACIÓN POR GIRO...")
        # Inyectar pose inicial
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id, msg.header.stamp = 'map', self.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation.w = -2.0, -0.5, 1.0
        for _ in range(5):
            self.initial_pose_pub.publish(msg)
            time.sleep(0.1)
        
        # Girar 360
        t = Twist()
        t.angular.z = 0.5
        end = time.time() + 15.0
        while time.time() < end:
            self.cmd_vel_pub.publish(t)
            time.sleep(0.1)
        self.cmd_vel_pub.publish(Twist()) # Parar
        self.send_next_goal()

    def send_next_goal(self):
        if self.current_wp_index >= len(self.waypoints):
            self.get_logger().info("🏁 RUTA FINALIZADA.")
            return

        wp = self.waypoints[self.current_wp_index]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = wp['x'], wp['y']
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(wp['yaw']/2), math.cos(wp['yaw']/2)

        self.get_logger().info(f"📍 Yendo a {wp['name']}...")
        self.nav_to_pose_client.send_goal_async(goal).add_done_callback(self.goal_response)

    def goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.current_wp_index += 1
            self.send_next_goal()
            return
        handle.get_result_async().add_done_callback(self.result_callback)

    def result_callback(self, future):
        if future.result().status == 4: # Éxito
            self.execute_detection()
        else:
            self.current_wp_index += 1
            self.send_next_goal()

    def execute_detection(self):
        wp_name = self.waypoints[self.current_wp_index]['name']
        if self.latest_cv_image is not None:
            cv2.imwrite(self.temp_img_path, self.latest_cv_image)
        
        try:
            subprocess.run(["python3", self.doctor_script, "--image", self.temp_img_path, "--name", wp_name], check=True)
        except: pass
        
        self.current_wp_index += 1
        self.send_next_goal()

def main(args=None):
    rclpy.init(args=args)
    node = GreenhouseNavigator()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    if rclpy.ok(): rclpy.shutdown()

if __name__ == "__main__":
    main()