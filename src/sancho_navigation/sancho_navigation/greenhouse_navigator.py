#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import math
import os
import yaml
import random
import glob
import time  # <--- IMPORTANTE: Necesario para evitar bucles infinitos rápidos
import urllib.request
from datetime import datetime

# --- LIBRERÍAS DE IA ---
import torch
import timm
from PIL import Image as PILImage
from torchvision import transforms

class GreenhouseNavigator(Node):
    def __init__(self):
        super().__init__('greenhouse_navigator')

        # --- 1. CONFIGURACIÓN Y PARÁMETROS ---
        self.declare_parameter('execution_mode', 'simulation')
        self.declare_parameter('camera_topic', '/camera/image_raw')
        
        self.mode = self.get_parameter('execution_mode').get_parameter_value().string_value
        self.camera_topic_name = self.get_parameter('camera_topic').get_parameter_value().string_value
        
        self.get_logger().info(f"🤖 INICIANDO EN MODO: {self.mode.upper()}")

        self.base_path = os.path.expanduser('~/gonzalo_ws/src/sancho_navigation')
        self.image_save_path = os.path.expanduser('~/gonzalo_ws/plant_photos_results/')
        self.test_images_path = os.path.join(self.base_path, 'test_images')
        self.model_path = os.path.join(self.base_path, 'models', 'tomato_model.pth')
        self.route_file = os.path.join(self.base_path, 'config', 'my_route.yaml')

        # Directorios
        if not os.path.exists(self.image_save_path):
            os.makedirs(self.image_save_path)
        
        if not os.path.exists(self.test_images_path):
            os.makedirs(self.test_images_path)
            try:
                url_tomate = "https://upload.wikimedia.org/wikipedia/commons/thumb/8/89/Tomato_je.jpg/320px-Tomato_je.jpg"
                img_dest = os.path.join(self.test_images_path, "tomate_test.jpg")
                urllib.request.urlretrieve(url_tomate, img_dest)
            except:
                pass

        # --- 2. CONFIGURACIÓN IMAGEN ---
        self.bridge = CvBridge()
        self.latest_cv_image = None
        self.test_images = []

        if self.mode == 'real':
            self.create_subscription(Image, self.camera_topic_name, self.camera_callback, 10)
        else:
            self.test_images = glob.glob(os.path.join(self.test_images_path, "*.*"))
            self.get_logger().info(f"📂 Imágenes de prueba encontradas: {len(self.test_images)}")

        # --- 3. CARGAR IA ---
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.classes = ['Bacterial Spot', 'Early_Blight', 'Healthy', 'Late_blight', 'Leaf Mold', 'Target_Spot', 'Black Spot']
        self.ai_ready = False

        try:
            self.model = timm.create_model('resnet18', pretrained=False, num_classes=len(self.classes))
            if os.path.exists(self.model_path):
                state_dict = torch.load(self.model_path, map_location=self.device)
                self.model.load_state_dict(state_dict)
                self.model.to(self.device)
                self.model.eval()
                
                self.transform = transforms.Compose([
                    transforms.Resize((224, 224)),
                    transforms.ToTensor(),
                    transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
                ])
                self.ai_ready = True
                self.get_logger().info("✅ IA Cargada correctamente.")
            else:
                self.get_logger().warn(f"⚠️ No se encontró modelo en {self.model_path}. Se simulará detección.")
        except Exception as e:
            self.get_logger().error(f"❌ Error IA: {e}")

        # --- 4. NAVEGACIÓN ---
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.waypoints = []
        if os.path.exists(self.route_file):
            with open(self.route_file, 'r') as f:
                data = yaml.safe_load(f)
                if data:
                    for p in data:
                        self.waypoints.append([p['x'], p['y'], p['yaw'], p['name']])
            self.get_logger().info(f"✅ Ruta cargada: {len(self.waypoints)} puntos.")
        else:
            self.get_logger().error("❌ No se encontró archivo de ruta yaml.")
            return 

        self.current_wp_index = 0
        self.nav_to_pose_client.wait_for_server()
        
        # Esperar un poco a que todo arranque
        time.sleep(2.0)
        self.get_logger().info("🚀 Iniciando patrulla...")
        self.send_next_goal()

    def camera_callback(self, msg):
        try:
            self.latest_cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except:
            pass

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

    def send_next_goal(self):
        if self.current_wp_index >= len(self.waypoints):
            self.get_logger().info("🏁 RUTA COMPLETADA.")
            return

        wp = self.waypoints[self.current_wp_index]
        x, y, yaw_deg, label = wp[0], wp[1], wp[2], wp[3]

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
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
            self.get_logger().warn("⚠️ Meta rechazada. Saltando punto.")
            self.retry_next_point()
            return
        
        res_future = goal_handle.get_result_async()
        res_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        status = future.result().status
        # Status 4 = SUCCEEDED
        if status == 4: 
            self.perform_inspection()
        else:
            # CORRECCIÓN: Si falla, esperamos un poco y saltamos al siguiente
            self.get_logger().warn(f"⚠️ Fallo navegación (Status: {status}). Saltando al siguiente en 1s...")
            self.retry_next_point()

    def retry_next_point(self):
        # Esta pausa evita que el terminal se bloquee si hay muchos fallos seguidos
        time.sleep(1.0) 
        self.current_wp_index += 1
        self.send_next_goal()

    def perform_inspection(self):
        current_label = self.waypoints[self.current_wp_index][3]
        
        # Si es un punto de giro, no tomamos foto
        if any(keyword in current_label for keyword in ["Giro", "Transicion", "Entrada"]):
            self.current_wp_index += 1
            self.send_next_goal()
            return

        self.get_logger().info(f"📸 Inspeccionando: {current_label}...")
        
        # Simulación de proceso de imagen (rápido)
        image_to_process = None
        if self.mode == 'simulation' and self.test_images:
            try:
                img_path = random.choice(self.test_images)
                image_to_process = cv2.imread(img_path)
            except: pass
        elif self.latest_cv_image is not None:
             image_to_process = self.latest_cv_image.copy()

        # Guardado simple si hay imagen
        if image_to_process is not None:
            try:
                timestamp = datetime.now().strftime("%H%M%S")
                filename = f"Analisis_{current_label}_{timestamp}.jpg"
                save_path = os.path.join(self.image_save_path, filename)
                
                # Escribir texto en imagen
                cv2.putText(image_to_process, f"Punto: {current_label}", (10,30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
                cv2.imwrite(save_path, image_to_process)
                self.get_logger().info(f"💾 Guardado: {filename}")
            except Exception as e:
                self.get_logger().error(f"Error guardando: {e}")
        
        # Pausa para simular análisis IA
        time.sleep(0.5)
        
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