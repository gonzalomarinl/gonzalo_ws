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
        self.declare_parameter('execution_mode', 'simulation') # Opciones: 'simulation' o 'real'
        self.declare_parameter('camera_topic', '/camera/image_raw')
        
        self.mode = self.get_parameter('execution_mode').get_parameter_value().string_value
        self.camera_topic_name = self.get_parameter('camera_topic').get_parameter_value().string_value
        
        self.get_logger().info(f"🤖 INICIANDO EN MODO: {self.mode.upper()}")

        # Configuración de rutas de archivos
        self.image_save_path = os.path.expanduser('~/tfg_ws/plant_photos_results/')
        self.test_images_path = os.path.expanduser('~/tfg_ws/src/sancho_navigation/test_images/') 
        
        if not os.path.exists(self.image_save_path):
            os.makedirs(self.image_save_path)

        # --- 2. CONFIGURACIÓN SEGÚN MODO ---
        self.bridge = CvBridge()
        self.latest_cv_image = None
        self.test_images = []

        if self.mode == 'real':
            # MODO REAL: Nos suscribimos a la cámara física
            self.get_logger().info(f"📷 Conectando a cámara real en: {self.camera_topic_name}")
            self.camera_sub = self.create_subscription(
                Image, 
                self.camera_topic_name, 
                self.camera_callback, 
                10
            )
        else:
            # MODO SIMULACIÓN: Cargamos fotos de prueba
            self.test_images = glob.glob(os.path.join(self.test_images_path, "*.*"))
            if not self.test_images:
                self.get_logger().warn(f"⚠️ NO HAY IMÁGENES EN {self.test_images_path}. La simulación fallará al detectar.")
            else:
                self.get_logger().info(f"📂 Simulación lista con {len(self.test_images)} imágenes de prueba.")

        # --- 3. CARGAR RED NEURONAL ---
        self.get_logger().info("🧠 Cargando Red Neuronal...")
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model_path = os.path.expanduser("~/tfg_ws/src/sancho_navigation/models/tomato_model.pth")
        
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
                self.get_logger().error(f"❌ No encuentro el modelo en {self.model_path}")
        except Exception as e:
            self.get_logger().error(f"❌ Error crítico cargando IA: {e}")

        # --- 4. NAVEGACIÓN ---
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Cargar Ruta YAML
        route_file = os.path.expanduser('~/tfg_ws/src/sancho_navigation/config/my_route.yaml')
        self.waypoints = []
        if os.path.exists(route_file):
            with open(route_file, 'r') as f:
                data = yaml.safe_load(f)
                if data:
                    for p in data:
                        self.waypoints.append([p['x'], p['y'], p['yaw'], p['name']])
            self.get_logger().info(f"✅ Ruta cargada: {len(self.waypoints)} puntos.")
        else:
            self.get_logger().error("❌ No hay archivo de ruta.")
            return 

        self.current_wp_index = 0
        self.nav_to_pose_client.wait_for_server()
        self.get_logger().info("🚀 Iniciando patrulla...")
        self.create_timer(2.0, self.send_next_goal)

    def camera_callback(self, msg):
        # Solo guardamos la última imagen recibida para procesarla cuando lleguemos al waypoint
        try:
            self.latest_cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error frame cámara: {e}")

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

    def send_next_goal(self):
        if hasattr(self, 'timer_wait'): self.timer_wait.cancel()

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

        self.get_logger().info(f"📍 Yendo a: {label}...")
        self.nav_to_pose_client.send_goal_async(goal_msg).add_done_callback(self.goal_accepted_callback)

    def goal_accepted_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted: return
        goal_handle.get_result_async().add_done_callback(self.result_callback)

    def result_callback(self, future):
        status = future.result().status
        if status == 4: # SUCCEEDED
            self.perform_inspection()
        else:
            self.get_logger().warn(f"⚠️ Fallo navegación ({status}). Saltando.")
            self.current_wp_index += 1
            self.send_next_goal()

    def perform_inspection(self):
        current_label = self.waypoints[self.current_wp_index][3]
        
        # Ignorar puntos de giro
        if any(keyword in current_label for keyword in ["Giro", "Transicion", "Entrada"]):
            self.current_wp_index += 1
            self.send_next_goal()
            return

        self.get_logger().info(f"📸 Inspeccionando ({self.mode}): {current_label}...")
        image_to_process = None

        # --- SELECCIÓN DE IMAGEN SEGÚN MODO ---
        if self.mode == 'real':
            if self.latest_cv_image is not None:
                image_to_process = self.latest_cv_image.copy()
            else:
                self.get_logger().warn("⚠️ MODO REAL: No llegan imágenes de la cámara.")
        else:
            # Modo Simulation
            if self.test_images:
                try:
                    img_path = random.choice(self.test_images)
                    image_to_process = cv2.imread(img_path)
                    if image_to_process is None: self.get_logger().error(f"Imagen corrupta: {img_path}")
                except Exception as e:
                    self.get_logger().error(f"Error cargando imagen test: {e}")
            else:
                self.get_logger().warn("⚠️ MODO SIM: No hay imágenes en la carpeta test.")

        # --- PROCESAMIENTO E INFERENCIA ---
        if image_to_process is not None and self.ai_ready:
            try:
                # Inferencia
                pil_img = PILImage.fromarray(cv2.cvtColor(image_to_process, cv2.COLOR_BGR2RGB))
                input_tensor = self.transform(pil_img).unsqueeze(0).to(self.device)
                
                with torch.no_grad():
                    outputs = self.model(input_tensor)
                    probs = torch.nn.functional.softmax(outputs, dim=1)
                    top_prob, top_class = probs.topk(1, dim=1)
                    
                    class_name = self.classes[top_class.item()]
                    confidence = top_prob.item() * 100

                # Logs y Guardado
                log_msg = f"{class_name} ({confidence:.1f}%)"
                if class_name != "Healthy":
                    self.get_logger().error(f"🚨 DETECTADO: {log_msg}")
                    color = (0, 0, 255) # Rojo
                else:
                    self.get_logger().info(f"✅ PLANTA SANA: {log_msg}")
                    color = (0, 255, 0) # Verde

                # Estampar resultado en la foto
                cv2.putText(image_to_process, log_msg, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
                cv2.putText(image_to_process, current_label, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
                filename = f"Analisis_{current_label}_{class_name}.jpg"
                save_path = os.path.join(self.image_save_path, filename)
                cv2.imwrite(save_path, image_to_process)
                self.get_logger().info(f"💾 Evidencia guardada en {filename}")

            except Exception as e:
                self.get_logger().error(f"Error en inferencia: {e}")
        
        # Pausa para simular tiempo de inspección y continuar
        self.timer_wait = self.create_timer(1.0, self.send_next_goal)

def main(args=None):
    rclpy.init(args=args)
    node = GreenhouseNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()