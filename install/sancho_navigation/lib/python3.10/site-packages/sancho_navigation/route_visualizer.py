#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Twist
from visualization_msgs.msg import Marker, MarkerArray
import threading
import math
import sys
import yaml
import os
import time

class InteractiveRouteMaker(Node):
    def __init__(self):
        super().__init__('interactive_route_maker')
        
        # Suscripción a la posición del robot (AMCL)
        self.current_pose = None
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10)
        
        # Publicador de velocidad para el giro inicial
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Publicador de marcadores (Bolas Rojas)
        self.marker_pub = self.create_publisher(MarkerArray, 'route_markers', 10)
        self.timer = self.create_timer(1.0, self.publish_markers)
        
        self.waypoints = [] # Aquí guardaremos los puntos calculados
        print("✅ Nodo de Calibración iniciado. Esperando posición del robot...")

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose

    def publish_markers(self):
        # Siempre publicamos, aunque esté vacío (para borrar si hiciera falta)
        if not self.waypoints:
            return

        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "calibrated_route"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0 # ROJO
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.points = self.waypoints
        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

    def get_current_xy(self):
        if self.current_pose is None:
            return None
        return (self.current_pose.position.x, self.current_pose.position.y)

    def perform_localization_spin(self):
        """Hace girar al robot 360 grados para despertar a AMCL"""
        print("\n🌀 Iniciando GIRO DE LOCALIZACIÓN (10 seg)...")
        twist = Twist()
        twist.angular.z = 0.5 # Velocidad de giro suave
        
        start_time = time.time()
        while time.time() - start_time < 10.0:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
            
        # Parar
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        print("✅ Giro completado. AMCL debería estar fino ahora.\n")

# --- LÓGICA MATEMÁTICA PURA ---
def calculate_row_separation(p1, p2, p3):
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    length = math.sqrt(dx**2 + dy**2)
    
    if length == 0: return 0.0, (0,0), (0,0), 0.0
    
    ux = dx / length
    uy = dy / length
    
    # Vector Normal (Perpendicular) (-y, x)
    nx = -uy
    ny = ux
    
    # Vector desde P1 a P3 (Inicio F1 a Inicio F2)
    v13_x = p3[0] - p1[0]
    v13_y = p3[1] - p1[1]
    
    # Proyección para sacar la distancia perpendicular
    separation = (v13_x * nx) + (v13_y * ny)
    
    return separation, (ux, uy), (nx, ny), length

def calculate_points(p_start, u_vec, n_vec, length, row_sep, num_rows, step_dist, lane_offset):
    points = []
    ux, uy = u_vec
    nx, ny = n_vec
    
    if row_sep < 0:
        row_sep = abs(row_sep)
        nx = -nx
        ny = -ny

    for i in range(num_rows):
        cx = p_start[0] + (nx * row_sep * i)
        cy = p_start[1] + (ny * row_sep * i)
        
        # LADO A
        start_A_x = cx + (nx * lane_offset)
        start_A_y = cy + (ny * lane_offset)
        
        curr = 0.0
        while curr <= length:
            points.append(Point(x=start_A_x + ux*curr, y=start_A_y + uy*curr, z=0.2))
            curr += step_dist
            
        # LADO B
        start_B_x = cx - (nx * lane_offset)
        start_B_y = cy - (ny * lane_offset)
        
        curr = 0.0
        while curr <= length:
            points.append(Point(x=start_B_x + ux*curr, y=start_B_y + uy*curr, z=0.2))
            curr += step_dist
            
    return points

def main():
    rclpy.init()
    node = InteractiveRouteMaker()
    
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    try:
        print("\n--- 🛠️ CALIBRADOR V4 (CON GIRO Y RE-AJUSTE) 🛠️ ---")
        
        # 0. FASE DE LOCALIZACIÓN
        do_spin = input("¿Quieres girar para localizar el robot antes de empezar? (s/n): ")
        if do_spin.lower() == 's':
            node.perform_localization_spin()
        
        # 1. CAPTURAR PUNTOS
        print("📍 Mueve el robot con '2D Pose Estimate' o Teleop.")
        input("👉 1. Pon el robot en INICIO Fila 1 (Centro Pasillo) -> [ENTER]")
        p1 = node.get_current_xy()
        if p1 is None: 
            print("❌ Error: Sin posición AMCL. ¿Has lanzado la localización?")
            return
        print(f"   📍 P1 capturado: {p1}")

        input("👉 2. Pon el robot en FINAL Fila 1 (Centro Pasillo) -> [ENTER]")
        p2 = node.get_current_xy()
        print(f"   📍 P2 capturado: {p2}")

        input("👉 3. Pon el robot en INICIO Fila 2 (Centro Pasillo) -> [ENTER]")
        p3 = node.get_current_xy()
        print(f"   📍 P3 capturado: {p3}")

        # Cálculos Automáticos
        sep, u_vec, n_vec, length = calculate_row_separation(p1, p2, p3)
        print(f"\n   📏 Longitud Fila: {length:.2f} m | ↔️ Separación Filas: {abs(sep):.2f} m")

        # 4. DATOS ESTÁTICOS
        n_rows = int(input("\n👉 Nº Filas a inspeccionar (Ej: 3): "))
        step = float(input("👉 Distancia entre fotos (metros) (Ej: 1.0): "))
        
        # 5. BUCLE DE AJUSTE FINO (NUEVO)
        while True:
            print("\n--- AJUSTE DE SEGURIDAD (Visualiza en RViz) ---")
            print("Distancia desde el centro de la mata hasta el robot.")
            offset = float(input("👉 Distancia al centro (Prueba 1.0, 1.2...): "))

            print("🔄 Generando ruta en RViz...")
            node.waypoints = calculate_points(p1, u_vec, n_vec, length, sep, n_rows, step, offset)
            
            # Pausa breve para asegurar que se publica
            time.sleep(0.5)
            print("👀 ¡Mira las bolas rojas en RViz!")
            
            ok = input("¿Te gusta cómo queda? (s = Guardar y Salir / n = Probar otra distancia): ")
            if ok.lower() == 's':
                break
            else:
                print("🔁 Repitiendo ajuste...")

        # 6. GUARDAR
        config_data = {
            'greenhouse_geometry': {
                'p1_start': list(p1),
                'p1_end': list(p2),
                'row_separation': abs(sep),
                'num_rows': n_rows,
                'step_distance': step,
                'lane_offset': offset
            }
        }
        path = os.path.expanduser('~/gonzalo_ws/src/sancho_navigation/config/greenhouse_config.yaml')
        with open(path, 'w') as f:
            yaml.dump(config_data, f)
        print(f"💾 ¡Configuración guardada en: {path}!")

    except KeyboardInterrupt:
        print("\nSaliendo...")
    finally:
        # Detener robot por si acaso
        stop_twist = Twist()
        node.cmd_vel_pub.publish(stop_twist)
        rclpy.shutdown()

if __name__ == '__main__':
    main()