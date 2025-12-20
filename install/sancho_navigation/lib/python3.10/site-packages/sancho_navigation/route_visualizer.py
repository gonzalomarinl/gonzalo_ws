#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
import threading
import math
import sys
import yaml
import os

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
        
        # Publicador de marcadores (Bolas Rojas)
        self.marker_pub = self.create_publisher(MarkerArray, 'route_markers', 10)
        self.timer = self.create_timer(1.0, self.publish_markers)
        
        self.waypoints = [] # Aquí guardaremos los puntos calculados
        print("✅ Nodo de Calibración iniciado. Esperando posición del robot...")

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose

    def publish_markers(self):
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

# --- LÓGICA MATEMÁTICA PURA ---
def calculate_row_separation(p1, p2, p3):
    """Calcula la distancia perpendicular exacta entre filas"""
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
        # Centro de la fila 'i'
        cx = p_start[0] + (nx * row_sep * i)
        cy = p_start[1] + (ny * row_sep * i)
        
        # LADO A (Izquierda de la fila)
        start_A_x = cx + (nx * lane_offset)
        start_A_y = cy + (ny * lane_offset)
        
        curr = 0.0
        while curr <= length:
            points.append(Point(x=start_A_x + ux*curr, y=start_A_y + uy*curr, z=0.2))
            curr += step_dist
            
        # LADO B (Derecha de la fila)
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
        print("\n--- 🛠️ CALIBRADOR V3 (AJUSTE FINO) 🛠️ ---")
        
        # 1. CAPTURAR PUNTOS (Si ya te sabes las coordenadas o no quieres mover el robot, 
        #    tendrás que moverlo otra vez, es lo más seguro).
        
        input("👉 1. Robot a INICIO Fila 1 (Eje Central) -> [ENTER]")
        p1 = node.get_current_xy()
        if p1 is None: 
            print("❌ Error: Sin posición AMCL.")
            return
        print(f"   📍 P1: {p1}")

        input("👉 2. Robot a FINAL Fila 1 (Eje Central) -> [ENTER]")
        p2 = node.get_current_xy()
        print(f"   📍 P2: {p2}")

        input("👉 3. Robot a INICIO Fila 2 (Eje Central) -> [ENTER]")
        p3 = node.get_current_xy()
        print(f"   📍 P3: {p3}")

        # Cálculos Automáticos
        sep, u_vec, n_vec, length = calculate_row_separation(p1, p2, p3)
        print(f"\n   📏 Longitud: {length:.2f} m | ↔️ Separación: {abs(sep):.2f} m")

        # 4. DATOS MANUALES
        n_rows = int(input("\n👉 Nº Filas (Ej: 3): "))
        step = float(input("👉 Paso entre fotos (metros) (Ej: 0.8): "))
        
        # --- NUEVA PREGUNTA CRÍTICA ---
        print("\n--- AJUSTE DE SEGURIDAD ---")
        print("Distancia desde el centro de la mata hasta el robot.")
        print(" - 1.4m = Muy separado (peligro de chocar con la otra fila).")
        print(" - 1.0m = Pegadito (mejor para fotos).")
        offset = float(input("👉 Distancia al centro (Ej: 1.0): "))

        # 5. GENERAR
        print("\n🔄 Generando...")
        node.waypoints = calculate_points(p1, u_vec, n_vec, length, sep, n_rows, step, offset)
        
        print(f"✅ ¡Hecho! Revisa RViz.")
        
        save = input("\n💾 ¿Guardar configuración? (s/n): ")
        if save.lower() == 's':
            config_data = {
                'greenhouse_geometry': {
                    'p1_start': list(p1),
                    'p1_end': list(p2),
                    'row_separation': abs(sep),
                    'num_rows': n_rows,
                    'step_distance': step,
                    'lane_offset': offset  # Guardamos tu elección
                }
            }
            path = os.path.expanduser('~/gonzalo_ws/src/sancho_navigation/config/greenhouse_config.yaml')
            with open(path, 'w') as f:
                yaml.dump(config_data, f)
            print(f"💾 Guardado en: {path}")

    except KeyboardInterrupt:
        print("\nSaliendo...")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()