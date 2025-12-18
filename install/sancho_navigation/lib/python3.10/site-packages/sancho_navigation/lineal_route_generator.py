#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Twist
import cv2
import numpy as np
import yaml
import os
import math
import time
import sys

# --- FUNCIONES DE AYUDA ROS & MATEMÁTICAS ---

def get_quaternion_from_euler(yaw):
    """Convierte ángulo yaw a cuaternio para ROS 2"""
    qx = 0.0
    qy = 0.0
    qz = math.sin(yaw / 2)
    qw = math.cos(yaw / 2)
    return [qx, qy, qz, qw]

def pixel_to_world(px, py, origin_x, origin_y, resolution, height_px):
    """Convierte píxeles de imagen a coordenadas de mundo (metros)"""
    # En imágenes, Y crece hacia abajo. En ROS map, Y crece hacia arriba.
    wx = origin_x + (px * resolution)
    wy = origin_y + ((height_px - py) * resolution)
    return wx, wy

# --- FASE 1: LOCALIZACIÓN (EL BAILE) ---

def perform_localization_spin():
    print("=================================================")
    print("   FASE 1: AUTO-LOCALIZACIÓN (GIRO 360º)         ")
    print("=================================================")
    print("🔄 Conectando con el robot para iniciar giros...")
    print("   MIRA RVIZ: Las flechas rojas deben concentrarse.")
    
    rclpy.init()
    node = rclpy.create_node('localization_spinner')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    
    twist = Twist()
    twist.angular.z = 0.5  # Velocidad de giro (rad/s)
    
    stop_twist = Twist()
    
    duration = 10.0 # Segundos de giro
    start_time = time.time()
    
    try:
        while time.time() - start_time < duration:
            pub.publish(twist)
            time.sleep(0.1)
            remaining = int(duration - (time.time() - start_time))
            print(f"   🌀 Girando... {remaining}s restantes", end='\r')
            
        # Parada
        print("\n🛑 Deteniendo robot...")
        for _ in range(10):
            pub.publish(stop_twist)
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        pub.publish(stop_twist)
        
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("✅ LOCALIZACIÓN COMPLETADA. Pasando a cálculo de ruta...\n")

# --- FASE 2: VISIÓN ARTIFICIAL (LA RUTA) ---

def generate_route_cv():
    print("=================================================")
    print("   FASE 2: GENERACIÓN DE RUTA (OPENCV)           ")
    print("=================================================")

    # 1. CONFIGURACIÓN
    base_path = os.path.expanduser('~/gonzalo_ws/src/sancho_navigation/maps/')
    map_yaml_name = "greenhouse_map.yaml" 
    yaml_path = os.path.join(base_path, map_yaml_name)

    if not os.path.exists(yaml_path):
        print(f"❌ Error: No encuentro el mapa en {yaml_path}")
        return

    # Leer metadatos del mapa
    with open(yaml_path, 'r') as f:
        map_data = yaml.safe_load(f)
        image_name = map_data['image']
        resolution = float(map_data['resolution'])
        origin = map_data['origin']

    pgm_path = os.path.join(base_path, image_name)
    img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
    
    if img is None:
        print("❌ Error cargando la imagen PGM.")
        return
    
    height, width = img.shape

    # 2. PARÁMETROS
    print("--- PARÁMETROS DE SEGURIDAD ---")
    try:
        dist_seguridad_m = float(input("1. Distancia de seguridad a la planta (metros, ej: 0.6): "))
        dist_waypoints_m = float(input("2. Separación entre fotos (metros, ej: 0.5): "))
        min_contour_area = 100 # Filtro de ruido fijo para simplificar
    except ValueError:
        return

    dist_seguridad_px = int(dist_seguridad_m / resolution)
    step_px = int(dist_waypoints_m / resolution)

    # 3. PROCESADO
    _, binary = cv2.threshold(img, 200, 255, cv2.THRESH_BINARY_INV)
    
    # Dilatación (Engordar obstáculos)
    kernel = np.ones((3,3), np.uint8)
    dilated = cv2.dilate(binary, kernel, iterations=dist_seguridad_px)
    
    # Obtener contornos (Ruta)
    contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    waypoints_list = []
    vis_img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    row_count = 0

    for cnt in contours:
        if cv2.contourArea(cnt) < min_contour_area: continue
        row_count += 1
        
        # Suavizar contorno
        epsilon = 0.005 * cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, epsilon, True)
        
        cv2.drawContours(vis_img, [approx], -1, (0, 255, 0), 2) # Ruta en Verde

        # Generar puntos
        for i in range(len(approx)):
            p1 = approx[i][0]
            p2 = approx[(i + 1) % len(approx)][0]
            
            dist_segment = np.linalg.norm(p1 - p2)
            num_steps = int(dist_segment / step_px)
            if num_steps < 1: continue

            for j in range(num_steps):
                alpha = j / num_steps
                x_px = int(p1[0] * (1 - alpha) + p2[0] * alpha)
                y_px = int(p1[1] * (1 - alpha) + p2[1] * alpha)

                # Visualizar punto
                cv2.circle(vis_img, (x_px, y_px), 3, (0, 0, 255), -1)

                # Convertir a Metros
                wx, wy = pixel_to_world(x_px, y_px, origin[0], origin[1], resolution, height)
                
                # Orientación (Mirando a la planta)
                dx = p2[0] - p1[0]
                dy = p2[1] - p1[1]
                yaw_ruta = math.atan2(-dy, dx)
                yaw_look = yaw_ruta + (math.pi / 2)

                waypoints_list.append({
                    'name': f"Fila{row_count}_P{len(waypoints_list)}",
                    'x': round(wx, 3),
                    'y': round(wy, 3),
                    'yaw': round(math.degrees(yaw_look), 1)
                })

# 4. RESULTADO
    print(f"\n✅ Ruta calculada: {len(waypoints_list)} puntos.")
    print("👀 Se abrirá una ventana para verificar. PULSA UNA TECLA PARA GUARDAR.")
    
    cv2.namedWindow("Ruta Visual", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Ruta Visual", 800, 600)
    cv2.imshow("Ruta Visual", vis_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # --- GUARDAR YAML (DATOS) ---
    output_path = os.path.expanduser('~/gonzalo_ws/src/sancho_navigation/config/my_route.yaml')
    with open(output_path, 'w') as f:
        yaml.dump(waypoints_list, f, sort_keys=False)
    print(f"💾 Ruta (YAML) guardada en: {output_path}")

    # --- NUEVO: GUARDAR IMAGEN (FOTO) ---
    img_output_path = os.path.expanduser('~/gonzalo_ws/src/sancho_navigation/config/ruta_generada.jpg')
    cv2.imwrite(img_output_path, vis_img)
    print(f"🖼️ Imagen guardada en: {img_output_path}")

if __name__ == "__main__":
    perform_localization_spin() 
    generate_route_cv()