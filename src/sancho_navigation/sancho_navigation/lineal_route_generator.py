#!/usr/bin/env python3
import cv2
import numpy as np
import yaml
import os
import math

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
    # Fórmula estándar de ROS Map Server
    wx = origin_x + (px * resolution)
    wy = origin_y + ((height_px - py) * resolution)
    return wx, wy

def lineal_route_generator():
    print("=================================================")
    print("   GENERADOR DE RUTAS POR VISIÓN ARTIFICIAL      ")
    print("=================================================")
    print("Este script analiza la imagen del mapa y genera")
    print("líneas de navegación paralelas a los obstáculos.")
    print("-------------------------------------------------")

    # --- 1. CONFIGURACIÓN ---
    base_path = os.path.expanduser('~/gonzalo_ws/src/sancho_navigation/maps/')
    map_yaml_name = "greenhouse_map.yaml" # Asegúrate que este es tu mapa
    yaml_path = os.path.join(base_path, map_yaml_name)

    if not os.path.exists(yaml_path):
        print(f"❌ Error: No encuentro el mapa en {yaml_path}")
        return

    # Leer metadatos del mapa
    with open(yaml_path, 'r') as f:
        map_data = yaml.safe_load(f)
        image_name = map_data['image']
        resolution = float(map_data['resolution']) # metros/pixel
        origin = map_data['origin'] # [x, y, z]

    pgm_path = os.path.join(base_path, image_name)
    print(f"📂 Cargando mapa: {pgm_path}")
    print(f"   - Resolución: {resolution} m/px")
    print(f"   - Origen: {origin}")

    # --- 2. LEER IMAGEN ---
    img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        print("❌ Error cargando la imagen PGM. Revisa la ruta.")
        return
    
    height, width = img.shape

    # --- 3. PARÁMETROS DE USUARIO ---
    print("\n--- CONFIGURACIÓN DE LA RUTA ---")
    try:
        dist_seguridad_m = float(input("1. ¿A qué distancia de la planta quieres navegar? (metros, ej: 0.6): "))
        dist_waypoints_m = float(input("2. ¿Cada cuántos metros quieres una parada (foto)? (metros, ej: 0.5): "))
        min_contour_area = float(input("3. Filtro de ruido (Tamaño mín planta, ej: 100): "))
    except ValueError:
        print("❌ Error: Introduce números válidos.")
        return

    # Convertir metros a píxeles
    dist_seguridad_px = int(dist_seguridad_m / resolution)
    step_px = int(dist_waypoints_m / resolution)
    
    print(f"\n⚙️  Procesando: Margen de {dist_seguridad_px} px alrededor de las plantas...")

    # --- 4. PROCESADO DE IMAGEN (LA MAGIA) ---
    # Invertir: En PGM, blanco (255) es libre, negro (0) es ocupado.
    # OpenCV trabaja mejor con Objetos Blancos sobre fondo negro.
    # Así que invertimos: Plantas = Blanco, Pasillo = Negro.
    _, binary = cv2.threshold(img, 200, 255, cv2.THRESH_BINARY_INV)

    # DILATACIÓN: "Engordamos" las plantas por la distancia de seguridad.
    # El borde de la planta engordada será nuestra ruta.
    kernel = np.ones((3,3), np.uint8)
    # Iteraciones define cuánto engordamos (radio)
    dilated = cv2.dilate(binary, kernel, iterations=dist_seguridad_px)

    # Encontrar contornos (Bordes de la zona engordada)
    contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    waypoints_list = []
    
    # Imagen para visualizar (Color)
    vis_img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    print(f"🔍 Detectadas {len(contours)} posibles filas.")

    row_count = 0
    for cnt in contours:
        # Filtrar ruido pequeño
        if cv2.contourArea(cnt) < min_contour_area:
            continue
        
        row_count += 1
        
        # Simplificar el contorno para que sean líneas rectas y no zig-zag pixelado
        epsilon = 0.01 * cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, epsilon, True)

        # Dibujar la ruta propuesta en VERDE
        cv2.drawContours(vis_img, [approx], -1, (0, 255, 0), 2)

        # Generar puntos sobre este contorno
        # Recorremos los puntos del polígono simplificado
        for i in range(len(approx)):
            p1 = approx[i][0]
            p2 = approx[(i + 1) % len(approx)][0] # Siguiente punto (cerrando bucle)
            
            # Distancia entre vértices
            dist_segment = np.linalg.norm(p1 - p2)
            num_steps = int(dist_segment / step_px)
            
            if num_steps < 1: continue

            # Interpolación lineal entre vértices
            for j in range(num_steps):
                alpha = j / num_steps
                # Punto interpolado (pixel)
                x_px = int(p1[0] * (1 - alpha) + p2[0] * alpha)
                y_px = int(p1[1] * (1 - alpha) + p2[1] * alpha)

                # Dibujar punto ROJO en imagen
                cv2.circle(vis_img, (x_px, y_px), 2, (0, 0, 255), -1)

                # Convertir a Mundo Real (Metros)
                wx, wy = pixel_to_world(x_px, y_px, origin[0], origin[1], resolution, height)
                
                # Calcular orientación (Yaw) para mirar hacia la planta (aprox perpendicular a la ruta)
                # Vector de la ruta
                dx = p2[0] - p1[0]
                dy = p2[1] - p1[1]
                yaw_ruta = math.atan2(-dy, dx) # Negativo en Y por coord imagen
                # Mirar 90 grados a la derecha de la ruta (hacia el interior del polígono/planta)
                yaw_look = yaw_ruta + (math.pi / 2)

                waypoints_list.append({
                    'name': f"Fila{row_count}_P{len(waypoints_list)}",
                    'x': round(wx, 3),
                    'y': round(wy, 3),
                    'yaw': round(math.degrees(yaw_look), 1)
                })

    # --- 5. VISUALIZACIÓN Y GUARDADO ---
    print(f"✅ Generados {len(waypoints_list)} waypoints.")
    print("👀 Se abrirá una ventana con la ruta propuesta (Línea Verde = Ruta).")
    print("   PULSA CUALQUIER TECLA EN LA VENTANA PARA GUARDAR Y SALIR.")
    
    # Mostrar resultado
    # Ajustar tamaño ventana si el mapa es muy grande
    cv2.namedWindow("Ruta Generada", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Ruta Generada", 800, 600)
    cv2.imshow("Ruta Generada", vis_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # Guardar YAML
    output_path = os.path.expanduser('~/gonzalo_ws/src/sancho_navigation/config/my_route.yaml')
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    
    with open(output_path, 'w') as f:
        yaml.dump(waypoints_list, f, sort_keys=False)
        
    print(f"\n💾 Archivo guardado en: {output_path}")
    print("🚀 Ya puedes lanzar la navegación.")

if __name__ == "__main__":
    lineal_route_generator()