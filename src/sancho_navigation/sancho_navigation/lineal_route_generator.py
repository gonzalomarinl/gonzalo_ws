import cv2
import numpy as np
import yaml
import os
import math

# --- CONFIGURACIÓN ESTÁTICA ---
MAP_NAME = "greenhouse_map"
BASE_DIR = os.path.expanduser("~/gonzalo_ws/src/sancho_navigation")
MAP_PATH = f"{BASE_DIR}/maps/{MAP_NAME}.pgm"
YAML_PATH = f"{BASE_DIR}/maps/{MAP_NAME}.yaml"
OUTPUT_YAML = f"{BASE_DIR}/config/my_route.yaml"
OUTPUT_IMG = f"{BASE_DIR}/config/ruta_generada.jpg"

def get_map_metadata(yaml_file):
    with open(yaml_file, 'r') as f:
        data = yaml.safe_load(f)
    return data['resolution'], data['origin']

def interpolate_points(p1, p2, interval_px):
    """Genera puntos intermedios a una distancia exacta en píxeles"""
    dist = math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)
    if dist < interval_px: return []
    num_points = int(dist // interval_px)
    new_points = []
    for i in range(1, num_points + 1):
        factor = i / (num_points + 1)
        nx = int(p1[0] + (p2[0] - p1[0]) * factor)
        ny = int(p1[1] + (p2[1] - p1[1]) * factor)
        new_points.append([nx, ny])
    return new_points

def generate_route():
    print("\n--- 🛰️ GENERADOR DE RUTA AGRÍCOLA (MODO U) ---")
    
    # 1. Obtener metadatos del mapa
    resolution, origin = get_map_metadata(YAML_PATH)
    
    # 2. SOLICITAR PARÁMETROS AL OPERARIO
    try:
        margin_m = float(input("📏 Distancia de seguridad a la fila (metros, ej: 0.6): "))
        interval_m = float(input("📸 Distancia entre fotos (metros, ej: 0.5): "))
    except ValueError:
        print("❌ Error: Introduce números válidos.")
        return

    # Convertir metros a píxeles
    SAFETY_MARGIN_PX = int(margin_m / resolution)
    POINT_INTERVAL_PX = int(interval_m / resolution)

    # 3. Procesar el mapa
    img = cv2.imread(MAP_PATH, cv2.IMREAD_GRAYSCALE)
    if img is None:
        print(f"❌ No se pudo cargar el mapa en {MAP_PATH}")
        return

    # Detectar filas (lo negro en el mapa)
    _, thresh = cv2.threshold(img, 200, 255, cv2.THRESH_BINARY_INV)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Ordenar filas de ABAJO a ARRIBA (Y mayor a menor en imagen)
    contours = sorted(contours, key=lambda c: cv2.boundingRect(c)[1], reverse=True)

    full_route = []
    debug_img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    print(f"🔍 Filas detectadas: {len(contours)}")

    for i, cnt in enumerate(contours):
        # Bounding box de la fila real
        x, y, w, h = cv2.boundingRect(cnt)
        
        # Definir coordenadas de las líneas paralelas (Cara A y Cara B)
        # La cara A es arriba de la fila, la cara B es abajo
        y_top = y - SAFETY_MARGIN_PX
        y_bottom = y + h + SAFETY_MARGIN_PX
        
        # Extendemos un poco los extremos para que el robot tenga espacio al girar
        x_start = x - SAFETY_MARGIN_PX
        x_end = x + w + SAFETY_MARGIN_PX

        # --- CARA A (De Izquierda a Derecha) ---
        p1_start = [x_start, y_top]
        p1_end = [x_end, y_top]
        face_a = [p1_start] + interpolate_points(p1_start, p1_end, POINT_INTERVAL_PX) + [p1_end]

        # --- CARA B (De Derecha a Izquierda) ---
        p2_start = [x_end, y_bottom]
        p2_end = [x_start, y_bottom]
        face_b = [p2_start] + interpolate_points(p2_start, p2_end, POINT_INTERVAL_PX) + [p2_end]

        # Unimos las dos caras formando la "U"
        full_route.extend(face_a)
        full_route.extend(face_b)

        # Dibujo para comprobación visual
        cv2.rectangle(debug_img, (x, y), (x+w, y+h), (0, 0, 255), 2) # Fila en Rojo
        cv2.line(debug_img, tuple(p1_start), tuple(p1_end), (0, 255, 0), 2) # Cara A en Verde
        cv2.line(debug_img, tuple(p2_start), tuple(p2_end), (255, 0, 0), 2) # Cara B en Azul

    # 4. CONVERTIR A COORDENADAS DE MAPA ROS (METROS)
    origin_x, origin_y = origin[0], origin[1]
    final_yaml_data = []

    for idx, point in enumerate(full_route):
        px, py = point[0], point[1]
        
        # Transformación Píxel -> Metros
        world_x = origin_x + (px * resolution)
        world_y = origin_y + ((img.shape[0] - py) * resolution)

        # Calcular orientación (mirando al siguiente punto)
        if idx < len(full_route) - 1:
            next_p = full_route[idx+1]
            yaw = math.atan2(-(next_p[1] - py), next_p[0] - px)
            yaw_deg = math.degrees(yaw)
        else:
            yaw_deg = final_yaml_data[-1]['yaw'] if final_yaml_data else 0.0

        final_yaml_data.append({
            'name': f"P_{idx}",
            'x': float(world_x),
            'y': float(world_y),
            'yaw': float(yaw_deg)
        })

    # 5. Guardar archivos
    with open(OUTPUT_YAML, 'w') as f:
        yaml.dump(final_yaml_data, f)

    cv2.imwrite(OUTPUT_IMG, debug_img)
    print(f"✅ Ruta generada con {len(full_route)} puntos.")
    print(f"💾 Guardado en: {OUTPUT_YAML}")
    print(f"🖼️  Visualización guardada en: {OUTPUT_IMG}")

if __name__ == "__main__":
    generate_route()