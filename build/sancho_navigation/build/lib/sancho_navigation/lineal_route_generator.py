import cv2
import numpy as np
import yaml
import os
import math

# --- CONFIGURACIÓN ---
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
    print("\n--- GENERADOR DE RUTA DOBLE CARA ---")
    resolution, origin = get_map_metadata(YAML_PATH)
    
    # Parámetros por defecto para ir rápido
    margin_m = 0.6
    interval_m = 0.5
    
    SAFETY_MARGIN_PX = int(margin_m / resolution)
    POINT_INTERVAL_PX = int(interval_m / resolution)

    img = cv2.imread(MAP_PATH, cv2.IMREAD_GRAYSCALE)
    if img is None: return

    # Detectar obstáculos
    _, thresh = cv2.threshold(img, 200, 255, cv2.THRESH_BINARY_INV)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) # Sin dilatar aquí para calcular bien los bordes
    
    # Ordenar de ABAJO a ARRIBA
    contours = sorted(contours, key=lambda c: cv2.boundingRect(c)[1], reverse=True)

    full_route = []
    debug_img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    print(f"✅ Detectadas {len(contours)} filas.")

    for i, cnt in enumerate(contours):
        x, y, w, h = cv2.boundingRect(cnt)
        
        # Definimos DOS líneas de paso por cada fila
        y_top_path = y - SAFETY_MARGIN_PX
        y_bottom_path = y + h + SAFETY_MARGIN_PX
        
        start_x = x - SAFETY_MARGIN_PX
        end_x = x + w + SAFETY_MARGIN_PX

        # LÓGICA DE BARRIDO COMPLETO
        # 1. Cara Superior (Izquierda -> Derecha)
        p1_start = [start_x, y_top_path]
        p1_end = [end_x, y_top_path]
        
        # 2. Cara Inferior (Derecha -> Izquierda)
        p2_start = [end_x, y_bottom_path]
        p2_end = [start_x, y_bottom_path]

        # Generar puntos Cara Superior
        face_a = [p1_start] + interpolate_points(p1_start, p1_end, POINT_INTERVAL_PX) + [p1_end]
        
        # Generar puntos Cara Inferior
        face_b = [p2_start] + interpolate_points(p2_start, p2_end, POINT_INTERVAL_PX) + [p2_end]

        # Añadir a la ruta: Primero cara A, luego bajamos a cara B
        full_route.extend(face_a)
        full_route.extend(face_b)

        # Dibujar para debug
        cv2.rectangle(debug_img, (x, y), (x+w, y+h), (100,100,100), 1)
        cv2.line(debug_img, tuple(p1_start), tuple(p1_end), (0,255,0), 2) # Verde Arriba
        cv2.line(debug_img, tuple(p2_start), tuple(p2_end), (255,0,0), 2) # Azul Abajo

    # Guardar YAML
    origin_x, origin_y = origin[0], origin[1]
    final_yaml_data = []

    for idx, point in enumerate(full_route):
        px, py = point[0], point[1]
        world_x = origin_x + (px * resolution)
        world_y = origin_y + ((img.shape[0] - py) * resolution)
        
        # Calcular orientación hacia el siguiente punto
        if idx < len(full_route) - 1:
            next_p = full_route[idx+1]
            delta_x = next_p[0] - px
            delta_y = -(next_p[1] - py)
            yaw_deg = np.degrees(np.arctan2(delta_y, delta_x))
        else:
            yaw_deg = 0.0

        point_dict = {
            'name': f"P_{idx}",
            'x': float(world_x),
            'y': float(world_y),
            'yaw': float(yaw_deg)
        }
        final_yaml_data.append(point_dict)

    with open(OUTPUT_YAML, 'w') as f:
        yaml.dump(final_yaml_data, f)

    print(f"✅ Ruta guardada: {OUTPUT_YAML}")
    cv2.imwrite(OUTPUT_IMG, debug_img)

if __name__ == "__main__":
    generate_route()