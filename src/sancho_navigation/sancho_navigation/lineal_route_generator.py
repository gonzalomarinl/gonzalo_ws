import cv2
import numpy as np
import yaml
import os
import math

MAP_NAME = "greenhouse_map"
BASE_DIR = os.path.expanduser("~/gonzalo_ws/src/sancho_navigation")
MAP_PATH = f"{BASE_DIR}/maps/{MAP_NAME}.pgm"
YAML_PATH = f"{BASE_DIR}/maps/{MAP_NAME}.yaml"
OUTPUT_YAML = f"{BASE_DIR}/config/my_route.yaml"
OUTPUT_IMG = f"{BASE_DIR}/config/ruta_visual.jpg"

def generate_route():
    print("\n--- 🛰️ GENERADOR DE RUTA ORTOGONAL (SIN CRUCES) ---")
    with open(YAML_PATH, 'r') as f:
        data = yaml.safe_load(f)
        res, origin = data['resolution'], data['origin']

    margin_m = float(input("📏 Distancia a la planta (m): "))
    interval_m = float(input("📸 Separación entre fotos (m): "))
    m_px, i_px = int(margin_m / res), int(interval_m / res)

    img = cv2.imread(MAP_PATH, cv2.IMREAD_GRAYSCALE)
    debug_img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    _, thresh = cv2.threshold(img, 210, 255, cv2.THRESH_BINARY_INV)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Filtrar y ordenar filas de abajo a arriba
    valid_rows = sorted([c for c in contours if cv2.contourArea(c) > 100], 
                        key=lambda c: cv2.boundingRect(c)[1], reverse=True)

    route_pixels = []
    for cnt in valid_rows:
        x, y, w, h = cv2.boundingRect(cnt)
        # Cara A (Abajo)
        pts_a = [[px, y + h + m_px] for px in range(x - m_px, x + w + m_px, i_px)]
        # Cara B (Arriba)
        pts_b = [[px, y - m_px] for px in range(x + w + m_px, x - m_px, -i_px)]
        
        # AÑADIR PUNTO DE GIRO SEGURO (Evita que la línea verde cruce la planta)
        # Conectamos el último de A con el primero de B rodeando la fila por el lateral
        if pts_a and pts_b:
            corner_1 = [pts_a[-1][0] + i_px//2, pts_a[-1][1]] # Un poco más a la derecha
            corner_2 = [pts_a[-1][0] + i_px//2, pts_b[0][1]]  # Subimos a la altura de B
            route_pixels.extend(pts_a + [corner_1, corner_2] + pts_b)
        else:
            route_pixels.extend(pts_a + pts_b)

    # Exportar y dibujar
    final_yaml = []
    for idx, pt in enumerate(route_pixels):
        wx = origin[0] + (pt[0] * res)
        wy = origin[1] + ((img.shape[0] - pt[1]) * res)
        cv2.circle(debug_img, (pt[0], pt[1]), 3, (0, 0, 255), -1) # Puntos rojos
        if idx > 0:
            cv2.line(debug_img, (route_pixels[idx-1][0], route_pixels[idx-1][1]), (pt[0], pt[1]), (0, 255, 0), 1)
        final_yaml.append({'name': f"P_{idx}", 'x': float(wx), 'y': float(wy), 'yaw': 0.0})

    with open(OUTPUT_YAML, 'w') as f:
        yaml.dump(final_yaml, f)
    cv2.imwrite(OUTPUT_IMG, debug_img)
    print(f"✅ ¡LISTO! Abre {OUTPUT_IMG}. Fíjate que ahora las líneas rodean la planta.")

if __name__ == "__main__":
    generate_route()