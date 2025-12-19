import cv2
import numpy as np
import yaml
import os

# --- CONFIGURACIÓN ---
BASE_PATH = os.path.expanduser("~/gonzalo_ws/src/sancho_navigation")
MAP_YAML_PATH = f"{BASE_PATH}/maps/greenhouse_map.yaml"
OUTPUT_ROUTE_PATH = f"{BASE_PATH}/config/my_route.yaml"
OUTPUT_PREVIEW_PATH = f"{BASE_PATH}/maps/route_preview.png"

# 🚜 PARÁMETROS AGRÍCOLAS
OFFSET_PLANTAS = 0.60   # 60 cm de separación
MARGEN_CABECERA = 1.0   # 1 metro extra al inicio/final

# --- FUNCIONES AUXILIARES ---
def get_quaternion(yaw_rad):
    w = np.cos(yaw_rad / 2.0)
    z = np.sin(yaw_rad / 2.0)
    return [0.0, 0.0, z, w]

def pixel_to_world(px_x, px_y, origin, res, height_px):
    orig_x, orig_y = origin[0], origin[1]
    world_x = orig_x + (px_x * res)
    world_y = orig_y + ((height_px - px_y) * res)
    return world_x, world_y

def world_to_pixel(wx, wy, origin, res, height_px):
    orig_x, orig_y = origin[0], origin[1]
    px = int((wx - orig_x) / res)
    py = int(height_px - ((wy - orig_y) / res))
    return px, py

# --- FUNCIÓN PRINCIPAL ---
def generate_route():
    print("🚜 GENERADOR AGRÍCOLA + VISUALIZADOR 🚜")

    # 1. Cargar Mapa
    if not os.path.exists(MAP_YAML_PATH):
        print("❌ Error: No encuentro greenhouse_map.yaml")
        return

    with open(MAP_YAML_PATH, 'r') as f:
        map_config = yaml.safe_load(f)

    res = map_config['resolution']
    origin = map_config['origin']
    img_path = os.path.join(os.path.dirname(MAP_YAML_PATH), map_config['image'])

    print(f"   📐 Resolución: {res} m/px")
    
    img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
    img_vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR) # Copia en color para pintar
    h_img, w_img = img.shape

    # 2. Detectar Filas (Umbral < 100 es planta)
    _, thresh = cv2.threshold(img, 100, 255, cv2.THRESH_BINARY_INV)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    rows_data = []
    for cnt in contours:
        if cv2.contourArea(cnt) > 200:
            x, y, w, h = cv2.boundingRect(cnt)
            rows_data.append({'x': x, 'y': y, 'w': w, 'h': h, 'cy': y + h/2})

    # Ordenar de Abajo a Arriba
    rows_data.sort(key=lambda k: k['cy'], reverse=True)
    print(f"   🌱 Filas detectadas: {len(rows_data)}")

    # 3. Calcular Waypoints
    waypoints = []
    px_offset = int(OFFSET_PLANTAS / res)
    px_margin = int(MARGEN_CABECERA / res)

    for i, row in enumerate(rows_data):
        rx, ry, rw, rh = row['x'], row['y'], row['w'], row['h']
        
        # Coordenadas X extendidas
        x_inicio = rx - px_margin
        x_fin    = rx + rw + px_margin

        # CARA A (Inferior) -> Ida
        y_cara_A = ry + rh + px_offset
        wx1, wy1 = pixel_to_world(x_inicio, y_cara_A, origin, res, h_img)
        wx2, wy2 = pixel_to_world(x_fin, y_cara_A, origin, res, h_img)
        
        waypoints.append({'x': wx1, 'y': wy1, 'q': get_quaternion(0.0)})
        waypoints.append({'x': wx2, 'y': wy2, 'q': get_quaternion(0.0)})

        # CARA B (Superior) -> Vuelta
        y_cara_B = ry - px_offset
        wx3, wy3 = pixel_to_world(x_fin, y_cara_B, origin, res, h_img)
        wx4, wy4 = pixel_to_world(x_inicio, y_cara_B, origin, res, h_img)

        waypoints.append({'x': wx3, 'y': wy3, 'q': get_quaternion(3.14159)})
        waypoints.append({'x': wx4, 'y': wy4, 'q': get_quaternion(3.14159)})

    # 4. Guardar YAML
    print(f"💾 Guardando ruta: {OUTPUT_ROUTE_PATH}")
    with open(OUTPUT_ROUTE_PATH, 'w') as f:
        f.write("# Ruta Generada Automáticamente\nposes:\n")
        for wp in waypoints:
            f.write(f"  - header:\n      frame_id: map\n")
            f.write(f"    pose:\n      position:\n")
            f.write(f"        x: {wp['x']:.4f}\n        y: {wp['y']:.4f}\n        z: 0.0\n")
            f.write(f"      orientation:\n")
            f.write(f"        x: {wp['q'][0]:.4f}\n        y: {wp['q'][1]:.4f}\n")
            f.write(f"        z: {wp['q'][2]:.4f}\n        w: {wp['q'][3]:.4f}\n")

    # 5. GENERAR VISUALIZACIÓN (Pintar sobre el mapa)
    print("🎨 Generando imagen de previsualización...")
    pixels = []
    
    # Convertir waypoints a píxeles para pintar
    for i, wp in enumerate(waypoints):
        px, py = world_to_pixel(wp['x'], wp['y'], origin, res, h_img)
        pixels.append((px, py))
        
        # Círculo Azul (Punto)
        cv2.circle(img_vis, (px, py), 4, (255, 0, 0), -1)
        # Texto con número
        cv2.putText(img_vis, str(i+1), (px+5, py-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

    # Líneas Verdes (Ruta)
    for i in range(len(pixels)-1):
        cv2.line(img_vis, pixels[i], pixels[i+1], (0, 200, 0), 2)

    cv2.imwrite(OUTPUT_PREVIEW_PATH, img_vis)
    print(f"✅ ¡LISTO! Abre esta imagen para ver tu ruta:\n   📂 {OUTPUT_PREVIEW_PATH}")

if __name__ == "__main__":
    generate_route()