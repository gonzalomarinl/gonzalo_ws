import cv2
import numpy as np
import yaml
import os
import math

# --- CONFIGURACIÓN ---
MAP_NAME = "greenhouse_map"  # Asegúrate que coincide con tu mapa
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
    """Genera puntos intermedios entre p1 y p2 cada 'interval_px'"""
    dist = math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)
    if dist < interval_px:
        return []
    
    num_points = int(dist // interval_px)
    new_points = []
    for i in range(1, num_points + 1):
        # Interpolación lineal
        factor = i / (num_points + 1)
        nx = int(p1[0] + (p2[0] - p1[0]) * factor)
        ny = int(p1[1] + (p2[1] - p1[1]) * factor)
        new_points.append([nx, ny])
    return new_points

def generate_route():
    print("\n--- PARÁMETROS DE SEGURIDAD ---")
    
    # 1. Obtener resolución para convertir metros a píxeles
    resolution, origin = get_map_metadata(YAML_PATH)
    print(f"Resolución del mapa: {resolution} m/px")

    # 2. Pedir datos al usuario
    try:
        margin_m = float(input("1. Distancia de seguridad a la planta (metros, ej: 0.6): "))
        interval_m = float(input("2. Separación entre fotos (metros, ej: 0.5): "))
    except ValueError:
        print("❌ Por favor introduce números válidos (usa punto para decimales).")
        return

    # Convertir a píxeles
    SAFETY_MARGIN_PX = int(margin_m / resolution)
    POINT_INTERVAL_PX = int(interval_m / resolution)

    print(f"\n🗺️  Cargando mapa: {MAP_PATH}")
    img = cv2.imread(MAP_PATH, cv2.IMREAD_GRAYSCALE)
    
    if img is None:
        print("❌ Error: No se encuentra la imagen del mapa (.pgm)")
        return

    # --- PROCESADO DE IMAGEN ---
    # Lo negro (0) son obstáculos. Invertimos para dilatar.
    _, thresh = cv2.threshold(img, 200, 255, cv2.THRESH_BINARY_INV)

    # Dilatamos para crear el "Area Prohibida" (Planta + Margen de seguridad)
    kernel = np.ones((SAFETY_MARGIN_PX, SAFETY_MARGIN_PX), np.uint8)
    dilated = cv2.dilate(thresh, kernel, iterations=1)

    # Encontramos contornos (Filas de plantas dilatadas)
    contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # --- LÓGICA ZIG-ZAG ---
    # Ordenar contornos de ABAJO a ARRIBA (Coordenada Y Mayor a Menor)
    # En imágenes, Y crece hacia abajo, así que Y Mayor = Abajo.
    contours = sorted(contours, key=lambda c: cv2.boundingRect(c)[1], reverse=True)

    full_route = []
    debug_img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    print(f"✅ Detectadas {len(contours)} filas.")

    for i, cnt in enumerate(contours):
        # 1. Obtenemos el rectángulo que envuelve la fila
        x, y, w, h = cv2.boundingRect(cnt)
        
        # 2. Definimos la línea de paso. 
        # Usamos el borde SUPERIOR del área dilatada para asegurar visibilidad y seguridad
        # Esto crea una línea recta paralela a la fila.
        # Restamos un pequeño offset extra para no pisar el borde exacto
        line_y = y  
        
        start_x = x
        end_x = x + w

        # 3. Decidimos la dirección (Zig-Zag)
        if i % 2 == 0: 
            # Filas PARES (0, 2...): Izquierda -> Derecha
            p_start = [start_x, line_y]
            p_end = [end_x, line_y]
            color = (0, 255, 0) # Verde
            print(f"Fila {i+1}: Izquierda -> Derecha")
        else:
            # Filas IMPARES (1, 3...): Derecha -> Izquierda
            p_start = [end_x, line_y]
            p_end = [start_x, line_y]
            color = (255, 0, 0) # Azul para diferenciar
            print(f"Fila {i+1}: Derecha -> Izquierda")

        # 4. Generar Waypoints interpolados
        # Añadimos punto inicial
        row_points = [p_start]
        
        # Añadimos intermedios (cada X metros)
        intermediates = interpolate_points(p_start, p_end, POINT_INTERVAL_PX)
        row_points.extend(intermediates)
        
        # Añadimos punto final
        row_points.append(p_end)

        full_route.extend(row_points)
        
        # DIBUJAR DEBUG
        # Dibujamos el contorno de seguridad (Gris)
        cv2.drawContours(debug_img, [cnt], -1, (200, 200, 200), 1)
        
        # Dibujamos la línea de ruta
        cv2.line(debug_img, tuple(p_start), tuple(p_end), color, 2)
        
        # Dibujamos los puntos
        for pt in row_points:
            cv2.circle(debug_img, tuple(pt), 3, (0, 0, 255), -1)

    # --- GUARDAR EN YAML ---
    origin_x, origin_y = origin[0], origin[1]
    final_yaml_data = []

    # Dibujar conexiones entre filas
    for k in range(len(full_route) - 1):
        cv2.line(debug_img, tuple(full_route[k]), tuple(full_route[k+1]), (0, 255, 255), 1)

    for idx, point in enumerate(full_route):
        px, py = point[0], point[1]
        
        # Pixel -> Metros
        world_x = origin_x + (px * resolution)
        world_y = origin_y + ((img.shape[0] - py) * resolution)

        # Orientación (apuntando al siguiente punto)
        if idx < len(full_route) - 1:
            next_p = full_route[idx+1]
            delta_x = next_p[0] - px
            delta_y = -(next_p[1] - py)
            yaw_deg = np.degrees(np.arctan2(delta_y, delta_x))
        else:
            yaw_deg = final_yaml_data[-1]['yaw'] if final_yaml_data else 0.0

        point_dict = {
            'name': f"Fila_P{idx}",
            'x': float(world_x),
            'y': float(world_y),
            'yaw': float(yaw_deg)
        }
        final_yaml_data.append(point_dict)

    with open(OUTPUT_YAML, 'w') as f:
        yaml.dump(final_yaml_data, f)

    print(f"\n✅ Ruta calculada: {len(full_route)} puntos.")
    print("👀 Se abrirá una ventana para verificar. PULSA UNA TECLA PARA GUARDAR.")
    
    cv2.imshow("Ruta ZigZag", debug_img)
    cv2.imwrite(OUTPUT_IMG, debug_img)
    print(f"📸 Imagen guardada en: {OUTPUT_IMG}")
    
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    generate_route()