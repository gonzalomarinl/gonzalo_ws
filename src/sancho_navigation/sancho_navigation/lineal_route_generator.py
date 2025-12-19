import cv2
import numpy as np
import yaml
import os

# --- CONFIGURACIÓN ---
MAP_NAME = "greenhouse_map"  # Nombre de tu mapa (sin extensión)
# Ajusta esta ruta a tu usuario real
BASE_DIR = "/home/gonzalomarin/gonzalo_ws/src/sancho_navigation"
MAP_PATH = f"{BASE_DIR}/maps/{MAP_NAME}.pgm"
YAML_PATH = f"{BASE_DIR}/maps/{MAP_NAME}.yaml"
OUTPUT_YAML = f"{BASE_DIR}/config/my_route.yaml"

# Parámetros de ajuste
SAFETY_MARGIN = 15     # Cuánto nos alejamos de la planta (píxeles)
POINT_INTERVAL = 25    # Cada cuántos píxeles ponemos un waypoint (para no tener mil puntos)

def get_map_resolution(yaml_file):
    with open(yaml_file, 'r') as f:
        data = yaml.safe_load(f)
    return data['resolution'], data['origin']

def generate_route():
    print(f"🗺️  Cargando mapa: {MAP_PATH}")
    img = cv2.imread(MAP_PATH, cv2.IMREAD_GRAYSCALE)
    
    if img is None:
        print("❌ Error: No se encuentra la imagen del mapa (.pgm)")
        return

    # 1. Procesar imagen para aislar obstáculos (las plantas)
    # Lo negro (0) son obstáculos. Lo blanco (254) es libre.
    _, thresh = cv2.threshold(img, 200, 255, cv2.THRESH_BINARY_INV)

    # 2. Dilatar los obstáculos para crear el margen de seguridad
    # Esto "engorda" las plantas para que la ruta se dibuje separada
    kernel = np.ones((SAFETY_MARGIN, SAFETY_MARGIN), np.uint8)
    dilated = cv2.dilate(thresh, kernel, iterations=1)

    # 3. Encontrar contornos (las filas de plantas)
    contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # --- LA MAGIA: ORDENAR Y UNIR ---
    
    # Ordenamos los contornos de Arriba a Abajo (usando la coordenada Y)
    # Así el robot hará Fila 1 -> Fila 2 -> Fila 3 en orden.
    contours = sorted(contours, key=lambda c: cv2.boundingRect(c)[1])

    full_route = []
    global_point_count = 0

    debug_img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR) # Para visualizar en color

    print(f"✅ Se han detectado {len(contours)} filas de plantas.")

    for i, cnt in enumerate(contours):
        # Simplificar el contorno para que no tenga demasiados vértices
        epsilon = 0.005 * cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, epsilon, True)

        # Tomar puntos cada cierto intervalo
        # Recorremos los puntos del contorno
        row_points = []
        for j in range(0, len(approx), max(1, len(approx)//(len(approx)//5 + 1))): 
            # Nota: El paso del range es un truco para no coger todos los puntos
            # Si quieres más precisión fija, usa POINT_INTERVAL con lógica de distancia
            pt = approx[j][0]
            row_points.append(pt)

        # Aseguramos cerrar el bucle de la fila (volver al inicio de la fila)
        row_points.append(row_points[0]) 

        # Añadimos estos puntos a la ruta global
        full_route.extend(row_points)
        
        # DIBUJAR (Visualización)
        cv2.drawContours(debug_img, [approx], -1, (0, 255, 0), 2) # Línea verde
        
        # Dibujar puntos rojos
        for pt in row_points:
            cv2.circle(debug_img, tuple(pt), 4, (0, 0, 255), -1)

    # --- GUARDAR RUTAS EN YAML ---
    resolution, origin = get_map_resolution(YAML_PATH)
    origin_x, origin_y = origin[0], origin[1]
    
    final_yaml_data = []

    # Dibujar líneas de conexión entre filas para verificar
    for k in range(len(full_route) - 1):
        p1 = tuple(full_route[k])
        p2 = tuple(full_route[k+1])
        cv2.line(debug_img, p1, p2, (0, 255, 0), 1)

    # Convertir píxeles a metros (Coordenadas del mundo real)
    for idx, point in enumerate(full_route):
        # Convertir pixel a coordenadas de mapa (invertir Y porque la imagen se lee de arriba a abajo)
        px, py = point[0], point[1]
        
        # Transformación Píxel -> Metros
        world_x = origin_x + (px * resolution)
        # En imágenes, la Y crece hacia abajo. En ROS, la Y crece hacia arriba.
        # Dependiendo de cómo guardó map_server, a veces hay que invertir la altura.
        # Probamos la fórmula estándar:
        world_y = origin_y + ((img.shape[0] - py) * resolution) 

        # Calcular orientación (YAW) para que mire al siguiente punto
        if idx < len(full_route) - 1:
            next_p = full_route[idx+1]
            next_px, next_py = next_p[0], next_p[1]
            # Ángulo en el sistema de imagen
            delta_x = next_px - px
            delta_y = -(next_py - py) # Negativo porque Y imagen va al revés
            yaw = np.arctan2(delta_y, delta_x)
            yaw_deg = np.degrees(yaw)
        else:
            yaw_deg = 0.0

        point_dict = {
            'name': f"Punto_{idx}",
            'x': float(world_x),
            'y': float(world_y),
            'yaw': float(yaw_deg)
        }
        final_yaml_data.append(point_dict)

    # Guardar
    with open(OUTPUT_YAML, 'w') as f:
        yaml.dump(final_yaml_data, f)

    print(f"💾 Ruta guardada con {len(full_route)} puntos en: {OUTPUT_YAML}")
    print("👀 Abriendo visualización... (Pulsa una tecla en la imagen para cerrar)")
    
    cv2.imshow("Ruta Conectada", debug_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    generate_route()