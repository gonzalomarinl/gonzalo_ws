#!/usr/bin/env python3
import yaml
import os
import math

def generate_route():
    print("=========================================")
    print("   GENERADOR DE RUTAS CORREGIDO (TFG)    ")
    print("=========================================")

    # --- 1. CONFIGURACIÓN DEL MAPA ---
    # Coordenadas X donde empiezan y terminan las filas
    start_x = -1.8       # Coordenada X de la primera planta
    end_x = 1.8          # Coordenada X de la última planta
    
    # Dónde están físicamente las filas de plantas (según greenhouse.world)
    # Fila 1 en Y=0.0, Fila 2 en Y=2.0, Fila 3 en Y=4.0
    posiciones_filas_y = [0.0, 2.0, 4.0] 
    
    # Configuración de navegación
    distancia_seguridad = 0.9  # Metros de distancia desde el centro de la planta al robot (pasillo)
    step_inspection = 0.6      # Cada cuántos metros se para a inspeccionar
    
    # --- 2. CÁLCULOS ---
    waypoints = []
    longitud_fila = abs(end_x - start_x)
    num_paradas = int(longitud_fila / step_inspection) + 1

    print(f"⚙️  Generando ruta para {len(posiciones_filas_y)} filas.")
    print(f"   - Distancia al cultivo: {distancia_seguridad}m")
    
    for i_fila, y_planta in enumerate(posiciones_filas_y):
        nombre_fila = f"Fila_{i_fila + 1}"
        print(f"\n🌱 Procesando {nombre_fila} (Centro en Y={y_planta})...")

        # --- LADO A (IDA) ---
        # El robot va por debajo de la planta (Y_planta - offset)
        # Debe mirar hacia la planta (hacia IZQUIERDA relativa al avance, o Norte absoluto -> Yaw 90º)
        # Pero cuidado: si avanza hacia X positivo y mira a Y planta...
        # Vamos a simplificar: Orientación absoluta.
        # Si la planta está en Y=0 y el robot en Y=-0.9, debe mirar a +90º (PI/2) para ver la planta.
        
        y_robot_ida = y_planta - distancia_seguridad
        print(f"   -> Generando Ida (Y={y_robot_ida})...")
        
        # Recorremos de izquierda a derecha (start_x -> end_x)
        for i in range(num_paradas):
            x = start_x + (i * step_inspection)
            if x > end_x: break # Seguridad
            
            waypoints.append({
                'name': f"{nombre_fila}_LadoA_P{i}",
                'x': round(x, 2),
                'y': round(y_robot_ida, 2),
                'yaw': 90.0 # Mirando hacia la planta (Norte)
            })

        # Waypoint de maniobra (salir de la fila para dar la vuelta)
        waypoints.append({
            'name': f"{nombre_fila}_Giro_Extremo",
            'x': round(end_x + 0.8, 2), # Un poco más adelante para girar
            'y': round(y_robot_ida, 2),
            'yaw': 0.0
        })

        # --- LADO B (VUELTA) ---
        # El robot va por encima de la planta (Y_planta + offset)
        # Si la planta está en Y=0 y robot en Y=+0.9, debe mirar a -90º (-PI/2) para ver la planta.
        
        y_robot_vuelta = y_planta + distancia_seguridad
        print(f"   <- Generando Vuelta (Y={y_robot_vuelta})...")

        # Waypoint de entrada al pasillo de vuelta
        waypoints.append({
            'name': f"{nombre_fila}_Entrada_Vuelta",
            'x': round(end_x + 0.8, 2),
            'y': round(y_robot_vuelta, 2),
            'yaw': 180.0
        })

        # Recorremos de derecha a izquierda (end_x -> start_x)
        for i in range(num_paradas):
            x = end_x - (i * step_inspection)
            if x < start_x: break
            
            waypoints.append({
                'name': f"{nombre_fila}_LadoB_P{i}",
                'x': round(x, 2),
                'y': round(y_robot_vuelta, 2),
                'yaw': -90.0 # Mirando hacia la planta (Sur)
            })

        # Maniobra para cambiar a siguiente bloque (si hay más filas)
        if i_fila < len(posiciones_filas_y) - 1:
            waypoints.append({
                'name': f"Transicion_Fila{i_fila+1}_a_{i_fila+2}",
                'x': round(start_x - 0.8, 2),
                'y': round(y_robot_vuelta, 2), # Estamos arriba de la fila actual
                'yaw': 90.0 # Miramos arriba para ir a la siguiente fila
            })
            
    # --- 3. GUARDADO ---
    output_path = os.path.expanduser('~/tfg_ws/src/sancho_navigation/config/my_route.yaml')
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    
    with open(output_path, 'w') as f:
        yaml.dump(waypoints, f, sort_keys=False)
        
    print(f"\n✅ ¡RUTA CORREGIDA GENERADA! Guardada en: {output_path}")

if __name__ == "__main__":
    generate_route()