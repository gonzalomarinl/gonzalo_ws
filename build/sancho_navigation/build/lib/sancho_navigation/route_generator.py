#!/usr/bin/env python3
import yaml
import os

def generate_route():
    print("=================================================")
    print("   GENERADOR DE RUTAS UNIVERSAL (ROBOT REAL)     ")
    print("=================================================")
    print("Este script genera una misión de patrulla basada")
    print("en las medidas físicas reales de TU invernadero.")
    print("-------------------------------------------------")

    try:
        # --- 1. DEFINICIÓN DE LA FILA (LONGITUD) ---
        print("\n--- PASO 1: LA FILA ---")
        num_plantas = int(input("1. ¿Cuántas paradas/plantas quieres hacer por lado? (Ej: 6): "))
        distancia_entre_paradas = float(input("2. ¿Distancia entre cada parada? (metros, Ej: 0.6): "))
        
        # --- 2. DEFINICIÓN DEL ESPACIO (ANCHURA) ---
        print("\n--- PASO 2: EL INVERNADERO ---")
        num_filas = int(input("3. ¿Cuántas filas de cultivo vas a inspeccionar? (Ej: 1, 2...): "))
        distancia_entre_filas = float(input("4. ¿Distancia entre el CENTRO de una fila y la siguiente? (metros, Ej: 2.0): "))
        
        # --- 3. DEFINICIÓN DE SEGURIDAD ---
        print("\n--- PASO 3: POSICIÓN DEL ROBOT ---")
        distancia_lateral = float(input("5. ¿A qué distancia de la planta debe ir el robot? (metros, Ej: 0.9): "))
        
        print("\n--- PASO 4: PUNTO DE INICIO ---")
        start_x = float(input("6. Coordenada X donde empieza la primera planta (Ej: -1.8): "))
        start_y = float(input("7. Coordenada Y donde está el CENTRO de la primera fila (Normalmente 0.0): "))

    except ValueError:
        print("❌ Error: Por favor introduce números válidos (usa punto '.' para decimales).")
        return

    # --- CÁLCULOS MATEMÁTICOS ---
    waypoints = []
    
    # Longitud total de trabajo de una fila
    longitud_fila = (num_plantas - 1) * distancia_entre_paradas
    end_x_calculado = start_x + longitud_fila
    
    # Puntos de maniobra (Headlands) - Un poco más allá del inicio y fin para poder girar
    margen_maniobra = 0.8 
    x_giro_final = end_x_calculado + margen_maniobra
    x_giro_inicio = start_x - margen_maniobra

    print(f"\n⚙️  CALCULANDO GEOMETRÍA...")
    print(f"   - Longitud de fila: {longitud_fila:.2f}m")
    print(f"   - Zona de cultivo X: [{start_x} a {end_x_calculado}]")
    print(f"   - Zona de giro X: [{x_giro_inicio} y {x_giro_final}]")

    # Bucle para generar cada fila según la distancia que nos has dicho
    for i_fila in range(num_filas):
        
        # Calculamos la Y central de ESTA fila actual
        y_centro_fila = start_y + (i_fila * distancia_entre_filas)
        nombre_fila = f"Fila_{i_fila + 1}"
        
        print(f"\n🌱 Generando {nombre_fila} (Centro Y={y_centro_fila:.2f})...")

        # === IDA (Lado A - "Sur" de la planta) ===
        # El robot va en Y = Centro - Distancia_Lateral
        y_robot_ida = y_centro_fila - distancia_lateral
        
        for i in range(num_plantas):
            x_actual = start_x + (i * distancia_entre_paradas)
            waypoints.append({
                'name': f"{nombre_fila}_LadoA_P{i}",
                'x': round(x_actual, 2),
                'y': round(y_robot_ida, 2),
                'yaw': 90.0 # Mirando a la planta (Izquierda relativa)
            })

        # === MANIOBRA DE CAMBIO DE LADO (La "U") ===
        # 1. Salir de la fila hasta la zona de giro
        waypoints.append({
            'name': f"{nombre_fila}_Salida_Ida",
            'x': round(x_giro_final, 2),
            'y': round(y_robot_ida, 2),
            'yaw': 0.0 # Mirando al frente
        })
        
        # 2. Entrar al pasillo de vuelta
        y_robot_vuelta = y_centro_fila + distancia_lateral
        waypoints.append({
            'name': f"{nombre_fila}_Entrada_Vuelta",
            'x': round(x_giro_final, 2),
            'y': round(y_robot_vuelta, 2),
            'yaw': 180.0 # Ya ha dado la vuelta
        })

        # === VUELTA (Lado B - "Norte" de la planta) ===
        for i in range(num_plantas):
            # Inspeccionamos de final a principio
            idx_inverso = (num_plantas - 1) - i
            x_actual = start_x + (idx_inverso * distancia_entre_paradas)
            
            waypoints.append({
                'name': f"{nombre_fila}_LadoB_P{idx_inverso}",
                'x': round(x_actual, 2),
                'y': round(y_robot_vuelta, 2),
                'yaw': -90.0 # Mirando a la planta
            })

        # === TRANSICIÓN A LA SIGUIENTE FILA (Si existe) ===
        if i_fila < num_filas - 1:
            # Calculamos dónde empieza la SIGUIENTE fila
            y_siguiente_fila_ida = (y_centro_fila + distancia_entre_filas) - distancia_lateral
            
            print(f"   -> Calculando transición a Fila {i_fila + 2}...")
            
            # 1. Salir de la fila actual hacia la zona de maniobra inicial
            waypoints.append({
                'name': f"Salida_Fila_{i_fila+1}",
                'x': round(x_giro_inicio, 2),
                'y': round(y_robot_vuelta, 2),
                'yaw': 180.0
            })
            
            # 2. Desplazarse lateralmente (o en diagonal) al inicio de la siguiente
            # Nav2 calculará la curva, nosotros solo damos el destino clave
            waypoints.append({
                'name': f"Inicio_Fila_{i_fila+2}",
                'x': round(x_giro_inicio, 2), # Nos mantenemos en zona segura X
                'y': round(y_siguiente_fila_ida, 2), # Vamos a la Y de la siguiente
                'yaw': 0.0 # Nos preparamos para entrar
            })

    # --- GUARDADO ---
    output_path = os.path.expanduser('~/gonzalo_ws/src/sancho_navigation/config/my_route.yaml')
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    
    with open(output_path, 'w') as f:
        yaml.dump(waypoints, f, sort_keys=False)
        
    print(f"\n✅ RUTA GUARDADA EN: {output_path}")
    print(f"📍 Total Waypoints: {len(waypoints)}")
    print("🚀 Ya puedes lanzar el robot (Terminal 3). Él se adaptará a estas medidas.")

if __name__ == "__main__":
    generate_route()