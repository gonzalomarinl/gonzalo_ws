#!/usr/bin/env python3
import yaml
import os
import time
import sys

# --- LIBRERÍAS ROS 2 PARA EL MOVIMIENTO ---
import rclpy
from geometry_msgs.msg import Twist

def perform_localization_spin():
    """
    Esta función convierte brevemente el script en un nodo de ROS 2
    para enviar comandos de giro al robot y forzar a AMCL a converger.
    """
    print("=================================================")
    print("   FASE 0: AUTO-LOCALIZACIÓN (MOVIMIENTO)        ")
    print("=================================================")
    print("🔄 Inicializando ROS 2 para mover el robot...")
    print("   El robot girará sobre sí mismo durante 4 segundos")
    print("   para que las partículas de AMCL converjan.")
    
    rclpy.init()
    node = rclpy.create_node('route_generator_spinner')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    
    # Mensaje de giro
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.4  # Velocidad de giro moderada (rad/s)
    
    # Mensaje de parada
    stop_twist = Twist()
    
    try:
        # Girar durante 4 segundos
        end_time = time.time() + 4.0
        rate_sleep = 0.1 # 10Hz
        
        while time.time() < end_time:
            pub.publish(twist)
            time.sleep(rate_sleep)
            
        # DETENER EL ROBOT
        print("🛑 Deteniendo robot...")
        for _ in range(5): # Enviar parada varias veces para asegurar
            pub.publish(stop_twist)
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        pub.publish(stop_twist)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("✅ LOCALIZACIÓN COMPLETADA. Partículas convergidas.\n")

def generate_route():
    print("=================================================")
    print("   FASE 1: GENERADOR DE RUTAS (MATEMÁTICAS)      ")
    print("=================================================")
    print("Ahora que el robot está localizado, define la misión.")
    print("-------------------------------------------------")

    try:
        # --- 1. DEFINICIÓN DE LA FILA (LONGITUD) ---
        print("\n--- PASO 1: LA FILA ---")
        num_plantas = int(input("1. ¿Cuántas paradas/plantas quieres hacer por lado? (Ej: 4): "))
        distancia_entre_paradas = float(input("2. ¿Distancia entre cada parada? (metros, Ej: 0.6): "))
        
        # --- 2. DEFINICIÓN DEL ESPACIO (ANCHURA) ---
        print("\n--- PASO 2: EL INVERNADERO ---")
        num_filas = int(input("3. ¿Cuántas filas de cultivo vas a inspeccionar? (Ej: 1): "))
        distancia_entre_filas = float(input("4. ¿Distancia entre el CENTRO de una fila y la siguiente? (metros, Ej: 2.0): "))
        
        # --- 3. DEFINICIÓN DE SEGURIDAD ---
        print("\n--- PASO 3: POSICIÓN DEL ROBOT ---")
        # TRUCO: Sugerir al usuario medir en RViz
        print("   (⚠️ RECOMENDACIÓN: Mide en RViz la distancia desde el centro negro al pasillo blanco)")
        distancia_lateral = float(input("5. ¿A qué distancia de la planta debe ir el robot? (metros, Ej: 0.65): "))
        
        print("\n--- PASO 4: PUNTO DE INICIO ---")
        print("   (⚠️ RECOMENDACIÓN: Usa 'Publish Point' en RViz para obtener estas coordenadas)")
        start_x = float(input("6. Coordenada X donde empieza la primera planta (Ej: -1.8): "))
        start_y = float(input("7. Coordenada Y donde está el CENTRO de la primera fila: "))

    except ValueError:
        print("❌ Error: Por favor introduce números válidos (usa punto '.' para decimales).")
        return

    # --- CÁLCULOS MATEMÁTICOS ---
    waypoints = []
    
    # Longitud total de trabajo de una fila
    longitud_fila = (num_plantas - 1) * distancia_entre_paradas
    end_x_calculado = start_x + longitud_fila
    
    # Puntos de maniobra (Headlands)
    margen_maniobra = 0.8 
    x_giro_final = end_x_calculado + margen_maniobra
    x_giro_inicio = start_x - margen_maniobra

    print(f"\n⚙️  CALCULANDO GEOMETRÍA...")
    print(f"   - Longitud de fila: {longitud_fila:.2f}m")
    print(f"   - Zona de cultivo X: [{start_x} a {end_x_calculado}]")

    # Bucle para generar cada fila
    for i_fila in range(num_filas):
        
        y_centro_fila = start_y + (i_fila * distancia_entre_filas)
        nombre_fila = f"Fila_{i_fila + 1}"
        
        print(f"\n🌱 Generando {nombre_fila} (Centro Y={y_centro_fila:.2f})...")

        # === IDA ===
        y_robot_ida = y_centro_fila - distancia_lateral
        
        for i in range(num_plantas):
            x_actual = start_x + (i * distancia_entre_paradas)
            waypoints.append({
                'name': f"{nombre_fila}_LadoA_P{i}",
                'x': round(x_actual, 2),
                'y': round(y_robot_ida, 2),
                'yaw': 90.0
            })

        # === MANIOBRA U ===
        waypoints.append({
            'name': f"{nombre_fila}_Salida_Ida",
            'x': round(x_giro_final, 2),
            'y': round(y_robot_ida, 2),
            'yaw': 0.0
        })
        
        y_robot_vuelta = y_centro_fila + distancia_lateral
        waypoints.append({
            'name': f"{nombre_fila}_Entrada_Vuelta",
            'x': round(x_giro_final, 2),
            'y': round(y_robot_vuelta, 2),
            'yaw': 180.0
        })

        # === VUELTA ===
        for i in range(num_plantas):
            idx_inverso = (num_plantas - 1) - i
            x_actual = start_x + (idx_inverso * distancia_entre_paradas)
            
            waypoints.append({
                'name': f"{nombre_fila}_LadoB_P{idx_inverso}",
                'x': round(x_actual, 2),
                'y': round(y_robot_vuelta, 2),
                'yaw': -90.0
            })

        # === TRANSICIÓN ===
        if i_fila < num_filas - 1:
            y_siguiente_fila_ida = (y_centro_fila + distancia_entre_filas) - distancia_lateral
            print(f"   -> Calculando transición a Fila {i_fila + 2}...")
            
            waypoints.append({
                'name': f"Salida_Fila_{i_fila+1}",
                'x': round(x_giro_inicio, 2),
                'y': round(y_robot_vuelta, 2),
                'yaw': 180.0
            })
            
            waypoints.append({
                'name': f"Inicio_Fila_{i_fila+2}",
                'x': round(x_giro_inicio, 2),
                'y': round(y_siguiente_fila_ida, 2),
                'yaw': 0.0
            })

    # --- GUARDADO ---
    output_path = os.path.expanduser('~/gonzalo_ws/src/sancho_navigation/config/my_route.yaml')
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    
    with open(output_path, 'w') as f:
        yaml.dump(waypoints, f, sort_keys=False)
        
    print(f"\n✅ RUTA GUARDADA EN: {output_path}")
    print("🚀 AHORA SÍ: Lanza el 'greenhouse_navigator' (Terminal 3).")

if __name__ == "__main__":
    # 1. Primero movemos el robot para localizarlo
    perform_localization_spin()
    # 2. Luego pedimos los datos para la ruta
    generate_route()