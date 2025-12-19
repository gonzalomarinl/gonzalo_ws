#!/usr/bin/env python3
import argparse
import time
import os
import cv2
import random
import glob
from datetime import datetime

# --- ESPACIO PARA IA (Preparado para tu modelo) ---
# import torch
# from torchvision import transforms

def analyze_plant(image_path, point_name, mode='simulation'):
    print(f"\n--- 👨‍⚕️ PLANT DOCTOR: Modo {mode.upper()} en {point_name} ---")
    
    # 1. SELECCIÓN DE LA IMAGEN
    final_image_to_process = None
    
    if mode == 'simulation':
        # Buscamos en tu carpeta de imágenes de prueba
        test_dir = os.path.expanduser('~/gonzalo_ws/src/sancho_navigation/test_images')
        # Buscamos archivos .jpg, .JPG, .png, etc.
        valid_extensions = ('*.jpg', '*.JPG', '*.png', '*.jpeg')
        test_images = []
        for ext in valid_extensions:
            test_images.extend(glob.glob(os.path.join(test_dir, ext)))
        
        if test_images:
            # Elegimos una al azar para que cada parada sea diferente
            chosen_path = random.choice(test_images)
            final_image_to_process = cv2.imread(chosen_path)
            print(f"🎲 Simulación: Usando imagen aleatoria: {os.path.basename(chosen_path)}")
        else:
            print(f"⚠️ No hay imágenes en {test_dir}. Usando captura de Gazebo.")
            final_image_to_process = cv2.imread(image_path)
    else:
        # MODO REAL: Usamos la captura directa que ha guardado el robot
        final_image_to_process = cv2.imread(image_path)
        print("📷 Modo Real: Analizando captura directa de la cámara.")

    if final_image_to_process is None:
        print("❌ Error crítico: No se ha podido cargar ninguna imagen para analizar.")
        return

    # 2. PROCESAMIENTO IA (Los 10 segundos de rigor)
    print("🧠 Ejecutando redes neuronales de detección...")
    # Simulación de carga y análisis
    time.sleep(10.0) 

    # --- Aquí iría tu lógica de inferencia real ---
    # diagnosis = mi_modelo(final_image_to_process)
    resultados = ["Saludable", "Oidio", "Araña Roja", "Falta de Riego"]
    diagnostico = random.choice(resultados) if mode == 'simulation' else "Bajo Análisis"
    confianza = random.uniform(85.0, 99.9)
    # ----------------------------------------------

    print(f"✅ Resultado: {diagnostico} ({confianza:.2f}%)")

    # 3. GUARDADO DE RESULTADOS PARA EL OPERARIO
    save_dir = os.path.expanduser('~/gonzalo_ws/plant_photos_results')
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{point_name}_{diagnostico}_{timestamp}.jpg"
    save_path = os.path.join(save_dir, filename)

    # Dibujamos el diagnóstico en la imagen para el registro visual
    color = (0, 255, 0) if diagnostico == "Saludable" else (0, 0, 255)
    cv2.putText(final_image_to_process, f"Punto: {point_name}", (20, 40), 1, 2, (255, 255, 255), 2)
    cv2.putText(final_image_to_process, f"DX: {diagnostico} ({confianza:.1f}%)", (20, 80), 1, 2, color, 3)

    cv2.imwrite(save_path, final_image_to_process)
    print(f"💾 Informe médico guardado: {save_path}")
    print("--- 👨‍⚕️ DOCTOR: Finalizado ---\n")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", required=True, help="Ruta de la captura del robot")
    parser.add_argument("--name", required=True, help="Nombre del waypoint")
    parser.add_argument("--mode", default="simulation", choices=["simulation", "real"], help="Modo de ejecución")
    
    args = parser.parse_args()
    analyze_plant(args.image, args.name, args.mode)