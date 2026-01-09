#!/usr/bin/env python3
import os
import cv2
import random
import glob
from datetime import datetime

def analyze_plant(image_path, point_name, mode='sim'): # <--- Cambiado a 'sim'
    
    final_image_to_process = None
    diagnostico = "Saludable"
    confianza = 0.0

    if mode == 'sim':
        # Buscamos imágenes de prueba
        test_dir = os.path.expanduser('~/gonzalo_ws/src/sancho_navigation/test_images')
        valid_extensions = ('*.jpg', '*.JPG', '*.png', '*.jpeg')
        test_images = []
        for ext in valid_extensions:
            test_images.extend(glob.glob(os.path.join(test_dir, ext)))
        
        if test_images:
            chosen_path = random.choice(test_images)
            final_image_to_process = cv2.imread(chosen_path)
        else:
            import numpy as np
            final_image_to_process = np.zeros((480, 640, 3), dtype="uint8")

        # Probabilidad de plaga para pruebas
        if random.random() < 0.3:
            diagnostico = random.choice(["Oidio", "Minador", "Araña Roja"])
            confianza = random.uniform(85.0, 99.9)
        else:
            diagnostico = "Saludable"
            confianza = random.uniform(90.0, 99.9)

    else: # MODO REAL
        final_image_to_process = cv2.imread(image_path)
        if final_image_to_process is None:
            print(f"❌ Error: No se pudo cargar la imagen en {image_path}")
            return False, "Error de Lectura"
        
        # --- AQUÍ CONECTARÁS TU MODELO DE TORCH/TENSORFLOW ---
        diagnostico = "Saludable" 
        confianza = 95.5

    es_anomalia = (diagnostico != "Saludable")

    if es_anomalia:
        print(f"\n🚨🚨 ¡ALERTA FITOSANITARIA EN {point_name.upper()}! 🚨🚨")
        print(f"🦠 Detectado: {diagnostico.upper()} | Confianza: {confianza:.2f}%")
        save_evidence(final_image_to_process, point_name, diagnostico, confianza)
        return True, diagnostico
    else:
        print(f"✅ {point_name}: Planta sana.")
        return False, diagnostico

def save_evidence(image, point_name, diagnostico, confianza):
    save_dir = os.path.expanduser('~/gonzalo_ws/plant_photos_results')
    if not os.path.exists(save_dir):
        os.makedirs(save_dir, exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"ALERTA_{point_name}_{diagnostico}_{timestamp}.jpg"
    save_path = os.path.join(save_dir, filename)

    if image is not None:
        # Copia para no machacar la original
        img_out = image.copy()
        cv2.putText(img_out, f"ALERTA: {point_name}", (20, 40), 1, 2, (0, 0, 255), 2)
        cv2.putText(img_out, f"{diagnostico} ({confianza:.1f}%)", (20, 80), 1, 2, (0, 0, 255), 3)
        cv2.imwrite(save_path, img_out)
        print(f"💾 Evidencia guardada en: {save_path}")