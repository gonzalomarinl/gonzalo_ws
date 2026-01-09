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
    """
    Analiza la planta y decide si lanzar una alerta.
    Retorna: (es_anomalia (bool), nombre_enfermedad (str))
    """
    
    # 1. SELECCIÓN DE LA IMAGEN
    final_image_to_process = None
    
    # --- LOGICA DE DIAGNÓSTICO ---
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
            # Fallback si no hay imágenes
            final_image_to_process = cv2.zeros((480, 640, 3), dtype="uint8")

        # SIMULACIÓN: 30% de probabilidad de plaga para probar el sistema
        if random.random() < 0.3:
            diagnostico = random.choice(["Oidio", "Minador", "Araña Roja"])
            confianza = random.uniform(85.0, 99.9)
        else:
            diagnostico = "Saludable"
            confianza = random.uniform(90.0, 99.9)

    else: # MODO REAL
        # Cargamos la foto que tomó el robot
        final_image_to_process = cv2.imread(image_path)
        
        if final_image_to_process is None:
            print(f"❌ Error: No se pudo cargar la imagen en {image_path}")
            return False, "Error de Lectura"

        # --- AQUÍ VA TU RED NEURONAL REAL ---
        # prediction = model(final_image_to_process)
        # diagnostico = prediction.label
        
        # POR AHORA (PRUEBA): Forzamos 'Saludable' para la demo, 
        # a menos que quieras descomentar la línea de abajo para probar la alarma:
        diagnostico = "Saludable"
        # diagnostico = "Oidio" # <--- DESCOMENTAR ESTO SI QUIERES FORZAR UNA ALERTA DE PRUEBA
        confianza = 95.5

    # 2. GESTIÓN POR EXCEPCIÓN (Reporte)
    es_anomalia = (diagnostico != "Saludable")

    if es_anomalia:
        # --- CASO: ALARMA (ROJO) ---
        print(f"\n🚨🚨 ¡ALERTA FITOSANITARIA! 🚨🚨")
        print(f"📍 Ubicación: {point_name}")
        print(f"🦠 Detectado: {diagnostico.upper()}")
        print(f"📉 Confianza: {confianza:.2f}%")
        print("-" * 30)
        
        # Guardamos evidencia visual
        save_evidence(final_image_to_process, point_name, diagnostico, confianza)
        return True, diagnostico
    else:
        # --- CASO: SALUDABLE (DISCRETO) ---
        print(f"✅ {point_name}: Planta sana. Sin novedades.")
        return False, diagnostico

def save_evidence(image, point_name, diagnostico, confianza):
    """Guarda la foto con el diagnóstico impreso sobre ella"""
    save_dir = os.path.expanduser('~/gonzalo_ws/plant_photos_results')
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    # Nombre de archivo destructivo: ALERTA_...
    filename = f"ALERTA_{point_name}_{diagnostico}_{timestamp}.jpg"
    save_path = os.path.join(save_dir, filename)

    if image is not None:
        # Dibujamos en rojo sobre la foto
        cv2.putText(image, f"ALERTA: {point_name}", (20, 40), 1, 2, (0, 0, 255), 2)
        cv2.putText(image, f"DETECTADO: {diagnostico}", (20, 80), 1, 2, (0, 0, 255), 3)
        cv2.imwrite(save_path, image)
        print(f"💾 Evidencia guardada en: {filename}")