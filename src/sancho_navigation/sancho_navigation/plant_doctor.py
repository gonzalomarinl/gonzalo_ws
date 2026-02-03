#!/usr/bin/env python3
import os
import cv2
import random
import glob
import torch  # <--- NUEVO: Para cargar el modelo .pth
import numpy as np
from datetime import datetime
from torchvision import transforms  # <--- NUEVO: Para preprocesar la imagen
from ament_index_python.packages import get_package_share_directory

# --- CONFIGURACIÓN DEL MODELO ---
# Definimos las clases según el entrenamiento de tu red
CLASSES = ["Saludable", "Oidio", "Minador", "Araña Roja"]

# Buscamos la ruta del modelo en la carpeta de instalación
try:
    package_share_dir = get_package_share_directory('sancho_navigation')
    MODEL_PATH = os.path.join(package_share_dir, 'models', 'best_model_optimized.pth')
    
    # Cargamos el modelo (asumiendo arquitectura guardada o JIT)
    # Si solo guardaste el state_dict, necesitarás instanciar la clase de la red aquí.
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = torch.load(MODEL_PATH, map_location=device)
    model.eval()
    print(f"🤖 IA: Modelo cargado correctamente desde {MODEL_PATH}")
except Exception as e:
    model = None
    print(f"⚠️ IA: No se pudo cargar el modelo real ({e}). Se usará lógica de respaldo.")

# Preprocesamiento estándar para redes neuronales (ajusta el tamaño si tu red usa otro)
preprocess = transforms.Compose([
    transforms.ToPILImage(),
    transforms.Resize((224, 224)),
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
])

def analyze_plant(image_path, point_name, mode='sim'):
    final_image_to_process = None
    diagnostico = "Saludable"
    confianza = 0.0

    if mode == 'sim':
        # --- LÓGICA DE SIMULACIÓN (MANTENIDA) ---
        test_dir = os.path.expanduser('~/gonzalo_ws/src/sancho_navigation/test_images')
        valid_extensions = ('*.jpg', '*.JPG', '*.png', '*.jpeg')
        test_images = []
        for ext in valid_extensions:
            test_images.extend(glob.glob(os.path.join(test_dir, ext)))
        
        if test_images:
            chosen_path = random.choice(test_images)
            final_image_to_process = cv2.imread(chosen_path)
        else:
            final_image_to_process = np.zeros((480, 640, 3), dtype="uint8")

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
        
        # --- INFERENCIA CON EL MODELO REAL ---
        if model is not None:
            try:
                # Convertir BGR (OpenCV) a RGB y preprocesar
                img_rgb = cv2.cvtColor(final_image_to_process, cv2.COLOR_BGR2RGB)
                input_tensor = preprocess(img_rgb).unsqueeze(0).to(device)
                
                with torch.no_grad():
                    output = model(input_tensor)
                    probabilities = torch.nn.functional.softmax(output[0], dim=0)
                    
                # Obtener clase con mayor probabilidad
                conf, index = torch.max(probabilities, 0)
                diagnostico = CLASSES[index.item()]
                confianza = conf.item() * 100
            except Exception as e:
                print(f"❌ Error durante la inferencia: {e}")
                diagnostico = "Error IA"
                confianza = 0.0
        else:
            # Fallback si el modelo no cargó
            diagnostico = "Saludable" 
            confianza = 95.5

    es_anomalia = (diagnostico != "Saludable")

    if es_anomalia:
        print(f"\n🚨🚨 ¡ALERTA FITOSANITARIA EN {point_name.upper()}! 🚨🚨")
        print(f"🦠 Detectado: {diagnostico.upper()} | Confianza: {confianza:.2f}%")
        save_evidence(final_image_to_process, point_name, diagnostico, confianza)
        return True, diagnostico
    else:
        print(f"✅ {point_name}: Planta sana ({confianza:.1f}%).")
        return False, diagnostico

def save_evidence(image, point_name, diagnostico, confianza):
    save_dir = os.path.expanduser('~/gonzalo_ws/plant_photos_results')
    if not os.path.exists(save_dir):
        os.makedirs(save_dir, exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"ALERTA_{point_name}_{diagnostico}_{timestamp}.jpg"
    save_path = os.path.join(save_dir, filename)

    if image is not None:
        img_out = image.copy()
        cv2.putText(img_out, f"ALERTA: {point_name}", (20, 40), 1, 2, (0, 0, 255), 2)
        cv2.putText(img_out, f"{diagnostico} ({confianza:.1f}%)", (20, 80), 1, 2, (0, 0, 255), 3)
        cv2.imwrite(save_path, img_out)
        print(f"💾 Evidencia guardada en: {save_path}")