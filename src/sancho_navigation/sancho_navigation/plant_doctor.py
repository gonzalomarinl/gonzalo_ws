#!/usr/bin/env python3
import os
import cv2
import random
import glob
import torch 
import numpy as np
from datetime import datetime
from torchvision import transforms 
from ament_index_python.packages import get_package_share_directory

# --- CONFIGURACIÓN DEL MODELO ---
CLASSES = [
    "Early_Blight",      
    "Healthy",           
    "Late_blight",       
    "black spot",         
    "Bacterial Spot",     
    "Leaf Mold",           
    "Target_Spot"
]

def load_model_robust(model_path, device):
    try:
        checkpoint = torch.load(model_path, map_location=device)
        if isinstance(checkpoint, dict) or "OrderedDict" in str(type(checkpoint)):
            from torchvision import models
            model = models.resnet18(weights=None) 
            model.fc = torch.nn.Linear(model.fc.in_features, len(CLASSES))
            model.load_state_dict(checkpoint)
        else:
            model = checkpoint
        model.eval()
        return model
    except Exception as e:
        print(f"❌ Error cargando modelo: {e}")
        return None

# Inicialización Global
try:
    package_share_dir = get_package_share_directory('sancho_navigation')
    MODEL_PATH = os.path.join(package_share_dir, 'models', 'best_model_optimized.pth')
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = load_model_robust(MODEL_PATH, device)
    if model:
        print(f"🤖 IA: Modelo cargado correctamente.")
except:
    model = None

preprocess = transforms.Compose([
    transforms.ToPILImage(),
    transforms.Resize((224, 224)),
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
])

def analyze_plant(image_path, point_name, mode='sim'):
    raw_img = cv2.imread(image_path)
    if raw_img is None:
        return False, "Error de Lectura", 0.0

    # --- ZOOM DIGITAL (RECORTE CENTRAL) ---
    # La planta se ve pequeña en el centro. Recortamos para que ocupe más espacio.
    h, w, _ = raw_img.shape
    # Definimos un cuadro de 400x400 píxeles en el centro de la imagen
    # Ajustamos y para que capture un poco más abajo (donde suelen estar las hojas)
    cx, cy = w // 2, (h // 2) + 80    
    size = 150 # Mitad del tamaño del cuadro (total 400px)
    crop_img = raw_img[cy-size:cy+size, cx-size:cx+size]
    
    # Si por algún motivo el recorte falla, usamos la imagen original
    final_image_to_process = crop_img if crop_img.size > 0 else raw_img
    # --------------------------------------

    diagnostico = "Healthy"
    confianza = 0.0

    if model is not None:
        try:
            img_rgb = cv2.cvtColor(final_image_to_process, cv2.COLOR_BGR2RGB)
            input_tensor = preprocess(img_rgb).unsqueeze(0).to(device)
            with torch.no_grad():
                output = model(input_tensor)
                probs = torch.nn.functional.softmax(output[0], dim=0)
            conf, index = torch.max(probs, 0)
            diagnostico = CLASSES[index.item()]
            confianza = conf.item() * 100
        except Exception as e:
            print(f"Error inferencia: {e}")
            diagnostico = "Error IA"
    else:
        diagnostico = "Healthy"
        confianza = 95.0

    es_anomalia = (diagnostico != "Healthy")
    if es_anomalia:
        save_evidence(final_image_to_process, point_name, diagnostico, confianza)
    
    return es_anomalia, diagnostico, confianza

def save_evidence(image, point_name, diagnostico, confianza):
    save_dir = os.path.expanduser('~/gonzalo_ws/plant_photos_results')
    os.makedirs(save_dir, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"ALERTA_{point_name}_{diagnostico}_{timestamp}.jpg"
    cv2.imwrite(os.path.join(save_dir, filename), image)