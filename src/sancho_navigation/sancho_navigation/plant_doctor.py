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
    """Intenta cargar el modelo manejando state_dicts o modelos completos."""
    try:
        # Cargamos el archivo .pth
        checkpoint = torch.load(model_path, map_location=device)
        
        # SI ES UN STATE_DICT (OrderedDict), necesitas la CLASE del modelo
        if isinstance(checkpoint, dict) or str(type(checkpoint)) == "<class 'collections.OrderedDict'>":
            print("⚠️ IA: Se detectó un 'state_dict'. Cargando pesos sobre arquitectura genérica...")
            # NOTA: Ajusta 'models.resnet18' si tu arquitectura es otra (MobileNet, etc.)
            from torchvision import models
            # Creamos una estructura compatible con tus 7 clases
            model = models.resnet18(weights=None) 
            model.fc = torch.nn.Linear(model.fc.in_features, len(CLASSES))
            model.load_state_dict(checkpoint)
        else:
            # SI ES EL MODELO COMPLETO
            model = checkpoint
            
        model.eval()
        return model
    except Exception as e:
        print(f"❌ Error crítico cargando modelo: {e}")
        return None

# Inicialización del modelo
try:
    package_share_dir = get_package_share_directory('sancho_navigation')
    MODEL_PATH = os.path.join(package_share_dir, 'models', 'best_model_optimized.pth')
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    
    model = load_model_robust(MODEL_PATH, device)
    if model:
        print(f"🤖 IA: Modelo cargado correctamente desde {MODEL_PATH}")
except Exception as e:
    model = None
    print(f"⚠️ IA: Fallo general. Usando lógica de respaldo.")

# Preprocesamiento (Mantenido igual)
preprocess = transforms.Compose([
    transforms.ToPILImage(),
    transforms.Resize((224, 224)),
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
])

# ... El resto de las funciones (analyze_plant, save_evidence) se mantienen igual ...
# (Asegúrate de copiar el resto de tu código original debajo)