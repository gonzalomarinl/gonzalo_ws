import cv2
import numpy as np
import os

# --- CONFIGURACIÓN ---
BASE_DIR = os.path.expanduser("~/gonzalo_ws/src/sancho_navigation/maps")
INPUT_NAME = "greenhouse_raw"
OUTPUT_NAME = "greenhouse_map"

# Unir puntos cercanos
KERNEL_SIZE = 12

def process_map():
    input_pgm = os.path.join(BASE_DIR, INPUT_NAME + ".pgm")
    input_yaml = os.path.join(BASE_DIR, INPUT_NAME + ".yaml")
    output_pgm = os.path.join(BASE_DIR, OUTPUT_NAME + ".pgm")
    output_yaml = os.path.join(BASE_DIR, OUTPUT_NAME + ".yaml")

    print(f"🔄 RE-INTENTO: Procesando {INPUT_NAME}...")

    # 1. Cargar imagen
    img = cv2.imread(input_pgm, cv2.IMREAD_GRAYSCALE)
    if img is None:
        print("❌ Error: No encuentro el mapa raw.")
        return

    # --- CORRECCIÓN DEL "PEDAZO DE MIERDA" ---
    # Usamos umbral 100.
    # - Lo que sea más oscuro que 100 (Plantas) se vuelve BLANCO en la máscara.
    # - Lo que sea más claro que 100 (Gris 205 y Blanco 255) se vuelve NEGRO (Fondo).
    _, thresh = cv2.threshold(img, 100, 255, cv2.THRESH_BINARY_INV)

    # 2. Dilatar (Unir los puntos de las plantas)
    kernel = np.ones((KERNEL_SIZE, KERNEL_SIZE), np.uint8)
    dilated = cv2.dilate(thresh, kernel, iterations=2)

    # 3. Detectar contornos
    contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 4. Crear lienzo BLANCO LIMPIO
    clean_img = np.ones_like(img) * 255

    count = 0
    print(f"📊 Analizando contornos detectados...")
    
    for cnt in contours:
        area = cv2.contourArea(cnt)
        # Filtramos ruido muy pequeño (menos de 300px)
        # Y filtramos "el mundo entero" (si el área es gigantesca, es que ha pillado el borde)
        if 300 < area < (img.shape[0] * img.shape[1] * 0.9):
            x, y, w, h = cv2.boundingRect(cnt)
            # Pintar rectángulo NEGRO (Obstáculo) sobre fondo blanco
            cv2.rectangle(clean_img, (x, y), (x+w, y+h), (0, 0, 0), -1)
            count += 1
            print(f"   -> Bloque {count}: Tamaño {w}x{h} píxeles")

    # 5. Guardar
    cv2.imwrite(output_pgm, clean_img)
    print(f"✅ ¡Arreglado! Imagen guardada: {output_pgm}")
    print(f"✅ Se han creado {count} rectángulos negros.")

    # 6. Actualizar YAML (solo por si acaso no estaba ya hecho)
    if os.path.exists(input_yaml):
        with open(input_yaml, 'r') as file:
            lines = file.readlines()
        with open(output_yaml, 'w') as file:
            for line in lines:
                if "image:" in line:
                    file.write(f"image: {OUTPUT_NAME}.pgm\n")
                else:
                    file.write(line)

if __name__ == "__main__":
    process_map()