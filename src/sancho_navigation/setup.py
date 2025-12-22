from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sancho_navigation'

# --- FUNCIÓN AUXILIAR PARA COPIAR CARPETAS RECURSIVAMENTE ---
# Necesario para copiar todos los modelos 3D y texturas de la carpeta 'models'
def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            file_path = os.path.join(path, filename)
            install_path = os.path.join('share', package_name, path)
            paths.append((install_path, [file_path]))
    return paths

# --- DEFINICIÓN DE ARCHIVOS A INSTALAR ---
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    # Copiar launch files
    ('share/' + package_name + '/launch', glob('launch/*.py')),
    # Copiar configuración YAML
    ('share/' + package_name + '/config', glob('config/*.yaml')),
    # Copiar mundos de Gazebo
    ('share/' + package_name + '/worlds', glob('worlds/*.world')),
    # Copiar mapas (.pgm y .yaml)
    ('share/' + package_name + '/maps', glob('maps/*')),
    # Copiar imágenes de prueba para el modo Simulación
    ('share/' + package_name + '/test_images', glob('test_images/*')),
]

# Añadimos recursivamente todo el contenido de la carpeta 'models' (meshes, materials, etc.)
data_files.extend(package_files('models'))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gonzalomarinl',
    maintainer_email='gonzalomarinl@uma.es', # Corregido email genérico
    description='Navigation package for greenhouse robot (Sancho)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 1. El cerebro principal (Máquina de estados + Navegación)
            'greenhouse_navigator = sancho_navigation.greenhouse_navigator:main',
            
            # 2. La herramienta de calibración y generación de rutas (Bolas rojas)
            'route_visualizer = sancho_navigation.route_visualizer:main',
            
            # NOTA: 'plant_doctor' no se registra aquí porque se ejecuta como 
            # subproceso independiente o script directo.
        ],
    },
)