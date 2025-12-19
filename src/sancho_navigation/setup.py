from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sancho_navigation'

# --- FUNCIÓN AUXILIAR PARA COPIAR CARPETAS RECURSIVAMENTE ---
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
    ('share/' + package_name + '/launch', glob('launch/*.py')),
    ('share/' + package_name + '/config', glob('config/*.yaml')),
    ('share/' + package_name + '/worlds', glob('worlds/*.world')),
    ('share/' + package_name + '/maps', glob('maps/*')),
    # --- CORRECCIÓN 1: AÑADIR LAS FOTOS DE PRUEBA ---
    ('share/' + package_name + '/test_images', glob('test_images/*')),
]

# Añadimos todos los archivos que haya dentro de 'models' automáticamente
data_files.extend(package_files('models'))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gonzalomarinl',
    maintainer_email='gonzalomarinl@todo.todo',
    description='Navigation package for greenhouse robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # El navegador principal
            'greenhouse_navigator = sancho_navigation.greenhouse_navigator:main',
            
            # --- CORRECCIÓN 2: AÑADIDO EL GENERADOR DE RUTAS NUEVO ---
            # Nota: apunta a la función generate_route dentro del archivo lineal_route_generator
            'lineal_route_generator = sancho_navigation.lineal_route_generator:generate_route',
            
            # --- CORRECCIÓN 3: COMENTADOS LOS SCRIPTS QUE NO EXISTEN EN SRC ---
            # Si en el futuro creas estos archivos .py, descomenta estas líneas:
            # 'waypoint_saver = sancho_navigation.waypoint_saver:main',
            # 'plant_doctor = sancho_navigation.plant_doctor:main',
            
            # Mantenemos este si el archivo route_generator.py viejo sigue ahí, 
            # aunque recomiendo usar solo el lineal.
            'route_generator = sancho_navigation.route_generator:main',
        ],
    },
)