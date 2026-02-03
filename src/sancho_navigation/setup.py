from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sancho_navigation'

# --- FUNCIÓN AUXILIAR MEJORADA ---
# Asegura que todos los archivos en subcarpetas se instalen correctamente
def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            file_path = os.path.join(path, filename)
            # Calculamos la ruta de instalación relativa a 'share/package_name'
            install_path = os.path.join('share', package_name, path)
            paths.append((install_path, [file_path]))
    return paths

# --- LISTA DE ARCHIVOS DE DATOS ---
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    # Instalación de carpetas principales usando glob
    ('share/' + package_name + '/launch', glob('launch/*.py')),
    ('share/' + package_name + '/config', glob('config/*.yaml')),
    ('share/' + package_name + '/worlds', glob('worlds/*.world')),
    # Especificamos extensiones para mapas para evitar capturar carpetas temporales de git
    ('share/' + package_name + '/maps', glob('maps/*.yaml')),
    ('share/' + package_name + '/maps', glob('maps/*.pgm')),
    ('share/' + package_name + '/test_images', glob('test_images/*.jpg')),
]

# Añadimos recursivamente carpetas que pueden tener subniveles (como models)
data_files.extend(package_files('models'))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gonzalomarinl',
    maintainer_email='gonzalomarinl@uma.es',
    description='Navigation package for greenhouse robot (Sancho)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'greenhouse_navigator = sancho_navigation.greenhouse_navigator:main',
            'route_visualizer = sancho_navigation.route_visualizer:main',
        ],
    },
)