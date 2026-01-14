import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Definición de directorios
    sancho_nav_dir = get_package_share_directory('sancho_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # 2. RUTAS FÍSICAS (Strings directos) - Esto soluciona el error del map_server
    # Definimos las rutas como texto plano antes de crear los argumentos
    default_map_path = os.path.join(sancho_nav_dir, 'maps', 'greenhouse_map.yaml')
    default_params_path = os.path.join(sancho_nav_dir, 'config', 'nav2_params_real.yaml')

    # 3. Declaración de argumentos de lanzamiento
    # Usamos las rutas directas (strings) como 'default_value'
    declare_map_yaml = DeclareLaunchArgument(
        'map', 
        default_value=default_map_path,
        description='Ruta absoluta al archivo del mapa .yaml')

    declare_params_file = DeclareLaunchArgument(
        'params_file', 
        default_value=default_params_path,
        description='Ruta absoluta al archivo de parámetros nav2')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='false',
        description='Falso para el robot físico Sancho')

    # 4. Incluir el launch principal de Nav2 (Bringup)
    # Aquí usamos LaunchConfiguration para capturar los valores finales
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'autostart': 'true'
        }.items()
    )

    # 5. Nodo de RViz (Visualización)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=['-d', os.path.join(sancho_nav_dir, 'rviz', 'nav2_view.rviz')]
    )

    return LaunchDescription([
        declare_map_yaml,
        declare_params_file,
        declare_use_sim_time,
        nav2_bringup_launch,
        rviz_node
    ])