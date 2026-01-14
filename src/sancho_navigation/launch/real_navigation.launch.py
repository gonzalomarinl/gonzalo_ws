import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Directorios clave
    sancho_nav_dir = get_package_share_directory('sancho_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Rutas físicas directas para evitar el error 'parameter yaml_filename is not initialized'
    default_map_path = os.path.join(sancho_nav_dir, 'maps', 'greenhouse_map.yaml')
    default_params_path = os.path.join(sancho_nav_dir, 'config', 'nav2_params_real.yaml')

    # 2. Argumentos de lanzamiento (LaunchConfiguration)
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    # 3. Declaración de argumentos de entrada
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Usar tiempo de simulación (Falso para robot real)')
    
    declare_map_yaml = DeclareLaunchArgument(
        'map', default_value=default_map_path,
        description='Ruta completa al archivo del mapa .yaml')
        
    declare_params_file = DeclareLaunchArgument(
        'params_file', default_value=default_params_path,
        description='Ruta completa al archivo de parámetros nav2')

    # 4. Incluir el launch principal de Nav2 (Bringup)
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml_file,
            'params_file': params_file,
            'autostart': 'true'
        }.items()
    )

    # 5. Añadir RViz (Visualizador)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': False}], # Tiempo real siempre en RViz para hardware
        arguments=['-d', os.path.join(sancho_nav_dir, 'rviz', 'nav2_view.rviz')]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_map_yaml,
        declare_params_file,
        nav2_bringup_launch,
        rviz_node
    ])