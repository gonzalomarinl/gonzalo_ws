import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. DEFINICIÓN DE DIRECTORIOS Y RUTAS ABSOLUTAS (STRINGS PUROS)
    # Esto asegura que Python calcule la ruta ANTES de que ROS empiece a pensar
    sancho_nav_dir = get_package_share_directory('sancho_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    map_file_path = os.path.join(sancho_nav_dir, 'maps', 'greenhouse_map.yaml')
    params_file_path = os.path.join(sancho_nav_dir, 'config', 'nav2_params_sim.yaml')

    # 2. CONFIGURACIÓN DE LANZAMIENTO (VARIABLES ROS)
    # Estas variables leerán lo que venga de la terminal o tomarán el default
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')

    # 3. DECLARACIÓN DE ARGUMENTOS
    # Aquí usamos las rutas 'string' calculadas arriba como default
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Usar tiempo de simulación (Gazebo)')

    declare_map_yaml = DeclareLaunchArgument(
        'map',
        default_value=map_file_path, # <--- CAMBIO CLAVE: Pasamos la ruta, no la config
        description='Ruta completa al archivo del mapa .yaml')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=params_file_path,
        description='Ruta al archivo de parametros nav2')
        
    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Arrancar automáticamente el stack de navegación')

    # 4. INCLUIR EL LAUNCH DE NAV2
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml,
            'params_file': params_file,
            'autostart': autostart
        }.items()
    )

    # 5. RVIZ
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
        # Opcional: Cargar config propia si la tienes
        # arguments=['-d', os.path.join(sancho_nav_dir, 'rviz', 'nav2_view.rviz')]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_map_yaml,
        declare_params_file,
        declare_autostart,
        nav2_bringup_launch,
        rviz_node
    ])