import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    sancho_nav_dir = get_package_share_directory('sancho_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # ARGUMENTOS
    # Apuntamos por defecto a la carpeta SRC para evitar el error "bad file" de la carpeta install
    # AJUSTA ESTA RUTA A TU USUARIO SI NO ERES 'gonzalo'
    default_map_path = '/home/gonzalo/tfg_ws/src/sancho_navigation/maps/greenhouse_map.yaml'
    
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    declare_map = DeclareLaunchArgument(
        'map', default_value=default_map_path,
        description='Ruta completa al mapa yaml')

    declare_use_sim = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Usar tiempo de simulación')
        
    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(sancho_nav_dir, 'config', 'nav2_params.yaml'),
        description='Ruta a nav2_params.yaml')

    # 1. LANZAR MAP SERVER Y AMCL (LOCALIZATION)
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true'
        }.items()
    )

    # 2. RVIZ (Para ver que el mapa carga bien antes de seguir)
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        declare_map,
        declare_use_sim,
        declare_params,
        localization_launch,
        rviz_node
    ])