import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    sancho_nav_dir = get_package_share_directory('sancho_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    declare_use_sim = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Usar tiempo de simulación')
        
    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(sancho_nav_dir, 'config', 'nav2_params.yaml'),
        description='Ruta a nav2_params.yaml')

    # LANZAR SOLO LA NAVEGACIÓN (Sin mapa, sin AMCL)
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true'
        }.items()
    )

    return LaunchDescription([
        declare_use_sim,
        declare_params,
        navigation_launch
    ])