import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # --- 1. Definir rutas y variables ---
    sancho_nav_dir = get_package_share_directory('sancho_navigation')
    
    # Ruta al archivo de parámetros que acabas de crear
    slam_params_file = os.path.join(sancho_nav_dir, 'config', 'mapper_params_online_async.yaml')

    # Configuración de argumentos
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='false',  # <--- IMPORTANTE: 'false' para ROBOT REAL
        description='Use simulation/Gazebo clock')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=slam_params_file,
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    # --- 2. El algoritmo de SLAM ---
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': params_file # <--- Aquí pasamos TU configuración
        }.items()
    )

    # --- 3. El visualizador RViz ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # parameters=[{'use_sim_time': use_sim_time}] # Opcional: descomentar si rviz da problemas de TF
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        slam_launch,
        rviz_node
    ])