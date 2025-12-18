import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # 1. Definir rutas
    sancho_nav_dir = get_package_share_directory('sancho_navigation')
    slam_params_file = os.path.join(sancho_nav_dir, 'config', 'mapper_params_online_async.yaml')

    # 2. Argumentos
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='true',  # <--- CORREGIDO: Por defecto a TRUE para simulación
        description='Use simulation/Gazebo clock')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=slam_params_file,
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    # 3. Nodo SLAM
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
            'slam_params_file': params_file
        }.items()
    )

    # 4. Nodo RViz (CORREGIDO)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}] # <--- ¡IMPORTANTE! SIN #
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        slam_launch,
        rviz_node
    ])