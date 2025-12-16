import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    sancho_nav_dir = get_package_share_directory('sancho_navigation')
    
    # Solo necesitamos la ruta a los parámetros (el mapa ya está escrito dentro de ellos)
    params_file = os.path.join(sancho_nav_dir, 'config', 'nav2_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Usar tiempo de simulación')
    
    execution_mode = LaunchConfiguration('execution_mode')
    declare_execution_mode = DeclareLaunchArgument(
        'execution_mode', default_value='simulation',
        description='Modo: simulation o real')

    camera_topic = LaunchConfiguration('camera_topic')
    declare_camera_topic = DeclareLaunchArgument(
        'camera_topic', default_value='/camera/image_raw',
        description='Topic de la camara')

    # INCLUIR NAVEGACIÓN
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sancho_nav_dir, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file
        }.items()
    )

    # TU NODO PRINCIPAL
    greenhouse_navigator_node = Node(
        package='sancho_navigation',
        executable='greenhouse_navigator',
        name='greenhouse_navigator',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'camera_topic': camera_topic},
            {'execution_mode': execution_mode}
        ]
    )

    # RVIZ
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_execution_mode,
        declare_camera_topic,
        navigation_launch,
        greenhouse_navigator_node,
        rviz_node
    ])