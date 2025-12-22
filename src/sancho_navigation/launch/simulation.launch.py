import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_sancho = get_package_share_directory('sancho_navigation')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    # 1. Definir rutas
    world_file = os.path.join(pkg_sancho, 'worlds', 'greenhouse.world')
    models_dir = os.path.join(pkg_sancho, 'models')

    # 2. Variables de entorno
    gazebo_models_path = models_dir
    if 'GAZEBO_MODEL_PATH' in os.environ:
        gazebo_models_path += ":" + os.environ['GAZEBO_MODEL_PATH']

    set_gazebo_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH', gazebo_models_path)

    # 3. Gazebo Server (Física)
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    # 4. Gazebo Client (Gráficos)
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # 5. Estado del Robot (TF)
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 6. Spawn del Robot (CON RETRASO DE 10 SEGUNDOS)
    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': '-3.0',
            'y_pose': '-1.0',
            'z_pose': '0.01'
        }.items()
    )

    # Envolvemos el spawn en un TimerAction
    delayed_spawn = TimerAction(
        period=10.0, # Espera 10 segundos antes de lanzar el robot
        actions=[spawn_turtlebot_cmd]
    )

    return LaunchDescription([
        set_gazebo_model_path,
        gzserver_cmd,
        gzclient_cmd,
        robot_state_publisher_cmd,
        delayed_spawn # Usamos la versión con retraso
    ])