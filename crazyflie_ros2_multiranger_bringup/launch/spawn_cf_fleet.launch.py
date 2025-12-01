# Save as: crazyflie_ros2_multiranger_bringup/launch/spawn_cf_fleet.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def _spawn_from_file(entity_name: str, file_path: str, x: float, y: float, z: float):
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_share, 'launch', 'gz_spawn_model.launch.py')),
        launch_arguments={
            'file': file_path,
            'entity_name': entity_name,
            'x': str(x), 'y': str(y), 'z': str(z),
        }.items()
    )

def _cmd_vel_bridge(uav_name: str, gz_topic: str):
    return Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[f'{gz_topic}@geometry_msgs/msg/Twist]gz.msgs.Twist'],
        remappings=[(gz_topic, f'/{uav_name}/cmd_vel')],
        output='screen',
        name=f'{uav_name}_cmd_vel_bridge',
    )

def _setup(context, *args, **kwargs):
    actions = []
    N = int(LaunchConfiguration('count').perform(context))
    models_dir = LaunchConfiguration('models_dir').perform(context)
    start_x = float(LaunchConfiguration('start_x').perform(context))
    spacing = float(LaunchConfiguration('spacing').perform(context))
    z = float(LaunchConfiguration('z').perform(context))

    if N >= 1:
        actions.append(_cmd_vel_bridge('cf1', '/crazyflie/gazebo/command/twist'))

    for i in range(2, N+1):
        uav = f'cf{i}'
        file_path = os.path.join(models_dir, f'crazyflie_{uav}', 'model.sdf')
        x = start_x + (i-1) * spacing
        actions.append(_spawn_from_file(uav, file_path, x, 0.0, z))
        actions.append(_cmd_vel_bridge(uav, f'/{uav}/gazebo/command/twist'))
    return actions

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('count', default_value='3', description='How many vehicles to control (1..10)'),
        DeclareLaunchArgument('models_dir', default_value='/home/ricardo/crazyflie_mapping_demo/ros2_ws/src/ros_gz_crazyflie/ros_gz_crazyflie_gazebo/models',
                              description='Directory containing crazyflie_cfX folders with model.sdf & model.config'),
        DeclareLaunchArgument('start_x', default_value='-1.5', description='Initial X for cf1; others spaced by "spacing"'),
        DeclareLaunchArgument('spacing', default_value='1.5', description='X spacing between consecutive UAVs'),
        DeclareLaunchArgument('z', default_value='0.35', description='Spawn altitude'),
        OpaqueFunction(function=_setup),
    ])