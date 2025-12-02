import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def _spawn_from_file(entity_name: str, file_path: str, x: float, y: float, z: float):
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_share, 'launch', 'gz_spawn_model.launch.py')),
        launch_arguments={
            'file': file_path, 'entity_name': entity_name,
            'x': str(x), 'y': str(y), 'z': str(z),
        }.items()
    )

def _bridge_cmd_vel(uav_name: str, gz_topic: str):
    # ROS -> GZ (gz.msgs.Twist)
    return Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=[f'{gz_topic}@geometry_msgs/msg/Twist]gz.msgs.Twist'],
        remappings=[(gz_topic, f'/{uav_name}/cmd_vel')],
        output='screen', name=f'{uav_name}_cmd_vel_bridge',
    )

def _bridge_odom(uav_name: str, gz_topic: str):
    # GZ -> ROS (gz.msgs.Odometry)
    return Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=[f'{gz_topic}@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
        remappings=[(gz_topic, f'/{uav_name}/odom')],
        output='screen', name=f'{uav_name}_odom_bridge',
    )

def _setup(context, *args, **kwargs):
    actions = []
    models_dir = LaunchConfiguration('models_dir').perform(context)
    ids_csv = LaunchConfiguration('ids').perform(context)
    z = float(LaunchConfiguration('z').perform(context))
    start_x = float(LaunchConfiguration('start_x').perform(context))
    spacing = float(LaunchConfiguration('spacing').perform(context))

    ids = [s.strip() for s in ids_csv.split(',') if s.strip()]
    for i, sid in enumerate(ids):
        uav = f'cf{sid}'
        file_path = os.path.join(models_dir, f'crazyflie_cf{sid}', 'model.sdf')
        x = start_x + i * spacing
        actions.append(_spawn_from_file(uav, file_path, x, 0.0, z))
        actions.append(_bridge_cmd_vel(uav, f'/{uav}/gazebo/command/twist'))
        actions.append(_bridge_odom(uav, f'/model/{uav}/odometry'))
    return actions

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('models_dir', default_value='/home/ricardo/crazyflie_mapping_demo/ros2_ws/src/ros_gz_crazyflie/ros_gz_crazyflie_gazebo/models'),
        DeclareLaunchArgument('ids', default_value='2,3,11', description='Comma-separated Crazyflie IDs to spawn (e.g., "2,3,11")'),
        DeclareLaunchArgument('z', default_value='0.35'),
        DeclareLaunchArgument('start_x', default_value='-1.0'),
        DeclareLaunchArgument('spacing', default_value='1.5'),
        OpaqueFunction(function=_setup),
    ])

