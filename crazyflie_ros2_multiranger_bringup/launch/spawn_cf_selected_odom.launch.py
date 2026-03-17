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
            'file': file_path,
            'entity_name': entity_name,
            'x': str(x),
            'y': str(y),
            'z': str(z),
        }.items(),
    )


def _bridge_cmd_vel(uav_name: str, gz_topic: str):
    return Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=[f'{gz_topic}@geometry_msgs/msg/Twist]gz.msgs.Twist'],
        remappings=[(gz_topic, f'/{uav_name}/cmd_vel')],
        output='screen', name=f'{uav_name}_cmd_vel_bridge',
    )


def _bridge_odom(uav_name: str, gz_topic: str):
    return Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=[f'{gz_topic}@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
        remappings=[(gz_topic, f'/{uav_name}/odom')],
        output='screen', name=f'{uav_name}_odom_bridge',
    )


def _parse_csv_floats(text: str):
    vals = []
    for s in text.split(','):
        s = s.strip()
        if s:
            vals.append(float(s))
    return vals


def _setup(context, *args, **kwargs):
    actions = []
    models_dir = LaunchConfiguration('models_dir').perform(context)
    ids_csv = LaunchConfiguration('ids').perform(context)
    z = float(LaunchConfiguration('z').perform(context))

    start_x = float(LaunchConfiguration('start_x').perform(context))
    spacing = float(LaunchConfiguration('spacing').perform(context))
    start_y = float(LaunchConfiguration('start_y').perform(context))
    y_spacing = float(LaunchConfiguration('y_spacing').perform(context))

    xs_csv = LaunchConfiguration('xs').perform(context).strip()
    ys_csv = LaunchConfiguration('ys').perform(context).strip()

    ids = [s.strip() for s in ids_csv.split(',') if s.strip()]
    xs = _parse_csv_floats(xs_csv) if xs_csv else []
    ys = _parse_csv_floats(ys_csv) if ys_csv else []

    if xs and len(xs) != len(ids):
        raise RuntimeError(f'xs count ({len(xs)}) must match ids count ({len(ids)})')
    if ys and len(ys) != len(ids):
        raise RuntimeError(f'ys count ({len(ys)}) must match ids count ({len(ids)})')

    for i, sid in enumerate(ids):
        uav = f'cf{sid}'
        file_path = os.path.join(models_dir, f'crazyflie_cf{sid}', 'model.sdf')
        x = xs[i] if xs else (start_x + i * spacing)
        y = ys[i] if ys else (start_y + i * y_spacing)
        actions.append(_spawn_from_file(uav, file_path, x, y, z))
        actions.append(_bridge_cmd_vel(uav, f'/{uav}/gazebo/command/twist'))
        actions.append(_bridge_odom(uav, f'/model/{uav}/odometry'))
    return actions



def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('models_dir', default_value='/home/ricardo/crazyflie_mapping_demo/ros2_ws/src/ros_gz_crazyflie/ros_gz_crazyflie_gazebo/models'),
        DeclareLaunchArgument('ids', default_value='1,2,3,4,5,6,7,8,9,10', description='Comma-separated Crazyflie IDs to spawn'),
        DeclareLaunchArgument('z', default_value='0.35'),
        DeclareLaunchArgument('start_x', default_value='-1.0'),
        DeclareLaunchArgument('spacing', default_value='1.5'),
        DeclareLaunchArgument('start_y', default_value='0.2'),
        DeclareLaunchArgument('y_spacing', default_value='0.0'),
        DeclareLaunchArgument('xs', default_value='', description='Optional comma-separated x positions, one per id'),
        DeclareLaunchArgument('ys', default_value='', description='Optional comma-separated y positions, one per id'),
        OpaqueFunction(function=_setup),
    ])
