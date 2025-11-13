# Save as: crazyflie_ros2_multiranger_bringup/launch/multi_cf_sim_bidirectional.launch.py
#
# Purpose:
# - Start the base Crazyflie sim (world + GUI + default 'crazyflie').
# - Spawn extra models (cf2, cf3, ...).
# - For *every* UAV (including the default), create a *bidirectional* bridge
#   between ROS '/<name>/cmd_vel' and Gazebo '/<name>/gazebo/command/twist'.
#
# Why bidirectional? To avoid any direction mismatch issues. It still allows you
# to publish from ROS to '/<name>/cmd_vel' normally.
#
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def _parse_uavs(spec: str):
    out = []
    if not spec:
        return out
    for item in spec.split(';'):
        item = item.strip()
        if not item:
            continue
        name, pose = item.split(':', 1)
        xyz = tuple(float(v.strip()) for v in pose.split(','))
        if len(xyz) != 3:
            raise RuntimeError("Each UAV pose must be 'x,y,z'")
        out.append((name.strip(), xyz))
    return out


def _mk_mapper(ns: str):
    return GroupAction([
        PushRosNamespace(ns),
        Node(
            package='crazyflie_ros2_multiranger_simple_mapper',
            executable='simple_mapper_multiranger',
            name='simple_mapper',
            output='screen',
            parameters=[
                {'robot_prefix': ns},
                {'use_sim_time': True},
            ],
        ),
    ])


def _mk_spawn_action(entity_name: str, x: float, y: float, z: float):
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_spawn_model.launch.py')
        ),
        launch_arguments={
            'file': 'model://crazyflie',
            'entity_name': entity_name,
            'x': str(x),
            'y': str(y),
            'z': str(z),
        }.items()
    )


def _mk_cmd_vel_bridge_bidirectional(uav_name: str):
    """
    Use bidirectional mapping to ensure compatibility with your sim version.
    Gazebo topic:  '/<uav>/gazebo/command/twist'
    ROS   topic:   '/<uav>/cmd_vel'
    """
    gz_topic = f'/{uav_name}/gazebo/command/twist'
    return Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        # '@' == bidirectional. Types: geometry_msgs/Twist <-> gz.msgs.Twist
        arguments=[f'{gz_topic}@geometry_msgs/msg/Twist@gz.msgs.Twist'],
        # Remap ROS side to '/<uav>/cmd_vel' for clean publishing:
        remappings=[(gz_topic, f'/{uav_name}/cmd_vel')],
        output='screen',
        name=f'{uav_name}_cmd_vel_bridge',
    )


def _setup(context, *args, **kwargs):
    actions = []

    # 1) Base sim (spawns 'crazyflie' by default)
    bringup_share = get_package_share_directory('ros_gz_crazyflie_bringup')
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, 'launch', 'crazyflie_simulation.launch.py')
            )
        )
    )

    # 2) RViz (optional, same as your original)
    rviz_config_path = os.path.join(
        get_package_share_directory('crazyflie_ros2_multiranger_bringup'),
        'config',
        'sim_mapping.rviz'
    )
    actions.append(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            parameters=[{"use_sim_time": True}],
            output='screen'
        )
    )

    # 3) Extra UAVs + per-UAV bridges (and optional mappers)
    start_mappers = LaunchConfiguration('start_mappers').perform(context).lower() in ('1', 'true', 'yes')
    uavs_spec = LaunchConfiguration('uavs').perform(context)
    uavs = _parse_uavs(uavs_spec)

    # Bridge also for the default 'crazyflie' to make behavior uniform
    actions.append(_mk_cmd_vel_bridge_bidirectional('crazyflie'))
    if start_mappers:
        actions.append(_mk_mapper('crazyflie'))

    for name, (x, y, z) in uavs:
        actions.append(_mk_spawn_action(name, x, y, z))
        actions.append(_mk_cmd_vel_bridge_bidirectional(name))
        if start_mappers:
            actions.append(_mk_mapper(name))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'uavs',
            default_value='cf2:1.0,0.0,0.35; cf3:-1.0,0.0,0.35',
            description="Semicolon-separated list of '<name>:x,y,z' for extra UAVs."
        ),
        DeclareLaunchArgument(
            'start_mappers',
            default_value='false',
            description='true/false: start a simple_mapper for each UAV, namespaced.'
        ),
        OpaqueFunction(function=_setup),
    ])
