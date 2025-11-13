# Save as: crazyflie_ros2_multiranger_bringup/launch/multi_crazyflie_simulation.launch.py
#
# Features:
# - Start the base Crazyflie sim once (world + GUI + default CF).
# - Spawn extra Crazyflies (cf2, cf3, ...) using ros_gz_sim's spawner.
# - Optional per-UAV simple_mapper in its namespace.
# - Per-UAV cmd_vel bridge (ROS -> Gazebo) to '/<uav>/gazebo/command/twist',
#   exposed on ROS as '/<uav>/cmd_vel' so you can drive each drone independently.
#
# Usage example:
#   ros2 launch crazyflie_ros2_multiranger_bringup multi_crazyflie_simulation.launch.py \
#       uavs:="cf2:1.0,0.0,0.35; cf3:-1.0,0.0,0.35" start_mappers:=true
#
# Notes:
# - We DO NOT add an extra bridge for the default 'crazyflie' because the original
#   bringup already provides one. Extra UAVs get their own bridges here.
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
    """
    Parse a UAV spec string like:
      'cf2:1.0,0.0,0.35; cf3:-1.0,0.0,0.35'
    -> [('cf2',(1.0,0.0,0.35)), ('cf3',(-1.0,0.0,0.35))]
    """
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
    """Launch one mapper in a namespace, with robot_prefix=<ns> and sim time."""
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
    """
    Spawn a Crazyflie model into the running Gazebo Sim world.
    Requires GZ_SIM_RESOURCE_PATH to include the Crazyflie model (as in the tutorial).
    """
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_spawn_model.launch.py')
        ),
        launch_arguments={
            'file': 'model://crazyflie',   # resolved via GZ_SIM_RESOURCE_PATH
            'entity_name': entity_name,
            'x': str(x),
            'y': str(y),
            'z': str(z),
        }.items()
    )


def _mk_cmd_vel_bridge_plain(uav_name: str):
    """
    Bridge ROS -> Gazebo for velocity commands of one UAV using the Bitcraze
    controller's expected topic: '/<uav_name>/gazebo/command/twist'.
    We expose a clean ROS topic: '/<uav_name>/cmd_vel'.
    """
    gz_topic = f'/{uav_name}/gazebo/command/twist'
    return Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        # ']' makes it one-way: ROS -> Gazebo
        arguments=[f'{gz_topic}@geometry_msgs/msg/Twist]gz.msgs.Twist'],
        # expose a clean ROS topic: /<uav_name>/cmd_vel
        remappings=[(gz_topic, f'/{uav_name}/cmd_vel')],
        output='screen',
        name=f'{uav_name}_cmd_vel_bridge',
    )


def _setup(context, *args, **kwargs):
    actions = []

    # --- Launch the standard single-UAV simulation once ---
    bringup_share = get_package_share_directory('ros_gz_crazyflie_bringup')
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, 'launch', 'crazyflie_simulation.launch.py')
            )
        )
    )

    # --- RViz with the same config as your original file ---
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

    # Read args
    start_mappers = LaunchConfiguration('start_mappers').perform(context).lower() in ('1', 'true', 'yes')
    uavs_spec = LaunchConfiguration('uavs').perform(context)

    # --- Spawn extra UAVs + per-UAV mapper + per-UAV cmd_vel bridge ---
    for name, (x, y, z) in _parse_uavs(uavs_spec):
        actions.append(_mk_spawn_action(name, x, y, z))
        if start_mappers:
            actions.append(_mk_mapper(name))
        actions.append(_mk_cmd_vel_bridge_plain(name))

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
