# Save as: crazyflie_ros2_multiranger_bringup/launch/multi_cf_sim_ign.launch.py
#
# Purpose:
# - Start the base Crazyflie sim (world + GUI + default 'crazyflie').
# - Spawn extra models (cf2, cf3, ...).
# - For *extra* UAVs, create a ROS->Gazebo bridge:
#     ROS  '/<name>/cmd_vel'  -->  GZ '/<name>/gazebo/command/twist'
#   using *ignition.msgs.Twist* (matches your working default bridge).
#
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _parse_uavs(spec: str):
    out = []
    if not spec:
        return out
    for item in spec.split(';'):
        item = item.strip()
        if not item:
            continue
        name, pose = item.split(':', 1)
        x, y, z = (float(v) for v in pose.split(','))
        out.append((name.strip(), (x, y, z)))
    return out


def _spawn_model(entity_name: str, x: float, y: float, z: float):
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_spawn_model.launch.py')
        ),
        launch_arguments={
            'file': 'model://crazyflie',
            'entity_name': entity_name,
            'x': str(x), 'y': str(y), 'z': str(z),
        }.items()
    )


def _mk_cmd_vel_bridge_ign(uav_name: str):
    """
    One-way bridge (ROS -> Gazebo) using ignition.msgs.Twist.
    Expose clean ROS topic '/<uav>/cmd_vel' for publishing.
    """
    gz_topic = f'/{uav_name}/gazebo/command/twist'
    return Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        # ']' = ROS -> GZ
        arguments=[f'{gz_topic}@geometry_msgs/msg/Twist]ignition.msgs.Twist'],
        # remap ROS side to '/<uav>/cmd_vel'
        remappings=[(gz_topic, f'/{uav_name}/cmd_vel')],
        output='screen',
        name=f'{uav_name}_cmd_vel_bridge',
    )


def _setup(context, *args, **kwargs):
    actions = []

    # 1) Base sim (includes default 'crazyflie' and its own cmd_vel bridge)
    bringup_share = get_package_share_directory('ros_gz_crazyflie_bringup')
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'crazyflie_simulation.launch.py')
        )
    ))

    # 2) Spawn extra UAVs + bridges
    uavs = _parse_uavs(LaunchConfiguration('uavs').perform(context))
    for name, (x, y, z) in uavs:
        actions.append(_spawn_model(name, x, y, z))
        actions.append(_mk_cmd_vel_bridge_ign(name))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'uavs',
            default_value='cf2:1.0,0.0,0.35; cf3:-1.0,0.0,0.35',
            description="Semicolon list '<name>:x,y,z' of extra drones to spawn."
        ),
        OpaqueFunction(function=_setup),
    ])
