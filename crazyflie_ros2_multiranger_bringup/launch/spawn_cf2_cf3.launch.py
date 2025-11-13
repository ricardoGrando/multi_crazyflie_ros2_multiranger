# Save as: crazyflie_ros2_multiranger_bringup/launch/spawn_cf2_cf3.launch.py
#
# This launch file:
# - Spawns two Crazyflie variants (cf2, cf3) from SDF files.
# - Creates per-UAV ROS->Gazebo bridges so you can publish to /cf2/cmd_vel and /cf3/cmd_vel.
#
# Assumes Gazebo Sim is already running (e.g., via your bringup). If not, start it first.
#
# Usage (adjust model paths if needed):
#   ros2 launch crazyflie_ros2_multiranger_bringup spawn_cf2_cf3.launch.py \
#     cf2_model:=/home/ricardo/crazyflie_mapping_demo/ros2_ws/src/ros_gz_crazyflie/ros_gz_crazyflie_gazebo/models/crazyflie_cf2/model.sdf \
#     cf3_model:=/home/ricardo/crazyflie_mapping_demo/ros2_ws/src/ros_gz_crazyflie/ros_gz_crazyflie_gazebo/models/crazyflie_cf3/model.sdf
#
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _spawn_action(entity_name: str, file_lc, x_lc, y_lc, z_lc):
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_spawn_model.launch.py')
        ),
        launch_arguments={
            'file': file_lc,
            'entity_name': entity_name,
            'x': x_lc,
            'y': y_lc,
            'z': z_lc,
        }.items()
    )


def _cmd_vel_bridge(uav_name: str):
    gz_topic = f'/{uav_name}/gazebo/command/twist'
    # Bridge ROS -> Gazebo using gz.msgs.Twist (per your system logs)
    return Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[f'{gz_topic}@geometry_msgs/msg/Twist]gz.msgs.Twist'],
        remappings=[(gz_topic, f'/{uav_name}/cmd_vel')],
        output='screen',
        name=f'{uav_name}_cmd_vel_bridge',
    )


def generate_launch_description():
    cf2_model = DeclareLaunchArgument(
        'cf2_model',
        default_value='/home/ricardo/crazyflie_mapping_demo/ros2_ws/src/ros_gz_crazyflie/ros_gz_crazyflie_gazebo/models/crazyflie_cf2/model.sdf',
        description='Absolute path to cf2 model.sdf',
    )
    cf3_model = DeclareLaunchArgument(
        'cf3_model',
        default_value='/home/ricardo/crazyflie_mapping_demo/ros2_ws/src/ros_gz_crazyflie/ros_gz_crazyflie_gazebo/models/crazyflie_cf3/model.sdf',
        description='Absolute path to cf3 model.sdf',
    )

    # Spawn poses
    cf2_x = DeclareLaunchArgument('cf2_x', default_value='1.0')
    cf2_y = DeclareLaunchArgument('cf2_y', default_value='0.0')
    cf2_z = DeclareLaunchArgument('cf2_z', default_value='0.35')

    cf3_x = DeclareLaunchArgument('cf3_x', default_value='-1.0')
    cf3_y = DeclareLaunchArgument('cf3_y', default_value='0.0')
    cf3_z = DeclareLaunchArgument('cf3_z', default_value='0.35')

    # LaunchConfigurations
    cf2_model_lc = LaunchConfiguration('cf2_model')
    cf3_model_lc = LaunchConfiguration('cf3_model')

    return LaunchDescription([
        cf2_model, cf3_model,
        cf2_x, cf2_y, cf2_z, cf3_x, cf3_y, cf3_z,

        # Spawn cf2 then cf3
        _spawn_action('cf2', cf2_model_lc, LaunchConfiguration('cf2_x'),
                      LaunchConfiguration('cf2_y'), LaunchConfiguration('cf2_z')),
        _spawn_action('cf3', cf3_model_lc, LaunchConfiguration('cf3_x'),
                      LaunchConfiguration('cf3_y'), LaunchConfiguration('cf3_z')),

        # Per-UAV bridges to expose /cf2/cmd_vel and /cf3/cmd_vel
        _cmd_vel_bridge('cf2'),
        _cmd_vel_bridge('cf3'),
    ])