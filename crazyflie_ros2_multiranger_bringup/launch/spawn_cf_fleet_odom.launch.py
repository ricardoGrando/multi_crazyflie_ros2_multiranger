# Save as: crazyflie_ros2_multiranger_bringup/launch/spawn_cf_fleet_odom.launch.py
#
# Spawns a fleet cf1..cfN and sets up:
#   - /cfX/cmd_vel  ->  /cfX/gazebo/command/twist       (ROS -> Gazebo)
#   - /cfX/odom     <-  /model/cfX/odometry             (Gazebo -> ROS)
#
# Assumes: Gazebo Sim already running (your base bringup) with the default 'crazyflie' model.
# For cf1 we **do not** spawn a new model; we alias:
#   /cf1/cmd_vel  ->  /crazyflie/gazebo/command/twist
#   /cf1/odom     <-  /model/crazyflie/odometry
#
# Example:
#   ros2 launch crazyflie_ros2_multiranger_bringup spawn_cf_fleet_odom.launch.py \
#     count:=5 \
#     models_dir:=/home/ricardo/crazyflie_mapping_demo/ros2_ws/src/ros_gz_crazyflie/ros_gz_crazyflie_gazebo/models \
#     start_x:=-1.5 spacing:=1.5 z:=0.35
#
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

def _bridge_cmd_vel(uav_name: str, gz_topic: str):
    # ROS -> Gazebo (use gz.msgs.Twist)
    return Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[f'{gz_topic}@geometry_msgs/msg/Twist]gz.msgs.Twist'],
        remappings=[(gz_topic, f'/{uav_name}/cmd_vel')],
        output='screen',
        name=f'{uav_name}_cmd_vel_bridge',
    )

def _bridge_odom(uav_name: str, gz_topic: str):
    # Gazebo -> ROS (use gz.msgs.Odometry)
    # Direction token '[' means GZ -> ROS
    return Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[f'{gz_topic}@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
        remappings=[(gz_topic, f'/{uav_name}/odom')],
        output='screen',
        name=f'{uav_name}_odom_bridge',
    )

def _setup(context, *args, **kwargs):
    actions = []
    N = int(LaunchConfiguration('count').perform(context))
    models_dir = LaunchConfiguration('models_dir').perform(context)
    start_x = float(LaunchConfiguration('start_x').perform(context))
    spacing = float(LaunchConfiguration('spacing').perform(context))
    z = float(LaunchConfiguration('z').perform(context))

    # cf1 bridges (alias to default model)
    if N >= 1:
        actions.append(_bridge_cmd_vel('cf1', '/crazyflie/gazebo/command/twist'))
        actions.append(_bridge_odom('cf1', '/model/crazyflie/odometry'))

    # cf2..cfN spawn + bridges
    for i in range(2, N+1):
        uav = f'cf{i}'
        file_path = os.path.join(models_dir, f'crazyflie_{uav}', 'model.sdf')
        x = start_x + (i-1) * spacing
        actions.append(_spawn_from_file(uav, file_path, x, 0.0, z))
        actions.append(_bridge_cmd_vel(uav, f'/{uav}/gazebo/command/twist'))
        actions.append(_bridge_odom(uav, f'/model/{uav}/odometry'))

    return actions

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('count', default_value='3', description='How many vehicles to control (1..10)'),
        DeclareLaunchArgument('models_dir', default_value='/home/ricardo/crazyflie_mapping_demo/ros2_ws/src/ros_gz_crazyflie/ros_gz_crazyflie_gazebo/models',
                              description='Directory with crazyflie_cfX folders (model.sdf & model.config)'),
        DeclareLaunchArgument('start_x', default_value='-1.5', description='Initial X for cf1; others spaced by "spacing"'),
        DeclareLaunchArgument('spacing', default_value='1.5', description='X spacing between consecutive UAVs'),
        DeclareLaunchArgument('z', default_value='0.35', description='Spawn altitude'),
        OpaqueFunction(function=_setup),
    ])