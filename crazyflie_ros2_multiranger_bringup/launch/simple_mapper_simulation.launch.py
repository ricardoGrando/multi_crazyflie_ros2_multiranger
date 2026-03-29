import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    pkg_project_crazyflie_gazebo = get_package_share_directory(
        'ros_gz_crazyflie_bringup'
    )
    pkg_bringup = get_package_share_directory(
        'crazyflie_ros2_multiranger_bringup'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_mapper = LaunchConfiguration('enable_mapper')
    robot_prefix = LaunchConfiguration('robot_prefix')
    rviz_config = LaunchConfiguration('rviz_config')

    crazyflie_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_project_crazyflie_gazebo,
                'launch',
                'crazyflie_simulation.launch.py'
            )
        )
    )

    simple_mapper = Node(
        package='crazyflie_ros2_multiranger_simple_mapper',
        executable='simple_mapper_multiranger',
        name='simple_mapper',
        output='screen',
        condition=IfCondition(enable_mapper),
        parameters=[
            {'robot_prefix': robot_prefix},
            {'use_sim_time': use_sim_time}
        ]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true'
        ),
        DeclareLaunchArgument(
            'enable_mapper',
            default_value='false'
        ),
        DeclareLaunchArgument(
            'robot_prefix',
            default_value='cf1'
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(
                pkg_bringup,
                'config',
                'sim_mapping_video.rviz'
            )
        ),

        crazyflie_simulation,
        simple_mapper,
        rviz,
    ])
