#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    ekf_config_file = os.path.join(get_package_share_directory('turtlebot3_localization'),
                                   'config', "ekf.yaml")

    robot_pose_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(
                'turtlebot3_localization'), 'launch',
                'robot_pose.launch.py'),
        )
    )

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
       parameters=[{'use_sim_time': use_sim_time},ekf_config_file],
                                 )


    return LaunchDescription([

        robot_localization_node,
        robot_pose_launch,
    ])
