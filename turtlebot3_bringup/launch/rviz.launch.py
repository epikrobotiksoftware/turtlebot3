#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rviz2_file_name = os.environ.get('RVIZ_FILE_NAME', 'turtlebot3.rviz')
    use_sim_time = os.environ.get('USE_SIM_TIME', True)

    rviz_config = os.path.join(
        get_package_share_directory('turtlebot3_bringup'), 'rviz',rviz2_file_name)

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{'use_sim_time':use_sim_time}]
    )
    
    return LaunchDescription([
        rviz
    ])
