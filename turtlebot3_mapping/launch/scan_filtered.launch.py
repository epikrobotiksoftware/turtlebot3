#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    scan_filtered_node = Node(
        package='turtlebot3_mapping',
        executable='scan_filtered.py',
        name='scan_filtered',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        scan_filtered_node
    ])
