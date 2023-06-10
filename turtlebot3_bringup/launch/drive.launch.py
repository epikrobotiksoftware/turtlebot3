#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    driver_goal_node = Node(
        package='turtlebot3_bringup',
        executable='action_server.py',
        name='driver_goal',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        driver_goal_node
    ])
