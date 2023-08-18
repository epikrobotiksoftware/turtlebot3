#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    save_last_position_node = Node(
        package='turtlebot3_localization',
        executable='save_last_pose.py',
        name='save_last_pose',
        output='screen',
    )

    publish_robot_pose_node = Node(
        package='turtlebot3_localization',
        executable='publish_robot_pose.py',
        name='publish_robot_pose',
        output='screen',
    )

    return LaunchDescription([
        save_last_position_node,
        publish_robot_pose_node
    ])
