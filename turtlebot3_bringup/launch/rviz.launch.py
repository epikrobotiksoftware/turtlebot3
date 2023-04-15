#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import getpass


def generate_launch_description():
    rviz2_file_name = os.environ.get('RVIZ_FILE_NAME', 'turtlebot3.rviz')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')


    WS = "/home/"+getpass.getuser()+"/Documents/ros2_ws"
    package_name = "/turtlebot3/turtlebot3_bringup"
    rviz_config = WS+"/src" + package_name + "/rviz/"+rviz2_file_name

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config]
    )
    
    return LaunchDescription([
        rviz
    ])
