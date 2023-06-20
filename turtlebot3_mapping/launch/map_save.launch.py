#!/usr/bin/env python3

from launch_ros.actions import Node
from launch import LaunchDescription
import getpass

def generate_launch_description():
    WS = "/home/"+getpass.getuser()+"/humble_ws"
    package_name = "/turtlebot3_mapping_demo/turtlebot3_mapping"
    map_name = "new_map"
    new_map_path = WS+"/src" + package_name + "/maps/"+map_name
    # Create the map_saver_cli node
    map_saver_cli_node = Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        output='screen',
        arguments=['-f', new_map_path, '--ros-args',
                    '-p', 'free_thresh_default:=0.196']
    )

    return LaunchDescription([
        map_saver_cli_node
    ])
