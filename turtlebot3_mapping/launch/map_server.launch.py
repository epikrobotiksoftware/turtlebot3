#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = os.environ.get('USE_SIM_TIME', True)
    map_yaml_file = os.path.join(get_package_share_directory(
        'turtlebot3_mapping'), 'maps/new_map.yaml')
    
    print(map_yaml_file)

    map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml_file,
                    'use_sim_time': use_sim_time}]
    )

    lifecycle_nodes = ['map_server']

    autostart = True

    start_lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes}])

    return LaunchDescription([
        map_server_cmd,
        start_lifecycle_manager_cmd
    ])
