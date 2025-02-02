#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    autostart = False

    params_file_dir = os.path.join(
        get_package_share_directory('turtlebot3_mapping'), 'config', 'mapper_params_online_async.yaml')

    launch_file_dir = os.path.join(
        get_package_share_directory('slam_toolbox'), 'launch')

    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [launch_file_dir, '/online_async_launch.py']),
        launch_arguments={'use_sim_time': 'true',
                          'params_file': params_file_dir,
                          'autostart': 'false'}.items(),
    )

    live_map_node = Node(
        package='turtlebot3_mapping',
        executable='liveMap.py',
        name='liveMap',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},{'autostart': autostart}],
    )

    lifecycle_nodes = ['liveMap']

    autostart = False

    start_lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_mapping',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'bond_timeout': 0.0},
                    {'node_names': lifecycle_nodes}])

    return LaunchDescription([
        mapping_launch,
        live_map_node,
        start_lifecycle_manager_cmd
    ])
