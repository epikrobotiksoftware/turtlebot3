#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    params_file_dir = os.path.join(
        get_package_share_directory('turtlebot3_mapping'), 'config', 'mapper_params_online_async.yaml')

    launch_file_dir = os.path.join(
        get_package_share_directory('slam_toolbox'), 'launch')

    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [launch_file_dir, '/online_async_launch.py']),
        launch_arguments={'use_sim_time': 'true',
                          'params_file': params_file_dir}.items(),
    )

    return LaunchDescription([
        mapping_launch,
    ])
