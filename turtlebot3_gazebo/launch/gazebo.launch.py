#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    #############################################################
    use_sim_time = os.environ.get('USE_SIM_TIME', True)
    world_file_name = 'my_world.model'
    world = os.path.join(get_package_share_directory(
        'turtlebot3_gazebo'), 'worlds', world_file_name)

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
    )

    return LaunchDescription([
        gzserver,
        gzclient,
    ])
