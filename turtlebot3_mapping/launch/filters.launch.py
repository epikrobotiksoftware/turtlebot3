#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    mapping_pkg_dir = get_package_share_directory("turtlebot3_mapping")

    keepout_zone = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            mapping_pkg_dir + '/launch/keepout_zone.launch.py'))

    speed_limit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            mapping_pkg_dir + '/launch/speedlimit_zone.launch.py'))
    
    preferred_lanes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            mapping_pkg_dir + '/launch/preferred_lanes.launch.py'))
    
    return LaunchDescription([
        keepout_zone,
        speed_limit,
        preferred_lanes        
    ])
