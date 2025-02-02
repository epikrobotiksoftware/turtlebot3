#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    pkg_dir = get_package_share_directory('turtlebot3_bringup')
    mapping_pkg_dir = get_package_share_directory("turtlebot3_mapping")
    turtlebot3_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(
                'turtlebot3_description'), 'launch',
                'turtlebot3_description.launch.py'),
        ))
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(
                'turtlebot3_gazebo'), 'launch',
                'gazebo.launch.py'),
        )
    )
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            pkg_dir + '/launch/rviz.launch.py'))

    robot_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(
                'turtlebot3_localization'), 'launch',
                'localization_bringup.launch.py'),
        )
    )

    robot_mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(
                'turtlebot3_mapping'), 'launch',
                'mapping.launch.py'),
        )
    )

    robot_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(
                'turtlebot3_navigation'), 'launch',
                'navigation.launch.py'),
        )
    )
    filters = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            mapping_pkg_dir + '/launch/filters.launch.py'))
    
    TwistMux_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(
                'turtlebot3_navigation'), 'launch',
                'twist_mux.launch.py'),
        ))
    

    save_last_pose_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(
                'turtlebot3_localization'), 'launch',
                'save_last_pose.launch.py'),
        )
    )

    return LaunchDescription([
        
        robot_navigation_launch,
        turtlebot3_description_launch,
        gazebo_launch,
        rviz_launch,
        robot_localization_launch,
        robot_mapping_launch,
        TwistMux_launch,
        save_last_pose_launch
        #filters
    ])
    #############################################################
