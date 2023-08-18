#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import yaml


def generate_launch_description():

    use_sim_time = os.environ.get('USE_SIM_TIME', True)
    world_file_name = 'my_world.model'
    world = os.path.join(get_package_share_directory(
        'turtlebot3_gazebo'), 'worlds', world_file_name)

    last_pose_yaml = os.path.join(get_package_share_directory(
        'turtlebot3_localization'), 'config/save_last_pose.yaml')

    with open(last_pose_yaml, 'r') as file:
        data = yaml.safe_load(file)

    initial_pose_x =  data['initial_pose_x']
    initial_pose_y =  data['initial_pose_y']
    initial_pose_yaw = data['initial_pose_yaw']

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

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'turtlebot3',
                   '-topic', "/robot_description",
                   '-x', str(initial_pose_x), '-y', str(initial_pose_y), '-z', '0.0',
                   '-Y', '0.0', '-P', '0.0', '-R', '0.0',
                   ],
        output='screen'
    )

    return LaunchDescription([
        gzserver,
        gzclient,
        spawn_entity
    ])
