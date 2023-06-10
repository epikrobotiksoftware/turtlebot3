#!/usr/bin/env python3

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    #############################################################
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    urdf_file_name = 'turtlebot3.urdf'
    topic_name = LaunchConfiguration('topic_name', default='/cmd_vel')
    print('urdf_file_name : {}'.format(urdf_file_name))

    urdf = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf',
        urdf_file_name)
    robot_description_config = xacro.process_file(urdf)
    robot_desc = robot_description_config.toxml()
    #############################################################

    robot_state_publisher = Node(package='robot_state_publisher',
                                 executable='robot_state_publisher',
                                 parameters=[{'use_sim_time': use_sim_time,
                                              'robot_description': robot_desc}])

    joint_state_publisher = Node(package='joint_state_publisher',
                                 executable='joint_state_publisher',
                                 name='joint_state_publisher',
                                 parameters=[
                                     {'use_sim_time': use_sim_time}],
                                 )
    rqt_robot_steering = Node(package='rqt_robot_steering',
                              executable='rqt_robot_steering',
                              name='rqt_robot_steering',
                              output='screen',
                              parameters=[{'topic': topic_name}])

    return LaunchDescription([
        DeclareLaunchArgument(name='model', default_value=robot_desc,
                              description='Absolute path to robot urdf file'),
        DeclareLaunchArgument(
            'topic_name',
            default_value='/cmd_vel',
            description='Name of the topic to publish velocity commands'
        ),
        joint_state_publisher,
        robot_state_publisher,
        # rqt_robot_steering
    ])
    #############################################################
