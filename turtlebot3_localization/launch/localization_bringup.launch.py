
import os
import launch
from launch.actions import IncludeLaunchDescription
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import yaml


def generate_launch_description():

    mapping_pkg_dir = get_package_share_directory("turtlebot3_mapping")
    localization_pkg_dir = get_package_share_directory(
        "turtlebot3_localization")

    map_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            mapping_pkg_dir + '/launch/map_server.launch.py'))

    amcl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            localization_pkg_dir + '/launch/amcl.launch.py'))

    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            localization_pkg_dir + '/launch/ekf.launch.py'))

    lifecycle_nodes = ['map_server', 'amcl']
    autostart = True

    start_lifecycle_manager_cmd = launch_ros.actions.Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes}])

    return launch.LaunchDescription([
        ekf,
        amcl,
        map_server,
        start_lifecycle_manager_cmd,
    ])
