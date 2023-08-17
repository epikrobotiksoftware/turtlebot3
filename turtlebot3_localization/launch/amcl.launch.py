
import os
import launch
from launch.actions import IncludeLaunchDescription
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import yaml


def generate_launch_description():

    pkg_share = launch_ros.substitutions.FindPackageShare(
        package='turtlebot3_localization').find('turtlebot3_localization')

    save_last_position_node = Node(
        package='turtlebot3_localization',
        executable='save_last_pose.py',
        name='save_last_pose',
        output='screen',
    )

    WS = os.getenv("WS")
    META_PACKAGE_NAME = os.getenv("META_PACKAGE_NAME")
    ROBOT_NAME = os.getenv("robot_name")
    path = f"{WS}/{META_PACKAGE_NAME}/{ROBOT_NAME}_localization/config/save_last_pose.yaml"
    with open(path, 'r') as f:
        pose_data = yaml.safe_load(f)
        pose_x_value = pose_data['initial_pose_x']
        pose_y_value = pose_data['initial_pose_y']
        pose_z_value = pose_data['initial_pose_z']
        pose_yaw_value = pose_data['initial_pose_yaw']

    extra_params = {
        "initial_pose": {
            "x": pose_x_value,
            "y": pose_y_value,
            "z": pose_z_value,
            "yaw": 0.0
        }
    }

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'localization.yaml'), {
            'use_sim_time': True}],
        # remappings=remappings
    )


    return launch.LaunchDescription([
        save_last_position_node,
        amcl,
    ])
