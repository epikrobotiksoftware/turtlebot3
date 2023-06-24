
import os
import launch
from launch.actions import IncludeLaunchDescription
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    pkg_share = launch_ros.substitutions.FindPackageShare(package='turtlebot3_localization').find('turtlebot3_localization')
    lifecycle_nodes = ['amcl']
    autostart = True
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
    
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'localization.yaml'),{'use_sim_time' : True}],# {'use_sim_time' : use_sim_time}
        # remappings=remappings
    )
    
    start_lifecycle_manager_cmd = launch_ros.actions.Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes}])
    
    return launch.LaunchDescription([
        amcl,
        start_lifecycle_manager_cmd,
    ])