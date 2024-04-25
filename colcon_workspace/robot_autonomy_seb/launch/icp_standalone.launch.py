import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription([
        # start simulation
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robot_autonomy_seb'), 'launch',
                         'sim_with_custom_configs.launch.py'))
        ),
        # start odom based on map
        Node(package="robot_autonomy_seb",
             executable="icp")
    ])
    return ld
