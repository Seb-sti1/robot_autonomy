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
                         'sim_with_custom_configs.launch.py')),
            launch_arguments={'rviz_filename': 'nbv.rviz',
                              'slam': 'True'}.items()
        ),
        # start mapping node
        Node(package="robot_autonomy_seb",
             executable="nbv")
    ])
    return ld
