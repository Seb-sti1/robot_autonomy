import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    my_package = get_package_share_directory('robot_autonomy_seb')

    my_turtlebot = get_package_share_directory('my_turtlebot')
    my_turtlebot_launch_dir = os.path.join(my_turtlebot, 'launch')

    nav2_params = os.path.join(my_package, 'params', 'nav2_params.yml')

    ld = LaunchDescription()

    # start turtlebot simulation
    ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(my_turtlebot_launch_dir, 'turtlebot_simulation.launch.py')),
        launch_arguments={'params_file': nav2_params,
                          'rviz_config_file': os.path.join(my_package,
                                                           'rviz',
                                                           'icp.rviz')}.items()
    ))
    return ld
