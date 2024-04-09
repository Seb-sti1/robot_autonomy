import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    my_package = get_package_share_directory('robot_autonomy_seb')

    my_turtlebot = get_package_share_directory('my_turtlebot')
    my_turtlebot_launch_dir = os.path.join(my_turtlebot, 'launch')

    nav2_params = os.path.join(my_package, 'params', 'nav2_params.yml')

    declare_rviz_filename = DeclareLaunchArgument(
        'rviz_filename',
        default_value='icp.rviz',
        description='The name of the rviz config file to use.')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=[os.path.join(my_package,
                                    'rviz',
                                    ''), LaunchConfiguration('rviz_filename')],
        description='Full path to the RVIZ config file to use'
    )

    ld = LaunchDescription()
    ld.add_action(declare_rviz_filename)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(my_turtlebot_launch_dir, 'turtlebot_simulation.launch.py')),
        launch_arguments={'params_file': nav2_params,
                          'rviz_config_file': LaunchConfiguration('rviz_config_file')}.items()
    ))
    return ld
