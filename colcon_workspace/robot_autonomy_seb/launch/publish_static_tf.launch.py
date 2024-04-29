from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    map_odom = Node(package="tf2_ros",
                    executable="static_transform_publisher",
                    arguments=["0", "0", "0", "0", "0", "0", "map", "odom"])

    wheel_left_link = Node(package="tf2_ros",
                           executable="static_transform_publisher",
                           arguments=["0", "0.08", "0.023", "0", "0", "0", "base_link", "wheel_left_link"])

    wheel_right_link = Node(package="tf2_ros",
                            executable="static_transform_publisher",
                            arguments=["0", "-0.08", "0.023", "0", "0", "0", "base_link", "wheel_right_link"])

    ld = LaunchDescription()
    ld.add_action(map_odom)
    ld.add_action(wheel_left_link)
    ld.add_action(wheel_right_link)
    return ld
