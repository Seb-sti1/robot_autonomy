import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')
    bringup_launch_dir = os.path.join(bringup_dir, 'launch')
    my_turtlebot_dir = get_package_share_directory('my_turtlebot')
    robot_autonomy_dir = get_package_share_directory('robot_autonomy_seb')

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    pose = {'x': LaunchConfiguration('x_pose', default='0.00'),
            'y': LaunchConfiguration('y_pose', default='0.00'),
            'z': LaunchConfiguration('z_pose', default='0.01'),
            'R': LaunchConfiguration('roll', default='0.00'),
            'P': LaunchConfiguration('pitch', default='0.00'),
            'Y': LaunchConfiguration('yaw', default='0.00')}
    namespace = 'turtlebot'

    rviz_nonamespace_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_launch_dir, 'rviz_launch.py')),
        launch_arguments={'rviz_config': os.path.join(robot_autonomy_dir,
                                                      'rviz',
                                                      'full.rviz')}.items())

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_launch_dir, 'rviz_launch.py')),
        launch_arguments={'namespace': namespace,
                          'use_namespace': 'True',
                          'rviz_config': os.path.join(robot_autonomy_dir,
                                                      'rviz',
                                                      'full.rviz')}.items())
    # Specify the actions
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', my_turtlebot_dir + '/worlds/lab_world.world'],
        cwd=[bringup_launch_dir], output='screen')

    start_gazebo_client_cmd = ExecuteProcess(
        cmd=['gzclient'],
        cwd=[bringup_launch_dir], output='screen')

    urdf = os.path.join(get_package_share_directory('turtlebot3_description'), 'urdf', 'turtlebot3_burger.urdf')
    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    start_robot_state_publisher_cmd = Node(
        # condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': 'true',
                     'robot_description': robot_description}],
        remappings=remappings
    )

    start_gazebo_spawner_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_autonomy_dir,
                         'launch',
                         'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': pose['x'],
            'y_pose': pose['y'],
            'namespace': namespace
        }.items()
    )

    ld = LaunchDescription([
        start_gazebo_server_cmd, start_gazebo_client_cmd,
        start_robot_state_publisher_cmd,
        start_gazebo_spawner_cmd,
        rviz_cmd, rviz_nonamespace_cmd
    ])
    return ld
