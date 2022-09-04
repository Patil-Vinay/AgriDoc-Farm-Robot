import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description = get_package_share_directory('agridoc_description')
    sim = get_package_share_directory('agridoc_gazebo')
    navigation = get_package_share_directory('agridoc_navigation')
    gazebo_ros = get_package_share_directory('gazebo_ros')

    xacro_file = os.path.join(description, 'urdf', 'agridoc.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world, 'gui': gui}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    # robot starts at home in the south-west corner, facing along the rows
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'agridoc', '-topic', 'robot_description',
                   '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                   '-z', '0.02', '-Y', LaunchConfiguration('yaw')],
        output='screen',
    )

    # wheel odom + imu, same as on the real robot
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[os.path.join(navigation, 'config', 'ekf.yaml'), {'use_sim_time': True}],
    )

    gyro_bias = Node(
        package='agridoc_navigation',
        executable='gyro_bias',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=os.path.join(sim, 'worlds', 'tomato_field.world')),
        DeclareLaunchArgument('gui', default_value='true', description='Open the Gazebo window'),
        DeclareLaunchArgument('x', default_value='-0.9'),
        DeclareLaunchArgument('y', default_value='-2.1'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        gazebo,
        robot_state_publisher,
        spawn,
        gyro_bias,
        ekf,
    ])
