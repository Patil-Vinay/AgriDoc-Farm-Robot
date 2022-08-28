import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('agridoc_navigation')
    nav2_dir = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    nav = LaunchConfiguration('nav')
    rviz = LaunchConfiguration('rviz')

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[os.path.join(pkg, 'config', 'slam.yaml'),
                    {'use_sim_time': use_sim_time}])

    # with nav:=true goals can be clicked in rviz while the map is being made
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={'use_sim_time': use_sim_time,
                          'params_file': os.path.join(pkg, 'config', 'nav2.yaml')}.items(),
        condition=IfCondition(nav))

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg, 'rviz', 'navigation.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz))

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('nav', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        slam,
        navigation,
        rviz_node,
    ])
