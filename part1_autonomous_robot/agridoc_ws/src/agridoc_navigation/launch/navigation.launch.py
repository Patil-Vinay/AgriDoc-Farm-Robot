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
    map_file = LaunchConfiguration('map')
    rviz = LaunchConfiguration('rviz')
    patrol = LaunchConfiguration('patrol')

    # map server, amcl and the nav2 servers
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={'map': map_file,
                          'use_sim_time': use_sim_time,
                          'params_file': os.path.join(pkg, 'config', 'nav2.yaml'),
                          'use_composition': 'False'}.items())

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg, 'rviz', 'navigation.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz))

    patrol_node = Node(
        package='agridoc_navigation',
        executable='patrol',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'route_file': LaunchConfiguration('route')}],
        condition=IfCondition(patrol))

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', default_value=os.path.join(pkg, 'maps', 'tomato_field.yaml')),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('patrol', default_value='false'),
        DeclareLaunchArgument('route', default_value=os.path.join(pkg, 'config', 'field_route.yaml')),
        bringup,
        rviz_node,
        patrol_node,
    ])
