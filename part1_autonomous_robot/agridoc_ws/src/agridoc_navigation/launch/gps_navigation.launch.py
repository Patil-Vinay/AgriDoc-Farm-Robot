import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def navsat_node(context):
    pkg = get_package_share_directory('agridoc_navigation')
    with open(LaunchConfiguration('route').perform(context)) as f:
        lat, lon, heading = yaml.safe_load(f)['home']
    # navsat_transform wants the yaw counterclockwise from east, in radians
    datum = [float(lat), float(lon), math.radians(90.0 - float(heading))]

    return [Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        parameters=[os.path.join(pkg, 'config', 'ekf.yaml'),
                    {'use_sim_time': LaunchConfiguration('use_sim_time'), 'datum': datum}],
        remappings=[('odometry/filtered', 'odometry/global')])]


# GPS localization and Nav2 without a saved map, for the field outdoors.
# Needs the wheel odometry EKF that sim.launch.py and robot.launch.py start
def generate_launch_description():
    pkg = get_package_share_directory('agridoc_navigation')
    nav2_dir = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')

    ekf_map = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_map_node',
        parameters=[os.path.join(pkg, 'config', 'ekf.yaml'), {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', 'odometry/global')])

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={'use_sim_time': use_sim_time,
                          'params_file': os.path.join(pkg, 'config', 'nav2_gps.yaml')}.items())

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg, 'rviz', 'gps_navigation.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz')))

    patrol_node = Node(
        package='agridoc_navigation',
        executable='gps_patrol',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'route_file': LaunchConfiguration('route')}],
        condition=IfCondition(LaunchConfiguration('patrol')))

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('patrol', default_value='false'),
        DeclareLaunchArgument('route', default_value=os.path.join(pkg, 'config', 'gps_route.yaml')),
        ekf_map,
        OpaqueFunction(function=navsat_node),
        navigation,
        rviz_node,
        patrol_node,
    ])
