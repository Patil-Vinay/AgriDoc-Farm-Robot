import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# the GPS route for the real field and the trained models live outside the workspace
ROUTE_DIR = os.path.expanduser('~/agridoc_route')
MODEL_DIR = os.path.expanduser('~/agridoc_models')


# Starts the real robot on the Jetson.
#   mode:=teleop   only the drivers, drive it from the laptop (and record the route)
#   mode:=patrol   drivers, GPS localization, Nav2 and the field patrol (default)
# Outdoors the lidar can't localize the robot, it only marks obstacles for Nav2.
# RViz runs on the laptop
def generate_launch_description():
    description = get_package_share_directory('agridoc_description')
    navigation = get_package_share_directory('agridoc_navigation')

    detection = LaunchConfiguration('detection')

    xacro_file = os.path.join(description, 'urdf', 'agridoc.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    # wheels, encoders, BNO055, GPS and pump through the Arduino Mega
    mega = Node(
        package='agridoc_bringup',
        executable='mega_bridge',
        output='screen',
        parameters=[{'port': LaunchConfiguration('mega_port')}],
        # comes back after the USB cable is plugged in again
        respawn=True,
        respawn_delay=2.0,
    )

    gyro_bias = Node(
        package='agridoc_navigation',
        executable='gyro_bias',
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[os.path.join(navigation, 'config', 'ekf.yaml')],
    )

    lidar = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        parameters=[{'serial_port': LaunchConfiguration('lidar_port'),
                     'serial_baudrate': 115200,
                     'frame_id': 'lidar_link',
                     'angle_compensate': True,
                     # turns the scan around if the lidar faces backwards in RViz
                     'flip_x_axis': LaunchConfiguration('lidar_flip')}],
    )

    patrol = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(navigation, 'launch', 'gps_navigation.launch.py')),
        launch_arguments={'use_sim_time': 'false',
                          'rviz': 'false',
                          'patrol': 'true',
                          'route': LaunchConfiguration('route')}.items(),
        condition=LaunchConfigurationEquals('mode', 'patrol'))

    weed = Node(
        package='agridoc_detection',
        executable='weed_detector',
        output='screen',
        parameters=[{'weights': LaunchConfiguration('weed_weights'),
                     'camera': LaunchConfiguration('weed_camera')}],
        condition=IfCondition(detection),
    )

    disease = Node(
        package='agridoc_detection',
        executable='disease_detector',
        output='screen',
        parameters=[{'model_dir': LaunchConfiguration('disease_model')}],
        condition=IfCondition(detection),
    )

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='patrol',
                              description='teleop or patrol'),
        DeclareLaunchArgument('detection', default_value='true',
                              description='weed spraying and disease logging'),
        DeclareLaunchArgument('route', default_value=os.path.join(ROUTE_DIR, 'gps_route.yaml')),
        DeclareLaunchArgument(
            'mega_port', default_value='/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0'),
        DeclareLaunchArgument(
            'lidar_port',
            default_value='/dev/serial/by-id/'
                          'usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'),
        DeclareLaunchArgument('lidar_flip', default_value='false'),
        DeclareLaunchArgument('weed_weights', default_value=os.path.join(MODEL_DIR, 'best.pt')),
        DeclareLaunchArgument('weed_camera', default_value='/dev/video1'),
        DeclareLaunchArgument('disease_model',
                              default_value=os.path.join(MODEL_DIR, 'disease_model')),
        robot_state_publisher,
        mega,
        gyro_bias,
        ekf,
        lidar,
        patrol,
        weed,
        disease,
    ])
