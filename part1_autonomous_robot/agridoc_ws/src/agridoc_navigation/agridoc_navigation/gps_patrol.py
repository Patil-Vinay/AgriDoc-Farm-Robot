import math

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry
from robot_localization.srv import FromLL


def make_pose(nav, x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = nav.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def wait_for_gps(nav):
    # navsat_transform publishes odometry/gps once the GPS has a fix
    got = []
    sub = nav.create_subscription(Odometry, 'odometry/gps', got.append, 10)
    nav.info('Waiting for a GPS fix')
    while not got:
        rclpy.spin_once(nav, timeout_sec=1.0)
    nav.destroy_subscription(sub)


def to_map(nav, client, lat, lon):
    request = FromLL.Request()
    request.ll_point.latitude = float(lat)
    request.ll_point.longitude = float(lon)
    request.ll_point.altitude = 0.0
    future = client.call_async(request)
    rclpy.spin_until_future_complete(nav, future)
    point = future.result().map_point
    return point.x, point.y


def main():
    rclpy.init()
    nav = BasicNavigator()

    nav.declare_parameter('route_file', '')
    route_file = nav.get_parameter('route_file').value
    with open(route_file) as f:
        route = yaml.safe_load(f)

    # robot_localization is not a lifecycle node, so wait for the controller instead of amcl
    nav.waitUntilNav2Active(localizer='controller_server')
    wait_for_gps(nav)

    client = nav.create_client(FromLL, 'fromLL')
    client.wait_for_service()
    points = [to_map(nav, client, lat, lon) for lat, lon in route['waypoints']]
    # finish at home, which is the origin of the map frame
    points.append((0.0, 0.0))

    # face the next point when arriving, at home face the way the robot started
    poses = []
    for i, (x, y) in enumerate(points):
        if i + 1 < len(points):
            yaw = math.atan2(points[i + 1][1] - y, points[i + 1][0] - x)
        else:
            yaw = 0.0
        poses.append(make_pose(nav, x, y, yaw))

    for i, pose in enumerate(poses):
        nav.info('Point %d at (%.2f, %.2f)' % (i + 1, pose.pose.position.x, pose.pose.position.y))
    nav.info('Starting patrol with %d points' % len(poses))
    nav.followWaypoints(poses)

    last = -1
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        if feedback and feedback.current_waypoint != last:
            last = feedback.current_waypoint
            nav.info('Going to point %d of %d' % (last + 1, len(poses)))

    result = nav.getResult()
    if result == TaskResult.SUCCEEDED:
        nav.info('Patrol done, robot is back home')
    elif result == TaskResult.CANCELED:
        nav.warn('Patrol was canceled')
    else:
        nav.error('Patrol failed')

    nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
