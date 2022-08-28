import math
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_srvs.srv import Empty


def make_pose(nav, x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = nav.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def settle_amcl(nav, times=5):
    # amcl only updates while the robot moves, so its last update can be from
    # the middle of a turn. Ask for a few more once the robot has stopped.
    client = nav.create_client(Empty, 'request_nomotion_update')
    if not client.wait_for_service(timeout_sec=2.0):
        return
    for _ in range(times):
        future = client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(nav, future, timeout_sec=2.0)
        time.sleep(0.5)


def main():
    rclpy.init()
    nav = BasicNavigator()

    nav.declare_parameter('route_file', '')
    route_file = nav.get_parameter('route_file').value
    with open(route_file) as f:
        route = yaml.safe_load(f)

    home = make_pose(nav, *route['home'])
    points = [make_pose(nav, *p) for p in route['waypoints']]
    # finish where we started
    points.append(make_pose(nav, *route['home']))

    nav.setInitialPose(home)
    nav.waitUntilNav2Active()

    nav.info('Starting patrol with %d points' % len(points))
    nav.followWaypoints(points)

    last = -1
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        if feedback and feedback.current_waypoint != last:
            last = feedback.current_waypoint
            nav.info('Going to point %d of %d' % (last + 1, len(points)))

    result = nav.getResult()
    if result == TaskResult.SUCCEEDED:
        settle_amcl(nav)
        nav.info('Patrol done, robot is back home')
    elif result == TaskResult.CANCELED:
        nav.warn('Patrol was canceled')
    else:
        nav.error('Patrol failed')

    nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
