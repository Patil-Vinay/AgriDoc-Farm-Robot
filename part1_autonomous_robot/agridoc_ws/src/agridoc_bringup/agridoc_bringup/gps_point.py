import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus


# Averages the GPS fixes for a while and prints the point for the route file.
# Drive the robot to a point in teleop mode, stop and run:
#   ros2 run agridoc_bringup gps_point
# At home add the heading the robot faces (compass degrees) as the third value
class GpsPoint(Node):
    def __init__(self):
        super().__init__('gps_point')
        self.duration = self.declare_parameter('duration', 30.0).value
        self.fixes = []
        self.start = None
        self.create_subscription(NavSatFix, 'gps/fix', self.on_fix, qos_profile_sensor_data)
        self.get_logger().info('Keep the robot still, averaging the GPS for %.0f s' % self.duration)

    def on_fix(self, msg):
        if msg.status.status == NavSatStatus.STATUS_NO_FIX or math.isnan(msg.latitude):
            self.get_logger().warn('No GPS fix', throttle_duration_sec=5.0)
            return
        now = self.get_clock().now()
        if self.start is None:
            self.start = now
        self.fixes.append((msg.latitude, msg.longitude))

    def done(self):
        return (self.start is not None and
                (self.get_clock().now() - self.start).nanoseconds / 1e9 >= self.duration)


def main():
    rclpy.init()
    node = GpsPoint()
    try:
        while rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    if node.fixes:
        lat = sum(f[0] for f in node.fixes) / len(node.fixes)
        lon = sum(f[1] for f in node.fixes) / len(node.fixes)
        # spread of the fixes in metres, to see how steady the GPS was
        spread = max(math.hypot((f[0] - lat) * 111320.0,
                                (f[1] - lon) * 111320.0 * math.cos(math.radians(lat)))
                     for f in node.fixes)
        print('  - [%.8f, %.8f]    # %d fixes, all within %.1f m' % (lat, lon, len(node.fixes), spread))
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
