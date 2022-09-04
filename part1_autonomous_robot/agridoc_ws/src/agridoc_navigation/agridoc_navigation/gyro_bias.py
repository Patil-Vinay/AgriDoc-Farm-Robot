import rclpy
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu


# Learns the gyro bias while the wheels say the robot is standing still
# and removes it before the IMU reaches the EKF
class GyroBias(Node):
    def __init__(self):
        super().__init__('gyro_bias')
        # below these the wheels count as stopped, the sim wheels jitter up to about 0.002
        self.max_linear = self.declare_parameter('max_linear', 0.005).value
        self.max_angular = self.declare_parameter('max_angular', 0.005).value
        # wait this long after stopping so the robot has stopped rocking
        self.settle_time = self.declare_parameter('settle_time', 1.0).value
        # averaging window in seconds
        self.time_constant = self.declare_parameter('time_constant', 10.0).value

        self.still_since = None
        self.last_odom = None
        self.last_imu = None
        self.samples = 0
        self.bias = [0.0, 0.0, 0.0]

        self.imu_pub = self.create_publisher(Imu, 'imu/data_corrected', 10)
        self.bias_pub = self.create_publisher(Vector3Stamped, '~/bias', 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_subscription(Imu, 'imu/data', self.on_imu, qos_profile_sensor_data)

    def on_odom(self, msg):
        v = msg.twist.twist
        still = abs(v.linear.x) < self.max_linear and abs(v.angular.z) < self.max_angular
        self.last_odom = self.get_clock().now()
        if not still:
            self.still_since = None
        elif self.still_since is None:
            self.still_since = self.last_odom

    def on_imu(self, msg):
        now = self.get_clock().now()
        stamp = Time.from_msg(msg.header.stamp)
        fresh = self.last_odom is not None and (now - self.last_odom).nanoseconds < 0.5e9
        settled = (self.still_since is not None and
                   (now - self.still_since).nanoseconds > self.settle_time * 1e9)

        if fresh and settled:
            dt = 0.02
            if self.last_imu is not None:
                dt = (stamp - self.last_imu).nanoseconds * 1e-9
                if dt <= 0.0 or dt > 0.5:
                    dt = 0.02
            # plain mean over the first samples, then a moving average that follows slow drift
            self.samples += 1
            a = max(1.0 / self.samples, dt / self.time_constant)
            gyro = msg.angular_velocity
            for i, value in enumerate((gyro.x, gyro.y, gyro.z)):
                self.bias[i] += a * (value - self.bias[i])

            out = Vector3Stamped()
            out.header = msg.header
            out.vector.x, out.vector.y, out.vector.z = self.bias
            self.bias_pub.publish(out)
        self.last_imu = stamp

        msg.angular_velocity.x -= self.bias[0]
        msg.angular_velocity.y -= self.bias[1]
        msg.angular_velocity.z -= self.bias[2]
        self.imu_pub.publish(msg)


def main():
    rclpy.init()
    node = GyroBias()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
