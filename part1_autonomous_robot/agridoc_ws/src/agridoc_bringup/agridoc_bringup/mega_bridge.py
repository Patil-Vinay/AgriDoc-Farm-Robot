import math

import rclpy
import serial
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState, NavSatFix, NavSatStatus
from std_msgs.msg import Bool


# Talks to the Arduino Mega over USB (protocol is in mega_firmware.ino).
# Sends wheel speeds and the pump state, publishes odom, joint_states, imu/data and gps/fix
class MegaBridge(Node):
    def __init__(self):
        super().__init__('mega_bridge')
        # the Mega clone (CH340) and the lidar are both ttyUSB, by-id keeps them apart
        port = self.declare_parameter(
            'port', '/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0').value
        self.radius = self.declare_parameter('wheel_radius', 0.05).value
        self.separation = self.declare_parameter('wheel_separation', 0.42).value
        # must match TICKS_PER_REV and MAX_SPEED in the firmware
        self.ticks_per_rev = self.declare_parameter('ticks_per_rev', 1393.0).value
        self.max_speed = self.declare_parameter('max_wheel_speed', 4.0).value
        self.cmd_timeout = self.declare_parameter('cmd_timeout', 0.5).value
        # the pump stays on only while pump messages keep coming
        self.pump_timeout = self.declare_parameter('pump_timeout', 1.0).value
        self.imu_frame = self.declare_parameter('imu_frame', 'imu_link').value
        self.gps_frame = self.declare_parameter('gps_frame', 'gps_link').value
        # NEO-M8N horizontal accuracy (2.5 m CEP) for an HDOP of 1
        self.gps_accuracy = self.declare_parameter('gps_accuracy', 2.5).value

        # opening the port resets the Mega, nothing is sent until it says READY
        try:
            self.serial = serial.Serial(port, 115200, timeout=0)
        except OSError as e:
            self.get_logger().error('Mega not found: %s' % e)
            raise SystemExit(1)
        self.buffer = b''
        self.ready = False

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_ticks = None
        self.target = (0.0, 0.0)
        self.pump_on = False
        self.last_cmd = self.get_clock().now()
        self.last_pump = self.get_clock().now()

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.gps_pub = self.create_publisher(NavSatFix, 'gps/fix', 10)
        self.gps_fix = None
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)
        self.create_subscription(Bool, 'pump', self.on_pump, 10)
        self.create_timer(0.01, self.receive)
        self.create_timer(0.05, self.send)
        self.get_logger().info('Waiting for the Mega on %s' % port)

    def on_cmd(self, msg):
        # wheel speeds in rad/s, scaled down together so the robot keeps its curve
        v = msg.linear.x
        w = msg.angular.z
        left = (v - w * self.separation / 2.0) / self.radius
        right = (v + w * self.separation / 2.0) / self.radius
        fastest = max(abs(left), abs(right))
        if fastest > self.max_speed:
            left *= self.max_speed / fastest
            right *= self.max_speed / fastest
        self.target = (left, right)
        self.last_cmd = self.get_clock().now()

    def on_pump(self, msg):
        if msg.data != self.pump_on:
            self.get_logger().info('Pump on' if msg.data else 'Pump off')
        self.pump_on = msg.data
        self.last_pump = self.get_clock().now()

    def send(self):
        if not self.ready:
            return
        now = self.get_clock().now()
        left, right = self.target
        if (now - self.last_cmd).nanoseconds / 1e9 > self.cmd_timeout:
            left, right = 0.0, 0.0
        pump = self.pump_on
        if pump and (now - self.last_pump).nanoseconds / 1e9 > self.pump_timeout:
            self.get_logger().info('Pump off, no pump messages')
            self.pump_on = False
            pump = False
        try:
            self.serial.write(b'C %.3f %.3f %d\n' % (left, right, pump))
        except OSError as e:
            self.lost(e)

    def lost(self, error):
        # USB unplugged or the Mega reset badly. The Mega stops on its own,
        # robot.launch.py starts this node again
        self.get_logger().error('Lost the Mega: %s' % error)
        raise SystemExit(1)

    def receive(self):
        try:
            self.buffer += self.serial.read(self.serial.in_waiting)
        except OSError as e:
            self.lost(e)
        while b'\n' in self.buffer:
            line, self.buffer = self.buffer.split(b'\n', 1)
            text = line.decode(errors='ignore').strip()
            if text.startswith('$'):
                self.on_nmea(text)
                continue
            fields = text.split()
            if not fields:
                continue
            try:
                if fields[0] == 'E' and len(fields) == 4:
                    self.on_encoders(int(fields[1]), int(fields[2]), int(fields[3]))
                elif fields[0] == 'I' and len(fields) == 11:
                    self.on_imu([float(f) for f in fields[1:]])
                elif fields[0] == 'READY':
                    self.on_ready(fields)
            except ValueError:
                pass

    def on_ready(self, fields):
        # the Mega restarted, its tick counts start from zero again
        self.last_ticks = None
        self.ready = True
        self.get_logger().info('Mega ready')
        if 'NO_IMU' in fields:
            self.get_logger().error('Mega did not find the BNO055')

    def on_encoders(self, millis, left, right):
        # a Mega that didn't reset on connect never says READY
        self.ready = True
        if self.last_ticks is None:
            self.last_ticks = (millis, left, right)
            return
        dt = (millis - self.last_ticks[0]) / 1000.0
        if dt <= 0.0:
            return
        step = 2.0 * math.pi / self.ticks_per_rev
        d_left = (left - self.last_ticks[1]) * step
        d_right = (right - self.last_ticks[2]) * step
        self.last_ticks = (millis, left, right)

        dist = (d_left + d_right) / 2.0 * self.radius
        turn = (d_right - d_left) * self.radius / self.separation
        self.x += dist * math.cos(self.th + turn / 2.0)
        self.y += dist * math.sin(self.th + turn / 2.0)
        self.th += turn

        now = self.get_clock().now().to_msg()
        self.publish_odom(now, dist / dt, turn / dt)
        self.publish_joints(now, left * step, right * step)

    def publish_odom(self, stamp, v, w):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_footprint'
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation.z = math.sin(self.th / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.th / 2.0)
        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = w
        # the ekf only takes the speeds from here
        msg.twist.covariance[0] = 0.01
        msg.twist.covariance[7] = 0.001
        msg.twist.covariance[35] = 0.05
        self.odom_pub.publish(msg)

    def publish_joints(self, stamp, left, right):
        msg = JointState()
        msg.header.stamp = stamp
        # front wheels have no encoder, they roll about the same as the rear ones
        msg.name = ['rear_left_wheel_joint', 'rear_right_wheel_joint',
                    'front_left_wheel_joint', 'front_right_wheel_joint']
        msg.position = [left, right, left, right]
        self.joint_pub.publish(msg)

    def on_imu(self, values):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.imu_frame
        q = msg.orientation
        q.w, q.x, q.y, q.z = values[0:4]
        g = msg.angular_velocity
        g.x, g.y, g.z = values[4:7]
        a = msg.linear_acceleration
        a.x, a.y, a.z = values[7:10]
        for i in (0, 4, 8):
            msg.orientation_covariance[i] = 0.01
            msg.angular_velocity_covariance[i] = 0.0004
            msg.linear_acceleration_covariance[i] = 0.04
        self.imu_pub.publish(msg)

    def on_nmea(self, sentence):
        # $GNGGA,time,lat,N,lon,E,quality,satellites,hdop,altitude,M,geoid,M,,*checksum
        body, _, checksum = sentence[1:].partition('*')
        value = 0
        for c in body:
            value ^= ord(c)
        try:
            if int(checksum[:2], 16) != value:
                return
        except ValueError:
            return
        f = body.split(',')
        if len(f) < 12 or f[0][2:] != 'GGA':
            return

        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.gps_frame
        msg.status.service = {'GP': NavSatStatus.SERVICE_GPS,
                              'GL': NavSatStatus.SERVICE_GLONASS}.get(
            f[0][:2], NavSatStatus.SERVICE_GPS | NavSatStatus.SERVICE_GLONASS)
        try:
            quality = int(f[6])
            # ddmm.mmmm and dddmm.mmmm
            lat = float(f[2][:2]) + float(f[2][2:]) / 60.0
            lon = float(f[4][:3]) + float(f[4][3:]) / 60.0
            hdop = float(f[8])
            # height above the ellipsoid, like NavSatFix wants
            alt = float(f[9]) + float(f[11])
        except ValueError:
            # no fix yet, the fields are empty
            quality = 0
        if quality == 0:
            msg.status.status = NavSatStatus.STATUS_NO_FIX
            msg.latitude = msg.longitude = msg.altitude = float('nan')
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        else:
            # 2 is an SBAS (DGPS) fix, 4 and 5 are RTK
            msg.status.status = {2: NavSatStatus.STATUS_SBAS_FIX,
                                 4: NavSatStatus.STATUS_GBAS_FIX,
                                 5: NavSatStatus.STATUS_GBAS_FIX}.get(quality, NavSatStatus.STATUS_FIX)
            msg.latitude = -lat if f[3] == 'S' else lat
            msg.longitude = -lon if f[5] == 'W' else lon
            msg.altitude = alt
            horizontal = (hdop * self.gps_accuracy) ** 2
            msg.position_covariance[0] = horizontal
            msg.position_covariance[4] = horizontal
            msg.position_covariance[8] = 4.0 * horizontal
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

        if (quality != 0) != self.gps_fix:
            self.gps_fix = quality != 0
            if self.gps_fix:
                self.get_logger().info('GPS fix with %d satellites' % int(f[7]))
            else:
                self.get_logger().warn('No GPS fix')
        self.gps_pub.publish(msg)

    def shutdown(self):
        try:
            self.serial.write(b'C 0 0 0\n')
            self.serial.close()
        except OSError:
            pass


def main():
    rclpy.init()
    node = MegaBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.shutdown()
    node.destroy_node()
    # ctrl+c has already shut rclpy down
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
