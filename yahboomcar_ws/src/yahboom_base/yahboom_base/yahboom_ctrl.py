import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3, TransformStamped
from std_msgs.msg import Int32, String, Bool, Float32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Imu
from tf2_ros import TransformBroadcaster
import DOGZILLALib as dog

_JOINT_NAMES = [
    "lf_upper_leg_joint", "lf_lower_leg_link", "lf_hip_joint",
    "lh_upper_leg_joint", "lh_lower_leg_joint", "lh_hip_joint",
    "rf_upper_leg_joint", "rf_lower_leg_joint", "rf_hip_joint",
    "rh_upper_leg_joint", "rh_lower_leg_joint", "rh_hip_joint",
]
_JOINT_SIGNS = [1, -1, 1, 1, -1, 1, -1, 1, 1, -1, 1, 1]
_SENSOR_PERIOD = 0.1  # seconds — serial reads at 10 Hz


class YahboomCtrl(Node):
    """ROS 2 ↔ DOGZILLALib bridge — sole owner of the serial bus.

    Subscribed topics:
      /cmd_vel              geometry_msgs/Twist    — movement (vx, vy, wz)
      /dogzilla/action      std_msgs/Int32         — action id (1-19, 255=reset)
      /dogzilla/pace        std_msgs/String        — 'slow' | 'normal' | 'high'
      /dogzilla/translation geometry_msgs/Vector3  — x(±35mm) y(±18mm) z(75-115mm)
      /dogzilla/attitude    geometry_msgs/Vector3  — x=roll(±20°) y=pitch(±15°) z=yaw(±11°)
      /dogzilla/imu         std_msgs/Bool          — IMU stabilisation on/off
      /dogzilla/perform     std_msgs/Int32         — action carousel (0=stop 1=start)

    Published topics:
      /battery_voltage      std_msgs/Float32       — battery level (V), polled at 5 s
      /joint_states         sensor_msgs/JointState — 12-DOF servo angles at 10 Hz
      /imu/data_raw_self    sensor_msgs/Imu        — roll/pitch/yaw orientation at 10 Hz
      /odom                 nav_msgs/Odometry      — dead-reckoning (only if publish_odom:=true)
      TF odom→base_footprint                       — only if publish_odom:=true
    """

    RATE = 40
    MIN_VEL = 0.05

    def __init__(self):
        super().__init__('yahboom_ctrl')
        self.dog = dog.DOGZILLA()
        self._action_active = False

        self.declare_parameter('publish_odom', False)
        self._publish_odom_enabled = self.get_parameter('publish_odom').value

        self.create_subscription(Twist,   'cmd_vel',              self._cmd_vel_cb,     10)
        self.create_subscription(Int32,   'dogzilla/action',      self._action_cb,      10)
        self.create_subscription(String,  'dogzilla/pace',        self._pace_cb,        10)
        self.create_subscription(Vector3, 'dogzilla/translation', self._translation_cb, 10)
        self.create_subscription(Vector3, 'dogzilla/attitude',    self._attitude_cb,    10)
        self.create_subscription(Bool,    'dogzilla/imu',         self._imu_cb,         10)
        self.create_subscription(Int32,   'dogzilla/perform',     self._perform_cb,     10)

        self._bat_pub = self.create_publisher(Float32, 'battery_voltage', 10)
        self.create_timer(5.0, self._publish_battery)

        # Joint states + IMU — read from the same serial instance as commands.
        self._joint_pub = self.create_publisher(JointState, '/joint_states', 5)
        self._imu_pub = self.create_publisher(Imu, 'imu/data_raw_self', 10)
        self._last_angles = [0.0] * 12
        self._imu_msg = Imu()
        self._imu_msg.header.frame_id = 'imu_link'
        self._imu_msg.orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        self._imu_msg.angular_velocity_covariance[0] = -1.0
        self._imu_msg.linear_acceleration_covariance[0] = -1.0
        self.create_timer(_SENSOR_PERIOD, self._publish_sensors)

        if self._publish_odom_enabled:
            self._x = 0.0
            self._y = 0.0
            self._theta = 0.0
            self._vx = 0.0
            self._vy = 0.0
            self._wz = 0.0
            self._last_odom_time = self.get_clock().now()
            self._odom_pub = self.create_publisher(Odometry, 'odom', 10)
            self._tf_broadcaster = TransformBroadcaster(self)
            self.create_timer(0.05, self._publish_odom)
            self.get_logger().info('yahboom_ctrl: dead-reckoning odom enabled')

        self.get_logger().info('yahboom_ctrl bridge ready')

    # ── motion ────────────────────────────────────────────────────────────────

    def _cmd_vel_cb(self, msg):
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        if self._publish_odom_enabled:
            self._vx = vx
            self._vy = vy
            self._wz = wz

        if abs(vx) < self.MIN_VEL and abs(vy) < self.MIN_VEL and abs(wz) < self.MIN_VEL:
            # Don't interrupt a running action with a spurious stop().
            # The browser publishes zero-vel on key release; that would cut actions short.
            if not self._action_active:
                self.dog.stop()
            return

        self._action_active = False
        self.dog.move('x', vx * self.RATE)
        self.dog.move('y', vy * self.RATE)
        self.dog.turn(wz * self.RATE)

    def _action_cb(self, msg):
        if msg.data == 255:
            self._action_active = False
        else:
            self._action_active = True
        self.dog.action(msg.data)

    def _pace_cb(self, msg):
        mode = msg.data.lower()
        if mode in ('slow', 'normal', 'high'):
            self.dog.pace(mode)
        else:
            self.get_logger().warn(f'Unknown pace: {msg.data}')

    def _translation_cb(self, msg):
        self.dog.translation('x', msg.x)
        self.dog.translation('y', msg.y)
        self.dog.translation('z', msg.z)

    def _attitude_cb(self, msg):
        self.dog.attitude('r', msg.x)
        self.dog.attitude('p', msg.y)
        self.dog.attitude('y', msg.z)

    def _imu_cb(self, msg):
        self.dog.imu(1 if msg.data else 0)

    def _perform_cb(self, msg):
        self.dog.perform(msg.data)

    # ── dead-reckoning odometry ───────────────────────────────────────────────

    def _publish_odom(self):
        now = self.get_clock().now()
        dt = (now - self._last_odom_time).nanoseconds / 1e9
        self._last_odom_time = now

        self._x += (self._vx * math.cos(self._theta) - self._vy * math.sin(self._theta)) * dt
        self._y += (self._vx * math.sin(self._theta) + self._vy * math.cos(self._theta)) * dt
        self._theta += self._wz * dt

        qz = math.sin(self._theta / 2.0)
        qw = math.cos(self._theta / 2.0)

        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self._x
        t.transform.translation.y = self._y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self._vx
        odom.twist.twist.linear.y = self._vy
        odom.twist.twist.angular.z = self._wz
        self._odom_pub.publish(odom)

    # ── sensors (joint states + IMU) ─────────────────────────────────────────

    def _publish_sensors(self):
        if self._action_active:
            return
        now = self.get_clock().now()
        try:
            angles = self.dog.read_motor()
            js = JointState()
            js.header.stamp = now.to_msg()
            js.name = _JOINT_NAMES
            js.position = [s * a * math.pi / 180 for s, a in zip(_JOINT_SIGNS, angles)]
            js.velocity = [
                (a - p) * math.pi / 180 / _SENSOR_PERIOD
                for a, p in zip(angles, self._last_angles)
            ]
            js.effort = [float('nan')] * 12
            self._last_angles = list(angles)
            self._joint_pub.publish(js)
        except Exception as e:
            self.get_logger().error(f'Motor read failed: {e}', throttle_duration_sec=1.0)

        try:
            roll = self.dog.read_roll()
            pitch = self.dog.read_pitch()
            yaw = self.dog.read_yaw()
            qx, qy, qz, qw = self._euler_to_quat(roll, pitch, yaw)
            self._imu_msg.header.stamp = self.get_clock().now().to_msg()
            self._imu_msg.orientation.x = qx
            self._imu_msg.orientation.y = qy
            self._imu_msg.orientation.z = qz
            self._imu_msg.orientation.w = qw
            self._imu_pub.publish(self._imu_msg)
        except Exception as e:
            self.get_logger().error(f'IMU read failed: {e}', throttle_duration_sec=1.0)

    @staticmethod
    def _euler_to_quat(roll, pitch, yaw):
        sr, cr = np.sin(roll / 2), np.cos(roll / 2)
        sp, cp = np.sin(pitch / 2), np.cos(pitch / 2)
        sy, cy = np.sin(yaw / 2), np.cos(yaw / 2)
        return (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )

    # ── telemetry ─────────────────────────────────────────────────────────────

    def _publish_battery(self):
        try:
            msg = Float32()
            msg.data = float(self.dog.read_battery())
            self._bat_pub.publish(msg)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = YahboomCtrl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
