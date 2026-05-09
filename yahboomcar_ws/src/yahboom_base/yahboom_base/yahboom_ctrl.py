import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Int32, String
import DOGZILLALib as dog


class YahboomCtrl(Node):
    """ROS 2 ↔ DOGZILLALib bridge.

    Topics subscribed:
      /cmd_vel              geometry_msgs/Twist    — movement (vx, vy, wz)
      /dogzilla/action      std_msgs/Int32         — action id (1-19, 255=reset)
      /dogzilla/pace        std_msgs/String        — 'slow' | 'normal' | 'high'
      /dogzilla/translation geometry_msgs/Vector3  — x(±35mm) y(±18mm) z(75-115mm)
      /dogzilla/attitude    geometry_msgs/Vector3  — x=roll(±20°) y=pitch(±15°) z=yaw(±11°)
    """

    RATE = 40
    MIN_VEL = 0.05

    def __init__(self):
        super().__init__('yahboom_ctrl')
        self.dog = dog.DOGZILLA()

        self.create_subscription(Twist, 'cmd_vel', self._cmd_vel_cb, 10)
        self.create_subscription(Int32, 'dogzilla/action', self._action_cb, 10)
        self.create_subscription(String, 'dogzilla/pace', self._pace_cb, 10)
        self.create_subscription(Vector3, 'dogzilla/translation', self._translation_cb, 10)
        self.create_subscription(Vector3, 'dogzilla/attitude', self._attitude_cb, 10)

        self.get_logger().info('yahboom_ctrl bridge ready')

    def _cmd_vel_cb(self, msg):
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z
        if abs(vx) < self.MIN_VEL and abs(vy) < self.MIN_VEL and abs(wz) < self.MIN_VEL:
            self.dog.stop()
            return
        self.dog.move('x', vx * self.RATE)
        self.dog.move('y', vy * self.RATE)
        self.dog.turn(wz * self.RATE)

    def _action_cb(self, msg):
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


def main(args=None):
    rclpy.init(args=args)
    node = YahboomCtrl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
