import math
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PointStamped, Twist
from std_msgs.msg import String

def clamp(x):
    return max(-1.0, min(1.0, x))

class RailFollower(Node):

    ACTIVATE = 'activate'
    DEACTIVATE = 'deactivate'

    def __init__(self):
        super().__init__('rail_follower')

        self.declare_parameter('image_width', 320)          # pixels
        self.declare_parameter('kp_sway', 0.8)              # sideways  effort per unit off center
        self.declare_parameter('kp_yaw', 0.02)              # turning effort per degree of angle
        self.declare_parameter('surge', 0.4)                # forward effort when lined up
        self.declare_parameter('x_limit', 0.5)              # this far off center -> forward drops to 0
        self.declare_parameter('angle_limit', 30.0)         # this many degrees off -> forward drops to 0
        self.declare_parameter('alpha', 0.3)                # smoothing: 1 = none, lower = smoother but lags more
        self.declare_parameter('lost_timeout', 1.0)         # seconds of silence from the detect -> hover

        self.active         = False
        self.x_smooth       = None
        self.angle_smooth   = None
        self.target_time    = None

        self.command_sub    = self.create_subscription(String, '/rail_follower/command', self.on_command, 10)
        self.target_sub     = self.create_subscription(PointStamped, '/rail_detector/target', self.on_target, 10)
        self.cmd_pub        = self.create_publisher(Twist, '/cmd_effort', 10)
        self.status_pub     = self.create_publisher(String, '/rail_follower/status', 10)

        self.create_timer(0.05, self.step)



    def p(self, name):
        return self.get_parameter(name).value


    def on_command(self, msg):
        command = msg.data.strip().lower()

        if command == self.ACTIVATE:
            self.activate()
        elif command == self.DEACTIVATE:
            self.deactivate()
        else:
            self.get_logger().warn(f'Unrecognized command: {msg.data}')


    def activate(self):
        if self.active:
            self.get_logger().info('Rail follower already active. Ignoring Command')
            return

        self.x_smooth = None
        self.angle_smooth = None
        self.active = True
        self.get_logger().info('Rail follower: ACTIVE')
        self.status_pub.publish(String(data='active'))

    def deactivate(self):
        if not self.active:
            self.get_logger().info('Rail follower already inactive. Ignoring command')
            return

        self.active = False
        self.cmd_pub.publish(Twist())
        self.get_logger().info('Rail follwer: INACTIVE')
        self.status_pub.publish(String(data='inactive'))



    def on_target(self, msg):
        if math.isnan(msg.point.x):
            self.x_smooth = None
            self.angle_smooth = None
            return

        a = self.p('alpha')
        if self.x_smooth is None:
            self.x_smooth = msg.point.x
            self.angle_smooth = msg.point.z
        else:
            self.x_smooth = a * msg.point.x + (1 - a) * self.x_smooth
            self.angle_smooth = a * msg.point.z + (1 - a) * self.angle_smooth

        self.target_time = self.get_clock().now()

    def step(self):
        if not self.active:
            return

        cmd = Twist()

        if self.x_smooth is None:
            self.cmd_pub.publish(cmd)
            return

        age = (self.get_clock().now() - self.target_time).nanoseconds * 1e-9

        if age > self.p('lost_timeout'):
            self.get_logger().warn(f'No target for {age:.1f} s. Hovering', throttle_duration_sec=2.0)
            self.x_smooth = None
            self.angle_smooth = None
            self.cmd_pub.publish(cmd)
            return

        half = self.p('image_width') / 2
        x_err = (self.x_smooth - half) / half
        angle = self.angle_smooth

        cmd.linear.y = clamp(-self.p('kp_sway') * x_err)
        cmd.angular.z = clamp(-self.p('kp_yaw') * angle)

        off = max(abs(x_err) / self.p('x_limit'), abs(angle) / self.p('angle_limit'))
        cmd.linear.x = clamp(self.p('surge') * max(0.0, 1.0 - off))

        self.cmd_pub.publish(cmd)



def main(args=None):
    rclpy.init(args=args)
    node = RailFollower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()