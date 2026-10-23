import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from mavros_msgs.msg import OverrideRCIn, State

NEUTRAL = 1500
SPAN = 400
NUM_CHANNELS = 18

PITCH = 0
ROLL = 1
THROTTLE = 2
YAW = 3
FORWARD = 4
LATERAL = 5

def clamp(x):
    return max(-1.0, min(1.0, x))

class ArduSubBridge(Node):


    ACTIVATE = 'activate'
    DEACTIVATE = 'deactivate'


    def __init__(self):
        super().__init__('ardusub_bridge')

        self.declare_parameter('max_effort', 0.3)
        self.declare_parameter('cmd_timeout', 0.5)

        self.active = False
        self.cmd = Twist()
        self.cmd_time = None
        self.timed_out = False
        self.last_state = None

        self.command_sub    = self.create_subscription(String, '/ardusub_bridge/command', self.on_command, 10)
        self.status_pub     = self.create_publisher(String, '/ardusub_bridge/status', 10)
        self.effort_sub     = self.create_subscription(Twist, '/cmd_effort', self.on_effort, 10)
        self.state_sub      = self.create_subscription(State, '/mavros/state', self.on_state, 10)
        self.rc_pub         = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)

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
            self.get_logger().info(f'Unrecognized command: {msg.data}')



    def activate(self):
        if self.active:
            self.get_logger().info('Bidge already ACTIVE. Ignoring command')
            return

        self.cmd = Twist()
        self.cmd_time = None
        self.timed_out = False
        self.active = True
        self.get_logger().info('Bridge ACTIVE')
        self.status_pub.publish(String(data='active'))



    def deactivate(self):
        if not self.active:
            self.get_logger().info('Bridge already INACTIVE. Ignoring command')
            return

        self.active = False
        self.release()
        self.get_logger().info('Bridnge INACTIVE')
        self.status_pub.publish(String(data='inactive'))

        

    def on_effort(self, msg):
        self.cmd = msg
        self.cmd_time = self.get_clock().now()
        if self.timed_out:
            self.timed_out = False
            self.get_logger().info('/cmd_effort is back')



    def on_state(self, msg):
        now = (msg.connected, msg.armed, msg.mode)
        if now != self.last_state:
            self.get_logger().info(f'FCU connected:{msg.connected}, armed:{msg.armed}, mode:{msg.mode}')
            self.last_state = now



    def pwm(self, effort):
        limit = max(0.0, min(1.0, self.p('max_effort')))
        return int(round(NEUTRAL + SPAN * limit * clamp(effort)))



    def step(self):
        if not self.active:
            return

        cmd = self.cmd
        stale = self.cmd_time is None or \
            (self.get_clock().now() - self.cmd_time).nanoseconds * 1e-9 > self.p('cmd_timeout')

        if stale:
            if self.cmd_time is not None and not self.timed_out:
                self.get_logger().warn('No /cmd_effort. Sending Neutral')
                self.timed_out = True
            cmd = Twist()

        msg = OverrideRCIn()
        msg.channels = [OverrideRCIn.CHAN_NOCHANGE] * NUM_CHANNELS
        msg.channels[PITCH]     = NEUTRAL
        msg.channels[ROLL]      = NEUTRAL
        msg.channels[THROTTLE]  = self.pwm(cmd.linear.z)
        msg.channels[YAW]       = self.pwm(-cmd.angular.z)
        msg.channels[FORWARD]   = self.pwm(cmd.linear.x)
        msg.channels[LATERAL]   = self.pwm(-cmd.linear.y)

        self.rc_pub.publish(msg)



    def release(self):
        msg = OverrideRCIn()
        msg.channels = [OverrideRCIn.CHAN_NOCHANGE] * NUM_CHANNELS

        for i in range(6):
            msg.channels[i] = OverrideRCIn.CHAN_RELEASE

        self.rc_pub.publish(msg)



def main(args=None):
        rclpy.init(args=args)
        node = ArduSubBridge()
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            if node.active:
                node.deactivate()
            node.destroy_node()
            rclpy.try_shutdown()
