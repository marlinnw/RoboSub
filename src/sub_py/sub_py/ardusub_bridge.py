"""ardusub_bridge.py -- turns /cmd_effort into RC override commands for ArduSub.

Whoever owns control (manual or autonomy) publishes thrust efforts in
[-1, 1] on /cmd_effort. This node converts them to RC PWM values and sends
them to the flight controller through MAVROS at 20 Hz.

PWM mapping:  1500 = neutral,  1500 +/- 400 * max_effort * effort
max_effort scales every axis down, which is useful as a safety limit while
testing.

If /cmd_effort goes quiet for longer than cmd_timeout, neutral (hover) is
sent instead of the last command. On deactivate, the overridden channels are
released so the pilot / ground station gets control back.

COMMAND  /ardusub_bridge/command  std_msgs/String  "activate" / "deactivate"
INPUT    /cmd_effort              geometry_msgs/Twist  efforts in [-1, 1]
         /mavros/state            mavros_msgs/State  (only logged when it changes)
OUTPUT   /mavros/rc/override      mavros_msgs/OverrideRCIn
         /ardusub_bridge/status   std_msgs/String  "active" / "inactive"

Parameters:
    max_effort   0..1, scales all efforts before conversion to PWM
    cmd_timeout  seconds without /cmd_effort before sending neutral

Examples:
  ros2 topic pub --once /ardusub_bridge/command std_msgs/String "{data: 'activate'}"
  ros2 topic pub --once /ardusub_bridge/command std_msgs/String "{data: 'deactivate'}"
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from mavros_msgs.msg import OverrideRCIn, State

NEUTRAL = 1500          # PWM (microseconds) for "no thrust"
SPAN = 400              # PWM swing at full effort: 1100 .. 1900
NUM_CHANNELS = 18       # OverrideRCIn always carries 18 channels

# ArduSub RC input channel assignments (0-indexed: PITCH = RC channel 1)
PITCH = 0
ROLL = 1
THROTTLE = 2            # vertical (up/down)
YAW = 3
FORWARD = 4
LATERAL = 5

def clamp(x):
    """Limit x to the effort range [-1.0, 1.0]."""
    return max(-1.0, min(1.0, x))

class ArduSubBridge(Node):
    """Bridge from /cmd_effort (Twist) to MAVROS RC override."""


    ACTIVATE = 'activate'
    DEACTIVATE = 'deactivate'


    def __init__(self):
        super().__init__('ardusub_bridge')

        self.declare_parameter('max_effort', 0.3)
        self.declare_parameter('cmd_timeout', 0.5)

        self.active = False
        self.cmd = Twist()          # latest /cmd_effort received
        self.cmd_time = None        # when it arrived; None = nothing received yet
        self.timed_out = False      # True while sending neutral because /cmd_effort went quiet
        self.last_state = None      # (connected, armed, mode) last logged

        self.command_sub    = self.create_subscription(String, '/ardusub_bridge/command', self.on_command, 10)
        self.status_pub     = self.create_publisher(String, '/ardusub_bridge/status', 10)
        self.effort_sub     = self.create_subscription(Twist, '/cmd_effort', self.on_effort, 10)
        self.state_sub      = self.create_subscription(State, '/mavros/state', self.on_state, 10)
        self.rc_pub         = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)

        # Send RC override at 20 Hz (does nothing while inactive)
        self.create_timer(0.05, self.step)



    def p(self, name):
        """Current value of a parameter (read every time so live changes work)."""
        return self.get_parameter(name).value



    def on_command(self, msg):
        """Handle a message on /ardusub_bridge/command.

        Expected format (std_msgs/String, case and surrounding spaces ignored):
            "activate"    start sending RC override
            "deactivate"  stop and release the RC channels

        e.g. ros2 topic pub --once /ardusub_bridge/command std_msgs/String "{data: 'activate'}"
        """
        command = msg.data.strip().lower()

        if command == self.ACTIVATE:
            self.activate()

        elif command == self.DEACTIVATE:
            self.deactivate()

        else:
            self.get_logger().info(f'Unrecognized command: {msg.data}')



    def activate(self):
        """Start sending RC override, beginning from a neutral command."""
        if self.active:
            self.get_logger().info('Bidge already ACTIVE. Ignoring command')
            return

        # Forget any command received before activation
        self.cmd = Twist()
        self.cmd_time = None
        self.timed_out = False
        self.active = True
        self.get_logger().info('Bridge ACTIVE')
        self.status_pub.publish(String(data='active'))



    def deactivate(self):
        """Stop sending RC override and release the channels back to the FCU."""
        if not self.active:
            self.get_logger().info('Bridge already INACTIVE. Ignoring command')
            return

        self.active = False
        self.release()
        self.get_logger().info('Bridnge INACTIVE')
        self.status_pub.publish(String(data='inactive'))



    def on_effort(self, msg):
        """Store the latest /cmd_effort; step() sends it on the next tick."""
        self.cmd = msg
        self.cmd_time = self.get_clock().now()
        if self.timed_out:
            self.timed_out = False
            self.get_logger().info('/cmd_effort is back')



    def on_state(self, msg):
        """Log the flight controller's connection / arming / mode when it changes."""
        now = (msg.connected, msg.armed, msg.mode)
        if now != self.last_state:
            self.get_logger().info(f'FCU connected:{msg.connected}, armed:{msg.armed}, mode:{msg.mode}')
            self.last_state = now



    def pwm(self, effort):
        """Convert an effort in [-1, 1] to a PWM value, scaled by max_effort."""
        limit = max(0.0, min(1.0, self.p('max_effort')))
        return int(round(NEUTRAL + SPAN * limit * clamp(effort)))



    def step(self):
        """Timer tick (20 Hz): publish one RC override from the latest command."""
        if not self.active:
            return

        cmd = self.cmd
        stale = self.cmd_time is None or \
            (self.get_clock().now() - self.cmd_time).nanoseconds * 1e-9 > self.p('cmd_timeout')

        if stale:
            # Warn once per dropout (not before the first command ever arrives)
            if self.cmd_time is not None and not self.timed_out:
                self.get_logger().warn('No /cmd_effort. Sending Neutral')
                self.timed_out = True
            cmd = Twist()

        msg = OverrideRCIn()
        # NOCHANGE = leave the channel alone; only the 6 motion channels are overridden
        msg.channels = [OverrideRCIn.CHAN_NOCHANGE] * NUM_CHANNELS
        msg.channels[PITCH]     = NEUTRAL
        msg.channels[ROLL]      = NEUTRAL
        msg.channels[THROTTLE]  = self.pwm(cmd.linear.z)
        # ROS: +angular.z = turn left, +linear.y = move left.
        # ArduSub: PWM above neutral = turn right / move right. Hence the sign flips.
        msg.channels[YAW]       = self.pwm(-cmd.angular.z)
        msg.channels[FORWARD]   = self.pwm(cmd.linear.x)
        msg.channels[LATERAL]   = self.pwm(-cmd.linear.y)

        self.rc_pub.publish(msg)



    def release(self):
        """Hand the 6 motion channels back to the FCU (stop overriding them)."""
        msg = OverrideRCIn()
        msg.channels = [OverrideRCIn.CHAN_NOCHANGE] * NUM_CHANNELS

        for i in range(6):
            msg.channels[i] = OverrideRCIn.CHAN_RELEASE

        self.rc_pub.publish(msg)



def main(args=None):
        """Entry point for 'ros2 run sub_py ardusub_bridge'."""
        rclpy.init(args=args)
        node = ArduSubBridge()
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            # Release the RC channels so the sub isn't left on the last override
            if node.active:
                node.deactivate()
            node.destroy_node()
            rclpy.try_shutdown()
