"""rail_follower.py -- steers the sub along a rail seen by the rail detector.

Turns the rail position/angle from rail_detector into thrust efforts on
/cmd_effort with three simple proportional controllers:
    sway  (linear.y)  -> slide sideways to center the rail in the image
    yaw   (angular.z) -> turn so the rail points straight up the image
    surge (linear.x)  -> drive forward, slowing to 0 as the sub gets off line

Only one node may drive /cmd_effort at a time. Ownership is negotiated on the
latched topic /control_command/active, which holds the NAME of the current
owner ('manual', 'rail_follower') or '' when nobody owns it.

COMMAND  /rail_follower/command   std_msgs/String  "activate" / "deactivate"
INPUT    /rail_detector/target    geometry_msgs/PointStamped
             x = rail center column (pixels), y = row (pixels), z = angle (deg)
             all NaN = detector running but no rail in view
OUTPUT   /cmd_effort              geometry_msgs/Twist  efforts in [-1, 1]
         /rail_follower/status    std_msgs/String  "active" / "inactive" / "refused"
BOTH     /control_command/active  std_msgs/String  (latched) current owner

Example:
  ros2 topic pub --once /rail_follower/command std_msgs/String "{data: 'activate'}"
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PointStamped, Twist
from std_msgs.msg import String

# "Latched" QoS: the last message is kept and handed to anyone who subscribes
# later, so a node started late still learns who currently owns control.
LATCHED = QoSProfile(depth=1,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)


def clamp(x):
    """Limit x to the effort range [-1.0, 1.0]."""
    return max(-1.0, min(1.0, x))


class RailFollower(Node):
    """Proportional rail-following controller that publishes /cmd_effort.

    Runs a 20 Hz control loop while active. Targets are low-pass filtered
    (exponential moving average) to reduce jitter from the detector. If the
    rail is lost or the detector goes quiet, the sub hovers (zero effort).
    """

    NAME = 'rail_follower'          # how this node identifies itself on /control_command/active
    MANUAL = 'manual'               # owner name used by controller_input
    ACTIVATE = 'activate'
    DEACTIVATE = 'deactivate'

    def __init__(self):
        super().__init__('rail_follower')

        self.declare_parameter('image_width', 320)          # pixels
        self.declare_parameter('kp_sway', 0.8)              # sideways effort per unit off center
        self.declare_parameter('kp_yaw', 0.02)              # turning effort per degree of angle
        self.declare_parameter('surge', 0.4)                # forward effort when lined up
        self.declare_parameter('x_limit', 0.5)              # this far off center -> forward drops to 0
        self.declare_parameter('angle_limit', 30.0)         # this many degrees off -> forward drops to 0
        self.declare_parameter('alpha', 0.3)                # smoothing: 1 = none, lower = smoother but lags more
        self.declare_parameter('lost_timeout', 1.0)         # seconds of silence from the detector -> hover

        self.active       = False
        self.owner        = ''                              # last value seen on /control_command/active
        self.x_smooth     = None                            # filtered rail column (pixels); None = no target
        self.angle_smooth = None                            # filtered rail angle (degrees)
        self.target_time  = None                            # when the last valid target arrived

        self.command_sub = self.create_subscription(String, '/rail_follower/command', self.on_command, 10)
        self.target_sub  = self.create_subscription(PointStamped, '/rail_detector/target', self.on_target, 10)
        self.owner_sub   = self.create_subscription(String, '/control_command/active', self.on_owner, LATCHED)
        self.owner_pub   = self.create_publisher(String, '/control_command/active', LATCHED)
        self.cmd_pub     = self.create_publisher(Twist, '/cmd_effort', 10)
        self.status_pub  = self.create_publisher(String, '/rail_follower/status', 10)

        # Control loop at 20 Hz (does nothing while inactive)
        self.create_timer(0.05, self.step)

    def p(self, name):
        """Current value of a parameter (read every time so live changes work)."""
        return self.get_parameter(name).value

    # ---------- commands / ownership ----------

    def on_command(self, msg):
        """Handle a message on /rail_follower/command.

        Expected format (std_msgs/String, case and surrounding spaces ignored):
            "activate"    claim /cmd_effort and start following (refused if manual holds it)
            "deactivate"  stop, send zero effort, and release /cmd_effort

        e.g. ros2 topic pub --once /rail_follower/command std_msgs/String "{data: 'activate'}"
        """
        command = msg.data.strip().lower()

        if command == self.ACTIVATE:
            self.activate()
        elif command == self.DEACTIVATE:
            self.deactivate()
        else:
            self.get_logger().warn(f'Unrecognized command: {msg.data}')

    def on_owner(self, msg):
        """Track the control owner; step aside if someone else takes over."""
        self.owner = msg.data
        if self.active and msg.data != self.NAME:
            self.get_logger().info(f'"{msg.data}" took control. Rail follower deactivating.')
            # The new owner already announced itself, so don't publish
            # a zero command or clear the owner on top of it.
            self.deactivate(announce=False)

    def activate(self):
        """Claim /cmd_effort and start following, unless manual control holds it."""
        if self.active:
            self.get_logger().info('Rail follower already active. Ignoring command')
            return
        if self.owner == self.MANUAL:
            self.get_logger().warn('Manual control is active. Release it on the controller first.')
            self.status_pub.publish(String(data='refused'))
            return

        # Start the filter fresh so stale targets from a previous run aren't used
        self.x_smooth = None
        self.angle_smooth = None
        self.active = True
        self.owner_pub.publish(String(data=self.NAME))
        self.get_logger().info('Rail follower: ACTIVE')
        self.status_pub.publish(String(data='active'))

    def deactivate(self, announce=True):
        """Stop following.

        announce=True (normal stop): send one zero-effort command and release
        ownership. announce=False is used when another node has already taken
        control, so we stay quiet and don't overwrite its claim.
        """
        if not self.active:
            self.get_logger().info('Rail follower already inactive. Ignoring command')
            return

        self.active = False
        if announce:
            self.cmd_pub.publish(Twist())
            self.owner_pub.publish(String(data=''))
        self.get_logger().info('Rail follower: INACTIVE')
        self.status_pub.publish(String(data='inactive'))

    # ---------- control ----------

    def on_target(self, msg):
        """Smooth each new detector target with an exponential moving average.

        A NaN target means the detector sees no rail: clear the filter so
        step() hovers until the rail is found again.
        """
        if math.isnan(msg.point.x):
            self.x_smooth = None
            self.angle_smooth = None
            return

        a = self.p('alpha')
        if self.x_smooth is None:
            # First target after a reset: take it as-is
            self.x_smooth = msg.point.x
            self.angle_smooth = msg.point.z
        else:
            self.x_smooth = a * msg.point.x + (1 - a) * self.x_smooth
            self.angle_smooth = a * msg.point.z + (1 - a) * self.angle_smooth

        self.target_time = self.get_clock().now()

    def step(self):
        """Control loop tick (20 Hz): compute and publish one /cmd_effort.

        Publishes zero effort (hover) when there is no target or the last
        target is older than lost_timeout.
        """
        if not self.active:
            return

        cmd = Twist()       # all zeros = hover

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

        # Horizontal error normalized to [-1, 1]: -1 = left edge, 0 = center, +1 = right edge
        half = self.p('image_width') / 2
        x_err = (self.x_smooth - half) / half
        angle = self.angle_smooth           # degrees, + = top of rail leans right

        # ROS convention: +linear.y = left, +angular.z = turn left (CCW).
        # Rail to the right / leaning right -> negative = sway / turn right.
        cmd.linear.y = clamp(-self.p('kp_sway') * x_err)
        cmd.angular.z = clamp(-self.p('kp_yaw') * angle)

        # How far off line we are, as a fraction of the limits (worst of the two).
        # Full surge when perfectly lined up, falling linearly to 0 at the limit.
        off = max(abs(x_err) / self.p('x_limit'), abs(angle) / self.p('angle_limit'))
        cmd.linear.x = clamp(self.p('surge') * max(0.0, 1.0 - off))

        self.cmd_pub.publish(cmd)


def main(args=None):
    """Entry point for 'ros2 run sub_py rail_follower'."""
    rclpy.init(args=args)
    node = RailFollower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
