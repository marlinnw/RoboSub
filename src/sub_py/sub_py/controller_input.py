"""controller_input.py -- manual control of the sub from a gamepad (/joy).

Converts joystick input into thrust efforts on /cmd_effort. Manual control is
a safety override: holding the deadman button takes control away from any
autonomous node.

Control ownership is negotiated on the latched topic /control_command/active,
which holds the NAME of the current owner ('manual', 'rail_follower') or ''
when nobody owns it.

States:
    inactive    -> ignores the controller completely
    active      -> standby: listens to the controller but doesn't drive
    in_control  -> owns /cmd_effort and drives while the deadman is held

Buttons / axes (indices into sensor_msgs/Joy):
    buttons[9]  deadman   hold to take control and drive; let go -> hover
    buttons[6]  release   hand control back (owner -> '')
    buttons[4]  autonomy  "Select": sends "activate" to autonomy_command_topic
    axes[1]     forward / back     -> linear.x
    axes[0]     left / right       -> linear.y
    axes[4/5]   left / right trigger -> linear.z (down / up)
    axes[2]     yaw                -> angular.z

If /joy goes quiet for 2 s while active, the node takes control and holds
the sub at zero effort (hover) as a failsafe.

COMMAND  /controller_input/command  std_msgs/String  "activate" / "deactivate"
INPUT    /joy                       sensor_msgs/Joy
OUTPUT   /cmd_effort                geometry_msgs/Twist  efforts in [-1, 1]
         /controller_input/status   std_msgs/String  "active" / "inactive" / "in_control"
         <autonomy_command_topic>   std_msgs/String  "activate" (only if the parameter is set)
BOTH     /control_command/active    std_msgs/String  (latched) current owner

Parameters:
    autonomy_command_topic  topic the Select button sends "activate" to
                            (e.g. /rail_follower/command); '' disables it

Examples:
  ros2 topic pub --once /controller_input/command std_msgs/String "{data: 'activate'}"
  ros2 topic pub --once /controller_input/command std_msgs/String "{data: 'deactivate'}"
  ros2 run sub_py controller_input --ros-args -p autonomy_command_topic:=/rail_follower/command
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# "Latched" QoS: late subscribers still receive the last message published
LATCHED = QoSProfile(depth=1,
                     durability = DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability = ReliabilityPolicy.RELIABLE)


class ControllerInput(Node):
    """Gamepad teleop node with deadman override and autonomy hand-off."""

    NAME = 'manual'             # how this node identifies itself on /control_command/active
    ACTIVATE = 'activate'
    DEACTIVATE = 'deactivate'
    DEADMAN_BUTTON = 9
    RELEASE_BUTTON = 6
    AUTONOMY_BUTTON = 4



    def __init__(self):
        super().__init__('controller_input')

        self.active = False         # listening to the controller at all
        self.in_control = False     # we currently own /cmd_effort
        self.prev_autonomy = 0      # last autonomy button state, for edge detection

        self.command_sub        = self.create_subscription(String, '/controller_input/command', self.on_command, 10)
        self.status_pub         = self.create_publisher(String, '/controller_input/status', 10)
        self.owner_sub          = self.create_subscription(String, '/control_command/active', self.on_owner, LATCHED)
        self.owner_pub          = self.create_publisher(String, '/control_command/active', LATCHED)
        self.joy_sub            = self.create_subscription(Joy, '/joy', self.on_joy, 10)
        self.cmd_pub            = self.create_publisher(Twist, '/cmd_effort', 10)

        self.declare_parameter('autonomy_command_topic', '')

        # The autonomy topic is fixed at startup (a publisher can't change topics later)
        topic = self.get_parameter('autonomy_command_topic').value
        if topic:
            self.autonomy_pub = self.create_publisher(String, topic, 10)
            self.get_logger().info(f'Select button hands off to: {topic}')
        else:
            self.autonomy_pub = None
            self.get_logger().info('No autonomy_command_topic set: Select button disabled')

        # Watchdog: force a hover if the controller stops sending /joy
        self.timeout = 2.0
        self.last_joy_time = None
        self.signal_lost = False
        self.create_timer(0.5, self.watchdog)

    def on_command(self, msg):
        """Handle a message on /controller_input/command.

        Expected format (std_msgs/String, case and surrounding spaces ignored):
            "activate"    start listening to the controller (standby)
            "deactivate"  stop listening; releases control first if held

        e.g. ros2 topic pub --once /controller_input/command std_msgs/String "{data: 'activate'}"
        """
        command = msg.data.strip().lower()
        if not command:
            return

        if command == self.ACTIVATE:
            self.activate()
        elif command == self.DEACTIVATE:
            self.deactivate()
        else:
            self.get_logger().warn(f'Unrecognized command: {msg.data}')



    def on_owner(self, msg):
        """Drop back to standby if another node claims /cmd_effort."""
        if self.in_control and msg.data != self.NAME:
            self.in_control = False
            self.get_logger().info(f'"{msg.data}" controller took over. Manual back to standby')
            self.status_pub.publish(String(data='active'))



    def activate(self):
        """Start listening to the controller (standby; deadman still needed to drive)."""
        if self.active:
            self.get_logger().info("Manual Controller already active. Ignoring command.")
            return
        self.active = True
        # Give the watchdog a fresh start so it doesn't trip immediately
        self.last_joy_time = self.get_clock().now()
        self.signal_lost = False
        self.get_logger().info("Manual Controller: ACTIVE (hold deadman to take control)")
        self.status_pub.publish(String(data='active'))



    def deactivate(self, announce=True):
        """Stop listening to the controller, releasing control first if we hold it."""
        if not self.active:
            self.get_logger().info("Manual Controller already deactive. Ignoring command")
            return
        if self.in_control:
            self.release_control()
        self.active = False
        self.get_logger().info("Manual Controller: INACTIVE")
        self.status_pub.publish(String(data='inactive'))



    def take_control(self):
        """Claim /cmd_effort. Any active autonomy node will see this and stand down."""
        self.in_control = True
        self.owner_pub.publish(String(data=self.NAME))
        self.status_pub.publish(String(data='in_control'))



    def release_control(self):
        """Send one zero-effort command and give up /cmd_effort (owner -> '')."""
        self.in_control = False
        self.cmd_pub.publish(Twist())
        self.owner_pub.publish(String(data=''))
        self.get_logger().info('Manual control released')
        self.status_pub.publish(String(data='active'))



    def apply_deadzone(self, value, threshold=0.1):
        """Treat small stick deflections as 0 so a resting stick doesn't drift the sub."""
        return value if abs(value) > threshold else 0.0


    def on_joy(self, msg):
        """Handle one /joy message: autonomy button, deadman/release, and driving."""
        if not self.active:
            return
        # Ignore messages from a controller with fewer axes/buttons than we index
        if len(msg.axes) < 6 or len(msg.buttons) < 11:
            return

        self.last_joy_time = self.get_clock().now()
        if self.signal_lost:
            self.signal_lost = False
            self.get_logger().info("Controller signal restored")

        # Only act on the moment the autonomy button goes down (rising edge),
        # not on every message while it is held
        autonomy_now = msg.buttons[self.AUTONOMY_BUTTON]
        autonomy_pressed = autonomy_now and not self.prev_autonomy
        self.prev_autonomy = autonomy_now

        if autonomy_pressed:
                    if self.in_control:
                        self.get_logger().info('Manual control still active. Must release to start autonomous control')
                    elif self.autonomy_pub is None:
                        self.get_logger().warn('No autonomy_command_topic set. Select button does nothing.')
                    else:
                        self.autonomy_pub.publish(String(data='activate'))
                        self.get_logger().info('Autonomy requested')

        deadman = msg.buttons[self.DEADMAN_BUTTON]

        # Pressing the deadman always overrides whoever is in control
        if deadman and not self.in_control:
            self.get_logger().warn('MANUAL OVERIDE: deadman pressed')
            self.take_control()

        if not self.in_control:
            return

        if msg.buttons[self.RELEASE_BUTTON]:
            self.release_control()
            return

        # In control but deadman let go: keep ownership and hover
        if not deadman:
            self.cmd_pub.publish(Twist())
            return


        twist = Twist()

        twist.linear.x = self.apply_deadzone(msg.axes[1]) # Forward and back
        twist.linear.y = self.apply_deadzone(msg.axes[0]) # left and right

        # Triggers rest at +1.0 and read -1.0 when fully pulled; map each to 0..1
        left_trigger = (1.0 - msg.axes[4]) / 2.0
        right_trigger = (1.0 - msg.axes[5]) / 2.0
        twist.linear.z = right_trigger - left_trigger # up and down

        twist.angular.z = self.apply_deadzone(msg.axes[2])  # yaw

        self.cmd_pub.publish(twist)

    def watchdog(self):
        """Every 0.5 s: if /joy has gone quiet, take control and hover (failsafe)."""
        if not self.active or self.signal_lost:
            return
        elapsed = (self.get_clock().now() - self.last_joy_time ).nanoseconds * 1e-9
        if elapsed > self.timeout:
            self.signal_lost = True
            self.get_logger().warn(f'No controller signal for {elapsed:.1f} s. Forcing hover.')
            self.take_control()
            self.cmd_pub.publish(Twist())

def main(args=None):
    """Entry point for 'ros2 run sub_py controller_input'."""
    rclpy.init(args=args)
    node = ControllerInput()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
