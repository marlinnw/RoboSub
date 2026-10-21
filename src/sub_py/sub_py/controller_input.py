import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

LATCHED = QoSProfile(depth=1,
                     durability = DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability = ReliabilityPolicy.RELIABLE)


class ControllerInput(Node):

    NAME = 'manual'
    ACTIVATE = 'activate'
    DEACTIVATE = 'deactivate'
    DEADMAN_BUTTON = 9
    RELEASE_BUTTON = 6



    def __init__(self):
        super().__init__('controller_input')

        self.active = False
        self.in_control = False

        self.command_sub        = self.create_subscription(String, '/controller_input/command', self.on_command, 10)
        self.status_pub         = self.create_publisher(String, '/controller_input/status', 10)
        self.owner_sub          = self.create_subscription(String, '/control_command/active', self.on_owner, LATCHED)
        self.owner_pub          = self.create_publisher(String, '/control_command/active', LATCHED)
        self.joy_sub            = self.create_subscription(Joy, '/joy', self.on_joy, 10)
        self.cmd_pub            = self.create_publisher(Twist, '/cmd_effort', 10)

        self.timeout = 2.0
        self.last_joy_time = None
        self.signal_lost = False
        self.create_timer(0.5, self.watchdog)

    def on_command(self, msg):
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
        if self.in_control and msg.data != self.NAME:
            self.in_control = False
            self.get_logger().info(f'"{msg.data}" controller took over. Manual back to standby')
            self.status_pub.publish(String(data='active'))



    def activate(self):
        if self.active:
            self.get_logger().info("Manual Controller already active. Ignoring command.")
            return
        self.active = True
        self.last_joy_time = self.get_clock().now()
        self.signal_lost = False
        self.get_logger().info("Manual Controller: ACTIVE (hold deadman to take control)")
        self.status_pub.publish(String(data='active'))



    def deactivate(self, announce=True):
        if not self.active:
            self.get_logger().info("Manual Controller already deactive. Ignoring command")
            return
        if self.in_control:
            self.release_control()
        self.active = False
        self.get_logger().info("Manual Controller: INACTIVE")
        self.status_pub.publish(String(data='inactive'))



    def take_control(self):
        self.in_control = True
        self.owner_pub.publish(String(data=self.NAME))
        self.status_pub.publish(String(data='in_control'))



    def release_control(self):
        self.in_control = False
        self.cmd_pub.publish(Twist())
        self.owner_pub.publish(String(data=''))
        self.get_logger().info('Manual control released')
        self.status_pub.publish(String(data='active'))



    def apply_deadzone(self, value, threshold=0.1):
        return value if abs(value) > threshold else 0.0


    def on_joy(self, msg):
        if not self.active:
            return
        if len(msg.axes) < 6 or len(msg.buttons) < 11:
            return

        self.last_joy_time = self.get_clock().now()
        if self.signal_lost:
            self.signal_lost = False
            self.get_logger().info("Controller signal restored")

        deadman = msg.buttons[self.DEADMAN_BUTTON]

        if deadman and not self.in_control:
            self.get_logger().warn('MANUAL OVERIDE: deadman pressed')
            self.take_control()

        if not self.in_control:
            return

        if msg.buttons[self.RELEASE_BUTTON]:
            self.release_control()
            return

        if not deadman:
            self.cmd_pub.publish(Twist())
            return
        

        twist = Twist()

        twist.linear.x = self.apply_deadzone(msg.axes[1]) # Forward and back
        twist.linear.y = self.apply_deadzone(msg.axes[0]) # left and right

        left_trigger = (1.0 - msg.axes[4]) / 2.0
        right_trigger = (1.0 - msg.axes[5]) / 2.0
        twist.linear.z = right_trigger - left_trigger # up and down

        twist.angular.z = self.apply_deadzone(msg.axes[2])

        self.cmd_pub.publish(twist)

    def watchdog(self):
        if not self.active or self.signal_lost:
            return
        elapsed = (self.get_clock().now() - self.last_joy_time ).nanoseconds * 1e-9
        if elapsed > self.timeout:
            self.signal_lost = True
            self.get_logger().warn(f'No controller signal for {elapsed:.1f} s. Forcing hover.')
            self.take_control()
            self.cmd_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = ControllerInput()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
