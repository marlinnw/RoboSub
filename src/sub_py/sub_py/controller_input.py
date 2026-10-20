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



    def __init__(self):
        super().__init__('controller_input')

        self.active = False

        self.command_sub        = self.create_subscription(String, '/controller_input/command', self.on_command, 10)
        self.status_pub         = self.create_publisher(String, '/controller_input/status', 10)
        self.owner_sub          = self.create_subscription(String, '/control_command/active', self.on_owner, LATCHED)
        self.owner_pub          = self.create_publisher(String, '/control_command/active', LATCHED)

        self.controller_input   = self.create_subscription(Joy, '/joy', self.on_joy, 10)

        self.cmd_pub            = self.create_publisher(Twist, '/cmd_effort', 10)


    def on_command(self, msg):
        command = msg.data.strip().lower()
        if not command:
            return
        
        if command == self.ACTIVATE:
            self.activate()
        elif comand == self.DEACTIVATE:
            self.deactivate()
        else:
            self.get_logger().warn(f'Unrecognized command: {msg.data}')



    def on_owner(self, msg):
        if self.active and msg.data != self.NAME:
            self.get_logger().info(f'"{msg.data}" controller took over. Deactivating')
            self.deactivate(annonce=False)



    def activate(self):
        if self.active:
            self.get_logger().info("Manual Controller already active. Ignoring command.")
            return
        self.active = True
        self.owner_pub.publish(String(data=self.NAME))
        self.get_logger().info("Manual Controller: ACTIVE")
        self.status_pub.publish(String(data='active'))



    def deactivate(self, announce=True):
        if not self.active:
            self.get_logger().info("Manual Controller already deactive. Ignoring command")
            return
        self.active = False
        if annonce:
            self.owner_pub.publish(String(data=''))
        self.get_logger().info("Manual Controller: INACTIVE")
        self.status_pub.publish(String(data='inactive'))



    def apply_deadzone(self, value, threshold=0.1):
        return value if abs(value) > threshold else 0.0


    def on_joy(self, msg):
        if not self.active:
            return
        
        if len(msg.axes) < 8 or len(msg.buttons) < 11:
            return

        twist = Twist()

        twist.linear.x = self.apply_deadzone(msg.axes[1])
        twist.linear.y = self.apply_deadzone(msg.axes[0])

        left_trigger = (1.0 - msg.axes[2]) / 2.0
        right_trigger = (1.0 - msg.axes[5]) / 2.0
        twist.linear.z = right_trigger - left_trigger

        twist.angular.x = self.apply_deadzone(msg.axes[3])
        twist.angular.y = self.apply_deadzone(msg.axes[4])

        left_bumper = mg.buttons[4]
        right_bumber = msg.buttons[5]
        twist.angular.z = float(right_bumber - left_bumper)

        self.cmd_pub.publish(twist)


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
