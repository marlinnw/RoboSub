import rclpy
from rclpy.node import Node
from rclpy.ExternalShutdownException import ExternalShutdownException
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class ControllerInput(Node):

    def __init__(self):
        super().__init__('controller_input'):

        self.command_sub    = create_subscription(String, '/controller_input/command', 10)




def main(args=None):
    rclpy.init(args=args)
    node = ControllerInput()

    try:
        rclpy.spin(node):
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
