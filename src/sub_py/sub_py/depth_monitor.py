"""
depth_monitor.py -- watches /depth and warns when the sub goes too deep

How it works
------------
SUBSCRIBES to /depth. Every time a new reading arrives, ROS calls on_depth() with that message.
If the depth is past a limit, it logs a warning.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64

class DepthMonitor(Node):
    """Logs a warning whenever a /depth reading exceeds MAX_DEPTH."""

    MAX_DEPTH = 2.4     # meters

    def __init__(self):
        super().__init__('depth_monitor')

        # Create a SUBSCRIPTION: 
        # whenever a Float64 message arrives on '/depth', call self.on_depth(message)

        # Float64 -> expected message type (must match the publisher)
        # 'depth' -> topic name to lister on (/depth)
        # self.on_depth -> the callback function ROS with call for each message
        # 10 -> que depth: buffer up to 10 incoming messages:

        self.sub = self.create_subscription(Float64, 'depth', self.on_depth, 10)

    def on_depth(self, msg):
        """Subscription callback: ROS calls this once per /depth message."""
        if msg.data > self.MAX_DEPTH:
            self.get_logger().warn(f'too deep: {msg.data: .2f} m')

def main(args=None):
    """Entry point for 'ros2 run sub_py depth_monitor'."""
    rclpy.init(args=args)
    node = DepthMonitor()
    try:
        rclpy.spin(node)
    except(KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
