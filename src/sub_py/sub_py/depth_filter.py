"""depth_filter.py -- smooths the /depth readings with a low-pass filter.

Uses an exponential moving average:
    y = alpha * new_reading + (1 - alpha) * y
alpha near 1 follows the raw data closely; alpha near 0 is very smooth but
reacts slowly. The first reading is used as the starting value.

INPUT    /depth               std_msgs/Float64
OUTPUT   /depth_filteredPY    std_msgs/Float64

Parameters:
    alpha  0..1 smoothing factor (read once at startup)
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float64

class FilterDepthPY(Node):
    """Exponential-moving-average filter from /depth to /depth_filteredPY."""

    def __init__(self):
        super().__init__('depth_filterPY')

        self.declare_parameter('alpha', 0.02)

        self.alpha = self.get_parameter('alpha').value
        self.initialized = False        # becomes True once the first reading arrives
        self.y = 0.0                    # current filtered value

        self.sub = self.create_subscription(Float64, 'depth', self.on_depth, 10)
        self.pub = self.create_publisher(Float64, 'depth_filteredPY', 10)


    def on_depth(self, msg):
        """Update the filter with one reading and publish the filtered value."""
        global initialized
        if not self.initialized:
            # Seed the filter with the first reading so it doesn't start from 0
            self.y = msg.data
            self.initialized = True

        self.y = self.alpha * msg.data + (1 - self.alpha) * self.y

        self.pub.publish(Float64(data=self.y))

def main(args=None):
    """Entry point for 'ros2 run sub_py depth_filterPY'."""
    rclpy.init(args=args)
    node = FilterDepthPY()

    try:
        rclpy.spin(node)
    except(KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
