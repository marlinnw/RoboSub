import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float64

class FilterDepthPY(Node):

    def __init__(self):
        super().__init__('depth_filterPY')

        self.declare_parameter('alpha', 0.02)

        self.alpha = self.get_parameter('alpha').value
        self.initialized = False
        self.y = 0.0

        self.sub = self.create_subscription(Float64, 'depth', self.on_depth, 10)
        self.pub = self.create_publisher(Float64, 'depth_filteredPY', 10)


    def on_depth(self, msg):
        global initialized
        if not self.initialized:
            self.y = msg.data
            self.initialized = True

        self.y = self.alpha * msg.data + (1 - self.alpha) * self.y

        self.pub.publish(Float64(data=self.y))

def main(args=None):
    rclpy.init(args=args)
    node = FilterDepthPY()

    try:
        rclpy.spin(node)
    except(KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
