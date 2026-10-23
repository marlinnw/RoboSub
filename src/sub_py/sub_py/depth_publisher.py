"""depth_publisher.py -- a fake depth sensor

What this node does
-------------------
Every 0,1 s (10 Hz) it makes up a depth reading that slowly bobs between
1.5 m and 2.5 m (a sine wave) and PUBLISHES it on the topic /depth

Any other node --Python or C++, on this machine or across the network 
can SUBSCRIBE to /depth and recieve these numbers. This node does not 
know or care who is listening or whether anyone is listening at all.
"""

# This is a test of git

import math

# Ros2 library for python
import rclpy
# Allows for external shutdown
from rclpy.executors import ExternalShutdownException
# Python node inherits from
from rclpy.node import Node
# message type holding one 64-bit float
from std_msgs.msg import Float64

class DepthPublisher(Node):
    """
    A ROS2 node that publishes a simulated depth reading at 10Hz.

    Inheriting from Node gives this class everything it needs to talk to 
    ROS: a name, a logger, and methods such as create_publisher() and 
    create_timer()
    """

    def __init__(self):
        # Register this node with ros under the name 'depth_publisher
        # This is the name shown by 'ros2 node list' and in rqt_graph
        super().__init__('depth_publisher')

        # Create a PUBLISHER
        # ------------------
        # This node will send Float64 messages on topic "depth"
        # Float64 -> message type;
            # PUBLISHER and subscribers must agree on it
        # 'depth' -> TOPIC name:
            # ROS turns it into /depth
        # 10 -> que depth:
            # if the network falls behine, hold up to 10 outgoing messages
            # then start dropping the oldest messages
        # We keep the reuturned object in self.pub so we can call .publish later
        self.pub = self.create_publisher(Float64, 'depth', 10)

        # Simulated time in seconds
        self.t = 0.0

        # Create a timer: ask ROS to call self.tick() every 0.1 s.
        # Nothing happens yet. Timers only fire once the node is "spun"
        self.timer = self.create_timer(0.1, self.tick)

    def tick(self):
        """
        Timer callback. ROS calls this every 0.1 s.
        
        Builds one Float64 message containing a fake depth and publishes it
        """    

        self.t += 1

        # Create an empty message object
        msg= Float64()

        # Fill in the the message data
        msg.data = 2.0 +0.5*math.sin(self.t)

        # Hand over the message to ROS
        self.pub.publish(msg)

        # Print to the terminal. this also goes to ROS log system,
        # where rqt_console can filter it by node and severity
        self.get_logger().info(f'depth: {msg.data: .2f} m')

def main(args=None):
    """
    Program entry point: 'ros2 run sub_py depth_publisher' calls this
    """
    # Step 1: start the ROS2 client library (always first)
    rclpy.init(args=None)

    # Step 2: build the node -> publisher and timer are registered
    node = DepthPublisher()

    # Step 3: Hand control to ROS. It loops forever, sleeping 
    # until the timer is due, then calling tick
    try:
        rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass # Ctrl+C lands here: this is the normal way to stop a node
    finally:
        node.destroy_node() # unregister the node cleanly
        rclpy.try_shutdown() # shutdown the client library
