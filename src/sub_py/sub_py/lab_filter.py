import cv2 as cv
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor, IntegerRange
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String

LATCHED = QoSProfile(depth=1,
                     durability = DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability = ReliabilityPolicy.RELIABLE)



def slider(low, high):
    return ParameterDescriptor(integer_range=[IntegerRange(from_value=low, to_value=high, step=1)])



class Lab_Filter(Node):

    NAME = 'lab'
    WINDOW = 'LAB Filter'
    ACTIVATE = "activate"
    DEACTIVATE = "deactivate"



    def __init__(self):
        super().__init__('lab_filter')

        self.declare_parameter('hue_min', 36, slider(0, 360))
        self.declare_parameter('hue_max', 82, slider(0, 360))
        self.declare_parameter('chroma_min', 0, slider(0, 150))
        self.declare_parameter('jpeg_quality', 70, slider(0, 100))
        self.declare_parameter('show_window', False)

        self.active = False
        self.show_window = False
        self.bridge = CvBridge()

        self.command_sub                = self.create_subscription(String, '/color_filter/lab/command', self.on_command, 10)
        self.image_sub                  = self.create_subscription(Image, '/camera/image_raw', self.on_img, 10)
        self.status_pub                 = self.create_publisher(String, '/color_filter/lab/status', 10)
        self.filtered_pub               = self.create_publisher(Image, '/color_filter/image_filtered', 10)
        self.filtered_pub_compressed    = self.create_publisher(CompressedImage, '/color_filter/image_filtered/compressed', 10)
        self.owner_pub                  = self.create_publisher(String, '/color_filter/active', LATCHED)
        self.owner_sub                  = self.create_subscription(String, '/color_filter/active', self.on_owner, LATCHED)

        self.timeout = 2.0
        self.last_img_time = None
        self.create_timer(0.5, self.watchdog)



    def p(self, name):
        return self.get_parameter(name).value



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
        if self.active and msg.data != self.NAME:
            self.get_logger().info(f'"{msg.data}" filter took over. DEACTIVATING.')
            self.deactivate(announce=False)



    def activate(self):
        if self.active:
            self.get_logger().info('LAB filter already activated. Ignoring command')
            return

        self.show_window = self.p('show_window')

        if self.show_window:
            cv.namedWindow(self.WINDOW)
        
        self.last_img_time = self.get_clock().now()
        self.active = True
        self.owner_pub.publish(String(data=self.NAME))
        self.get_logger().info("LAB filter: ACTIVE")
        self.status_pub.publish(String(data='active'))



    def deactivate(self, announce=True):
        if not self.active:
            self.get_logger().info("LAB filter already deactivated. Ignoring command")
            return
        self.active = False
        if self.show_window:
            cv.destroyAllWindows()
            cv.waitKey(1)
        if announce:
            self.owner_pub.publish(String(data=''))
        self.get_logger().info("LAB filter: INACTIVE")
        self.status_pub.publish(String(data='inactive'))



    def watchdog(self):
        if not self.active:
            return
        elapsed = (self.get_clock().now() - self.last_img_time).nanoseconds * 1e-9
        if elapsed > self.timeout:
            self.get_logger().warn(f'No images for {elapsed:.1f} s.')
            self.deactivate()


    #----Image processing----
        
    def on_img(self, msg):
        if not self.active:
            return
        self.last_img_time = self.get_clock().now()
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        lab = cv.cvtColor(frame, cv.COLOR_BGR2LAB)

        a = lab[..., 1].astype(np.float32) - 128
        b = lab[..., 2].astype(np.float32) - 128

        c = np.hypot(a, b)
        hue = np.degrees(np.arctan2(b,a)) % 360

        hue_min = self.p('hue_min')
        hue_max = self.p('hue_max')
        if hue_min <= hue_max:
            hue_ok = (hue >= hue_min) & (hue <= hue_max)     
        else:
            hue_ok = (hue >= hue_min) | (hue <= hue_max) 

        mask = (hue_ok & (c >= self.p('chroma_min'))).astype(np.uint8) * 255
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, np.ones((5, 5), np.uint8))
        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, np.ones((5, 5), np.uint8))

        filtered = cv.bitwise_and(frame, frame, mask=mask)

        out = self.bridge.cv2_to_imgmsg(filtered, encoding='bgr8')
        out.header = msg.header
        self.filtered_pub.publish(out)

        if self.show_window:
            cv.imshow(self.WINDOW, filtered)
            cv.waitKey(1)

        if self.filtered_pub_compressed.get_subscription_count() > 0:
            ok, jpg = cv.imencode('.jpg', filtered, [cv.IMWRITE_JPEG_QUALITY, self.p('jpeg_quality')])
            if ok:
                comp = CompressedImage()
                comp.header = out.header
                comp.format = 'jpeg'
                comp.data = jpg.tobytes()
                self.filtered_pub_compressed.publish(comp)


def main(args=None):
    rclpy.init(args=args)
    node = Lab_Filter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()