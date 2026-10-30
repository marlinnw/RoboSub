"""lab_filter.py -- keeps only pixels of a chosen color, using the LAB color space.

Each camera pixel is converted to LAB. The a (green-red) and b (blue-yellow)
channels are treated as a 2D color plane:
    hue    = angle around that plane, in degrees [0, 360)
    chroma = distance from the center (how saturated / "colorful" it is)
Pixels whose hue falls in [hue_min, hue_max] and whose chroma is at least
chroma_min are kept; everything else is set to black. Small specks are
removed and small holes filled with a morphological open/close.

Only one color filter should feed /color_filter/image_filtered at a time.
Ownership is negotiated on the latched topic /color_filter/active, which
holds the NAME of the active filter ('lab') or '' when none is active.

COMMAND  /color_filter/lab/command  std_msgs/String  "activate" / "deactivate"
INPUT    /camera/image_raw          sensor_msgs/Image
OUTPUT   /color_filter/image_filtered             sensor_msgs/Image
         /color_filter/image_filtered/compressed  sensor_msgs/CompressedImage  (only when someone subscribes)
         /color_filter/lab/status   std_msgs/String  "active" / "inactive"
BOTH     /color_filter/active       std_msgs/String  (latched) current filter owner

Deactivates itself if no camera images arrive for 2 s.

Examples:
  ros2 topic pub --once /color_filter/lab/command std_msgs/String "{data: 'activate'}"
  ros2 param set /lab_filter hue_min 40
"""

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

# "Latched" QoS: late subscribers still receive the last message published
LATCHED = QoSProfile(depth=1,
                     durability = DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability = ReliabilityPolicy.RELIABLE)



def slider(low, high):
    """Gives a parameter an allowed range. ROS rejects values outside it,
    and rqt_reconfigure shows the parameter as a slider."""
    return ParameterDescriptor(integer_range=[IntegerRange(from_value=low, to_value=high, step=1)])



class Lab_Filter(Node):
    """Color filter node: LAB hue/chroma threshold on the camera image."""

    NAME = 'lab'                # how this filter identifies itself on /color_filter/active
    WINDOW = 'LAB Filter'
    ACTIVATE = "activate"
    DEACTIVATE = "deactivate"



    def __init__(self):
        super().__init__('lab_filter')

        # Hue range in degrees. If hue_min > hue_max the range wraps through 0/360.
        self.declare_parameter('hue_min', 36, slider(0, 360))
        self.declare_parameter('hue_max', 82, slider(0, 360))
        # Minimum chroma: raise it to reject grayish / washed-out pixels
        self.declare_parameter('chroma_min', 0, slider(0, 150))
        self.declare_parameter('jpeg_quality', 70, slider(0, 100))
        self.declare_parameter('show_window', False)        # true on a desk, false on the Jetson

        self.active = False
        self.show_window = False        # snapshot of the show_window parameter taken at activate()
        self.bridge = CvBridge()

        self.command_sub                = self.create_subscription(String, '/color_filter/lab/command', self.on_command, 10)
        self.image_sub                  = self.create_subscription(Image, '/camera/image_raw', self.on_img, 10)
        self.status_pub                 = self.create_publisher(String, '/color_filter/lab/status', 10)
        self.filtered_pub               = self.create_publisher(Image, '/color_filter/image_filtered', 10)
        self.filtered_pub_compressed    = self.create_publisher(CompressedImage, '/color_filter/image_filtered/compressed', 10)
        self.owner_pub                  = self.create_publisher(String, '/color_filter/active', LATCHED)
        self.owner_sub                  = self.create_subscription(String, '/color_filter/active', self.on_owner, LATCHED)

        # Watchdog: deactivate if the camera stops sending images
        self.timeout = 2.0
        self.last_img_time = None
        self.create_timer(0.5, self.watchdog)



    def p(self, name):
        """Current value of a parameter (read every time so live changes work)."""
        return self.get_parameter(name).value



    def on_command(self, msg):
        """Handle a message on /color_filter/lab/command.

        Expected format (std_msgs/String, case and surrounding spaces ignored):
            "activate"    claim /color_filter/active and start filtering
            "deactivate"  stop filtering and release /color_filter/active

        e.g. ros2 topic pub --once /color_filter/lab/command std_msgs/String "{data: 'activate'}"
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
        """Step aside if a different color filter announces itself as active."""
        if self.active and msg.data != self.NAME:
            self.get_logger().info(f'"{msg.data}" filter took over. DEACTIVATING.')
            # The new filter already claimed the topic, so don't clear it
            self.deactivate(announce=False)



    def activate(self):
        """Claim /color_filter/active and start filtering camera images."""
        if self.active:
            self.get_logger().info('LAB filter already activated. Ignoring command')
            return

        self.show_window = self.p('show_window')    # read once; a change applies on the next activate

        if self.show_window:
            cv.namedWindow(self.WINDOW)

        # Give the watchdog a fresh start so it doesn't trip immediately
        self.last_img_time = self.get_clock().now()
        self.active = True
        self.owner_pub.publish(String(data=self.NAME))
        self.get_logger().info("LAB filter: ACTIVE")
        self.status_pub.publish(String(data='active'))



    def deactivate(self, announce=True):
        """Stop filtering.

        announce=True (normal stop) clears /color_filter/active. announce=False
        is used when another filter has taken over, so its claim isn't erased.
        """
        if not self.active:
            self.get_logger().info("LAB filter already deactivated. Ignoring command")
            return
        self.active = False
        if self.show_window:
            cv.destroyAllWindows()
            cv.waitKey(1)           # let OpenCV process the close event
        if announce:
            self.owner_pub.publish(String(data=''))
        self.get_logger().info("LAB filter: INACTIVE")
        self.status_pub.publish(String(data='inactive'))



    def watchdog(self):
        """Every 0.5 s: deactivate if no camera image has arrived within the timeout."""
        if not self.active:
            return
        elapsed = (self.get_clock().now() - self.last_img_time).nanoseconds * 1e-9
        if elapsed > self.timeout:
            self.get_logger().warn(f'No images for {elapsed:.1f} s.')
            self.deactivate()


    #----Image processing----

    def on_img(self, msg):
        """Filter one camera frame and publish the result."""
        if not self.active:
            return
        self.last_img_time = self.get_clock().now()
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        lab = cv.cvtColor(frame, cv.COLOR_BGR2LAB)

        # OpenCV stores 8-bit a and b offset by 128; shift so 0 = neutral gray
        a = lab[..., 1].astype(np.float32) - 128
        b = lab[..., 2].astype(np.float32) - 128

        # Polar coordinates on the a/b plane
        c = np.hypot(a, b)                              # chroma
        hue = np.degrees(np.arctan2(b,a)) % 360         # hue angle in [0, 360)

        hue_min = self.p('hue_min')
        hue_max = self.p('hue_max')
        if hue_min <= hue_max:
            hue_ok = (hue >= hue_min) & (hue <= hue_max)
        else:
            # Range wraps around 0/360 (e.g. 330 -> 20 for reds)
            hue_ok = (hue >= hue_min) | (hue <= hue_max)

        mask = (hue_ok & (c >= self.p('chroma_min'))).astype(np.uint8) * 255
        # Open = erase isolated specks, Close = fill small gaps inside blobs
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, np.ones((5, 5), np.uint8))
        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, np.ones((5, 5), np.uint8))

        # Original colors where the mask is set, black everywhere else
        filtered = cv.bitwise_and(frame, frame, mask=mask)

        # Always publish the raw filtered image: rail_detector depends on it
        out = self.bridge.cv2_to_imgmsg(filtered, encoding='bgr8')
        out.header = msg.header                 # keep the camera's capture time
        self.filtered_pub.publish(out)

        if self.show_window:
            cv.imshow(self.WINDOW, filtered)
            cv.waitKey(1)

        # Compressed copy for viewing over the tether -- only if someone is watching
        if self.filtered_pub_compressed.get_subscription_count() > 0:
            ok, jpg = cv.imencode('.jpg', filtered, [cv.IMWRITE_JPEG_QUALITY, self.p('jpeg_quality')])
            if ok:
                comp = CompressedImage()
                comp.header = out.header
                comp.format = 'jpeg'
                comp.data = jpg.tobytes()
                self.filtered_pub_compressed.publish(comp)


def main(args=None):
    """Entry point for 'ros2 run sub_py lab_filter'."""
    rclpy.init(args=args)
    node = Lab_Filter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
