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
from geometry_msgs.msg import PointStamped

LATCHED = QoSProfile(depth=1,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)


def slider(low, high):
    """Gives a parameter an allowed range. ROS rejects values outside it,
    and rqt_reconfigure shows the parameter as a slider."""
    return ParameterDescriptor(integer_range=[IntegerRange(from_value=low, to_value=high, step=1)])


class RailDetector(Node):

    WINDOW = 'Rail Detector'
    ACTIVATE = 'activate'
    DEACTIVATE = 'deactivate'

    def __init__(self):
        super().__init__('rail_detector')

        # Tune live with:  ros2 param set /rail_detector min_area 3000   (or rqt_reconfigure)
        self.declare_parameter('min_area', 2614, slider(0, 20000))
        self.declare_parameter('max_area', 30000, slider(20000, 76800))   # 76800 = 320 x 240
        self.declare_parameter('jpeg_quality', 70, slider(1, 100))
        self.declare_parameter('show_window', False)                      # true on a desk, false on the Jetson

        self.active = False
        self.show_image = False
        self.bridge = CvBridge()
        self.source = ''

        self.command_sub              = self.create_subscription(String, '/rail_detector/command', self.on_command, 10)
        self.image_sub                = self.create_subscription(Image, '/color_filter/image_filtered', self.on_img, 10)
        self.owner_sub                = self.create_subscription(String, '/color_filter/active', self.on_owner, LATCHED)
        self.status_pub               = self.create_publisher(String, '/rail_detector/status', 10)
        self.target_pub               = self.create_publisher(PointStamped, '/rail_detector/target', 10)
        self.annotated_pub            = self.create_publisher(Image, '/rail_detector/image_annotated', 10)
        self.annotated_pub_compressed = self.create_publisher(CompressedImage, '/rail_detector/image_annotated/compressed', 10)

        self.timeout = 2.0
        self.last_img_time = None
        self.create_timer(0.5, self.watchdog)

    def p(self, name):
        return self.get_parameter(name).value

    # ---------- commands ----------

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
        self.source = msg.data
        self.get_logger().info(f'Filter source: {self.source or "none"}')

    def activate(self):
        if self.active:
            self.get_logger().info('Rail detector already active. Ignoring command.')
            return

        self.show_image = self.p('show_window')     # read once; a change applies on the next activate
        if self.show_image:
            cv.namedWindow(self.WINDOW)

        self.last_img_time = self.get_clock().now()
        self.active = True
        self.get_logger().info(f'Rail detector: ACTIVE (filter source: {self.source or "none"})')
        self.status_pub.publish(String(data='active'))

    def deactivate(self):
        if not self.active:
            self.get_logger().info('Rail detector already inactive. Ignoring command.')
            return
        self.active = False
        if self.show_image:
            cv.destroyAllWindows()
            cv.waitKey(1)
        self.get_logger().info('Rail detector: INACTIVE')
        self.status_pub.publish(String(data='inactive'))

    def watchdog(self):
        if not self.active:
            return
        elapsed = (self.get_clock().now() - self.last_img_time).nanoseconds * 1e-9
        if elapsed > self.timeout:
            self.get_logger().warn(f'No filtered images for {elapsed:.1f} s. Deactivating.')
            self.deactivate()

    # ---------- detection ----------

    def on_img(self, msg):
        if not self.active:
            return

        self.last_img_time = self.get_clock().now()

        filtered = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Any non-black pixel = something the color filter kept
        mask = np.any(filtered > 0, axis=2).astype(np.uint8) * 255

        min_area = self.p('min_area')
        max_area = self.p('max_area')
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        blobs = [c for c in contours if min_area <= cv.contourArea(c) <= max_area]

        annotated = filtered.copy()
        cv.drawContours(annotated, blobs, -1, (0, 255, 0), 2)
        for c in blobs:                                            # each blob: its own area
            x, y, w, h = cv.boundingRect(c)
            cv.putText(annotated, f'{int(cv.contourArea(c))}', (x, y - 5),
                       cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv.LINE_AA)

        target = PointStamped()
        target.header = msg.header                                 # keep the camera's capture time

        if blobs:
            biggest = max(blobs, key=cv.contourArea)
            m = cv.moments(biggest)
            cx = m['m10'] / m['m00']
            cy = m['m01'] / m['m00']

            # Long-axis direction from the 2nd-order moments
            theta = 0.5 * np.arctan2(2 * m['mu11'], m['mu20'] - m['mu02'])
            vx, vy = np.cos(theta), np.sin(theta)
            if vy > 0:                                             # point UP the image (toward the sub's nose)
                vx, vy = -vx, -vy
            angle = float(np.degrees(np.arctan2(vx, -vy)))         # 0 = straight up, + = top leans right

            cv.circle(annotated, (int(cx), int(cy)), 6, (0, 0, 255), -1)
            cv.line(annotated, (int(cx - 80 * vx), int(cy - 80 * vy)),
                               (int(cx + 80 * vx), int(cy + 80 * vy)), (255, 0, 0), 2)
            cv.putText(annotated, f'{angle:.1f} deg', (int(cx) + 10, int(cy)),
                       cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv.LINE_AA)

            target.point.x = cx
            target.point.y = cy
            target.point.z = angle
        else:
            # NaN = "detector is running but sees no rail" -> rail follower hovers
            target.point.x = float('nan')
            target.point.y = float('nan')
            target.point.z = float('nan')

        self.target_pub.publish(target)

        # Raw annotated image -- only if someone subscribes (bag recording, local viewer)
        if self.annotated_pub.get_subscription_count() > 0:
            out = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            out.header = msg.header
            self.annotated_pub.publish(out)

        # Compressed copy for viewing over the tether -- only if someone is watching
        if self.annotated_pub_compressed.get_subscription_count() > 0:
            ok, jpg = cv.imencode('.jpg', annotated, [cv.IMWRITE_JPEG_QUALITY, self.p('jpeg_quality')])
            if ok:
                comp = CompressedImage()
                comp.header = msg.header
                comp.format = 'jpeg'
                comp.data = jpg.tobytes()
                self.annotated_pub_compressed.publish(comp)

        if self.show_image:
            cv.imshow(self.WINDOW, annotated)
            cv.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = RailDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()