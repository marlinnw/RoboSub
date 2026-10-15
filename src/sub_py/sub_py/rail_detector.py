import cv2 as cv
import numpy as np  
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped

LATCHED = QoSProfile(depth = 1,
                     durability = DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability = ReliabilityPolicy.RELIABLE)

class RailDetector(Node):

    ACTIVATE = 'activate'
    DEACTIVATE = 'deactivate'

    def __init__(self):
        super().__init__('rail_detector')

        self.active = False
        self.bridge = CvBridge()
        self.source = ''

        self.command_sub    = self.create_subscription(String, '/rail_detector/command', self.on_command, 10)
        self.image_sub      = self.create_subscription(Image, 'color_filter/image_filtered', self.on_img, 10)
        self.owner_sub      = self.create_subscription(String, 'color_filter/active', self.on_owner, LATCHED)
        self.status_pub     = self.create_publisher(String, '/rail_detector/status', 10)
        self.target_pub     = self.create_publisher(PointStamped, '/rail_detector/target', 10)
        self.annotated_pub  = self.create_publisher(Image, '/rail_detector/image_annotated', 10)

        self.timeout = 2.0
        self.last_img_time = None
        self.create_timer(2.0, self.watchdog)

    def on_command(self, msg):
        command = msg.data.strip().lower()

        if not command:
            return

        if command == self.ACTIVATE:
            self.activate()
        elif command == self.DEACTIVATE:
            self.deactivate()
        else:
            self.get_logger().warn(f'Unrecognized command {msg.data}')

    
    
    def on_owner(self, msg):
        self.source = msg.data
        self.get_logger().info(f'Filter source: {self.source or "none"}')



    def activate(self):
        if self.active:
            self.get_logger().info("Blob detector already active. Ignoring command.")
            return

        cv.namedWindow('blobs')
        cv.createTrackbar('Min-Area', 'blobs', 2614, 20000, lambda _: None)
        cv.createTrackbar('Max-Area', 'blobs', 30000, 76800, lambda _: None)

        self.last_img_time = self.get_clock().now()
        self.active = True
        self.get_logger().info(f'Blob Detecto: ACTIVE (filter source: {self.source or "none"})')
        self.status_pub.publish(String(data='active'))



    def deactivate(self):
        if not self.active:
            self.get_logger().info("Blob detector already inactive. Ignoring command")
            return
        self.active = False
        cv.destroyAllWindows()
        cv.waitKey(1)
        self.get_logger().info("Blob detector: INACTIVE")
        self.status_pub.publish(String(data='inactive'))

    def on_img(self, msg):
        if not self.active:
            return

        self.last_img_time = self.get_clock().now()

        filtered = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        mask = np.any(filtered > 0, axis=2).astype(np.uint8) * 255

        min_area = cv.getTrackbarPos('Min-Area', 'blobs')
        max_area = cv.getTrackbarPos('Max-Area', 'blobs')

        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        blobs = [c for c in contours if min_area <= cv.contourArea(c) <= max_area]

        annotated = filtered.copy()
        cv.drawContours(annotated, blobs, -1, (0, 255, 0), 2)

        if blobs:
            biggest = max(blobs, key=cv.contourArea)
            m = cv.moments(biggest)
            cx = m['m10'] / m['m00']
            cy = m['m01'] / m['m00']

            theta = 0.5*np.arctan2(2*m['mu11'], m['mu20'] - m['mu02'])
            vx, vy = np.cos(theta), np.sin(theta)

            vx, vy, _, _ = cv.fitLine(biggest, cv.DIST_L2, 0, 0.01, 0.01). flatten()
            if vy > 0:
                vx, vy = -vx, -vy
            angle = float(np.degrees(np.arctan2(vx, -vy)))

            for c in blobs:
                x, y, w, h = cv.boundingRect(c)
                cv.putText(annotated, f'{float(angle)}', (x, y -5),
                        cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv.LINE_AA)

            cv.circle(annotated, (int(cx), int(cy)), 6, (0, 0, 255), -1)
            cv.line(annotated, (int(cx - 80 * vx), int(cy - 80 * vy)),
                               (int(cx + 80 * vx), int(cy + 80 * vy)), (255, 0, 0), 2)

            target = PointStamped()
            target.header = msg.header
            target.point.x = cx
            target.point.y = cy
            target.point.z = angle
            self.target_pub.publish(target)

        else:
            target = PointStamped()
            target.header = msg.header
            target.point.x = float('nan')
            target.point.y = float('nan')
            target.point.z = float('nan')
            self.target_pub.publish(target)

        out = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        out.header = msg.header
        self.annotated_pub.publish(out)

        cv.imshow('blobs', annotated)
        cv.waitKey(1)



    def watchdog(self):
        if not self.active:
            return
        elapsed = (self.get_clock().now() - self.last_img_time).nanoseconds * 1e-9
        if elapsed > self.timeout:
            self.get_logger().warn(f'No filtered images for {elapsed: .1f} s.')
            self.deactivate()



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