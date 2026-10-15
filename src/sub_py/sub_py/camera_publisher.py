import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from cv_bridge import CvBridge
import cv2 as cv
from sensor_msgs.msg import Image
from std_msgs.msg import String

class CameraPublisher(Node):

    ACTIVATE = "activate"
    DEACTIVATE = "deactivate"

    def __init__(self):
        super().__init__("camera_publisher")

        self.active = False
        self.cap = None
        self.timer = None

        self.bridge = CvBridge()

        self.status         = self.create_publisher(String, '/camera/status', 10)
        self.pub            = self.create_publisher(Image, '/camera/image_raw', 10)
        self.command_sub    = self.create_subscription(String, '/camera/command', self.on_command, 10)

    def on_command(self, msg):

        words = msg.data.strip().lower().split()

        if not words:
            return

        if words[0] == self.ACTIVATE:
            if len(words) != 3:
                self.get_logger().warn("Usage: activate <device> <fps>")
                return
            try:
                device = int(words[1])
                fps = float(words[2])
            except:
                self.get_logger().warn("Device and fps must be numbers")
                return

            self.activate(device, fps)

        elif words[0] == self.DEACTIVATE:
            self.deactivate()

        else:
            self.get_logger().warn(f'Unrecognized command: {msg.data}')



    def activate(self, device, fps):
        self.deactivate()

        self.cap = cv.VideoCapture(device, cv.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error(f'Cannot open /dev/video{device}')
            self.cap = None
            return

        self.cap.set(cv.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, 240)
        self.timer = self.create_timer(1.0 / fps, self.grab)
        self.active = True
        self.status.publish(String(data='active'))
        self.get_logger().info(f'Camera ACTIVE: /dev/video{device} at {fps} fps')



    def deactivate(self):
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None
        if self.cap is not None:
            self.cap.release()
            self.cap = None
        if self.active:
            cv.destroyAllWindows()
            cv.waitKey(1)
            self.get_logger().info("Camera INACTIVE")
        self.active = False

    def grab(self):
        ok, frame = self.cap.read()
        if not ok:
            return
        img = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img.header.stamp = self.get_clock().now().to_msg()
        img.header.frame_id = 'camera'
        self.pub.publish(img)

        cv.imshow('Raw Image', frame)
        cv.waitKey(1)



def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.deactivate()
        node.destroy_node()
        rclpy.try_shutdown()
        
