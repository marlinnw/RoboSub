import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np
import cv2 as cv

class EdgeDetect(Node):

    ACTIVATE = "activate_edge"
    DEACTIVATE = "deactivate_edge"

    def __init__(self):
        super().__init__('edge_detect_listener')

        self.active = False
        self.frames = 0
        self.cap = None

        
        self.sub = self.create_subscription(String, "edge_detect/command", self.on_command, 10)
        self.status_pub = self.create_publisher(String, "edge_detect/status", 10)
        self.timer = self.create_timer(0.03, self.process_frame)

    def on_command(self, msg):
        command = msg.data.strip().lower()

        if command == self.ACTIVATE:
            if self.active:
                self.get_logger().info("Edge detection already active. Ignoring command")
                return
            self.activate()

        elif command == self.DEACTIVATE:
            if not self.active:
                self.get_logger().info("Edge detection already deactive. Ignoring command")
                return
            self.deactivate()

        else:
            self.get_logger().warn(f"Unknown command: {msg.data}")


    
    
    
    def activate(self):
        self.cap = cv.VideoCapture(0, cv.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error("Cannot open camera")
            self.cap = None
            return

        cv.namedWindow("edges")
        cv.createTrackbar("channel", "edges", 6, 6, lambda _:None)
        cv.createTrackbar("low", "edges", 30, 90, lambda _:None)
        cv.createTrackbar("high", "edges", 90, 255, lambda _:None)
        cv.createTrackbar("blur", "edges", 2, 10, lambda _: None)

        self.active = True
        self.frames = 0
        self.get_logger().info("Edge detection: ACTIVATED")
        self.status_pub.publish(String(data='active'))

    
    
    
    def deactivate(self):
        self.active = False
        if self.cap is not None:
            self.cap.release()
            self.cap = None
        cv.destroyAllWindows()
        cv.waitKey(1)
        self.get_logger().info("Edge detection: DEACTIVATED")
        self.get_logger().info(f'Processed {self.frames} frames')
        self.status_pub.publish(String(data="deactive"))

    
    
    
    def process_frame(self):
        if not self.active:
            return

        # capture frame by frame
        ok, frame = self.cap.read()

        # if frame is read correctly ret is True
        if not ok:
            self.get_logger().error("Can't recieve frame (stream end?). Exiting...")
            self.deactivate()
            return

        self.frames += 1

        # read the sliders
        ch = cv.getTrackbarPos("channel", "edges")
        lo = cv.getTrackbarPos("low", "edges")
        hi = cv.getTrackbarPos("high", "edges")
        k = cv.getTrackbarPos("blur", "edges") * 2 + 1


        lab_frame = cv.cvtColor(cv.GaussianBlur(frame, (k, k), 0), cv.COLOR_BGR2Lab)
        l, a, b = cv.split(lab_frame)

        # Canny works on one channel at a time
        if ch == 0:
            edges = cv.Canny(l, lo, hi)
        elif ch == 1:
            edges = cv.Canny(a, lo, hi)
        elif ch == 2:
            edges = cv.Canny(b, lo, hi)
        elif ch == 3:
            edges = cv.Canny(l, lo, hi) | cv.Canny(a, lo, hi)
        elif ch == 4:
            edges = cv.Canny(l, lo, hi) | cv.Canny(b, lo, hi)
        elif ch == 5:
            edges = cv.Canny(a, lo, hi) | cv.Canny(b, lo, hi)
        else:
            edges = cv.Canny(l, lo, hi) | cv.Canny(a, lo, hi) | cv.Canny(b, lo, hi)

        cv.imshow("camera", frame)
        cv.imshow("edges", edges)
        cv.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = EdgeDetect()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()