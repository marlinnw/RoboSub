"""camera_publisher.py -- publishes camera frames (USB or Jetson CSI).

COMMAND  /camera/command  std_msgs/String
           "activate <device> <fps>"
               device = 0, 1, ...      -> USB camera /dev/video0, /dev/video1 (V4L2)
               device = csi0, csi1     -> Jetson CSI ribbon camera, sensor-id 0 or 1 (GStreamer / Argus)
           "deactivate"
OUTPUT   /camera/image_raw             sensor_msgs/Image            (only when someone subscribes)
         /camera/image_raw/compressed  sensor_msgs/CompressedImage  (only when someone subscribes)
         /camera/status                std_msgs/String  "active" / "inactive"

Frames are always resized to WIDTH x HEIGHT before publishing.

Parameters:
    jpeg_quality  1..100, quality of the compressed stream
    show_window   true = also show frames in an OpenCV window (desk testing only)
    rotate        0 / 90 / 180 / 270, clockwise rotation applied to each frame
                  (other values = no rotation)

Examples:
  ros2 topic pub --once /camera/command std_msgs/String "{data: 'activate csi0 30'}"
  ros2 topic pub --once /camera/command std_msgs/String "{data: 'activate 0 30'}"
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from cv_bridge import CvBridge
import cv2 as cv
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String

# Published frame size (pixels). Kept small so the Jetson can filter at full frame rate.
WIDTH = 320
HEIGHT = 240


class CameraPublisher(Node):
    """Opens a camera on command and publishes its frames at a fixed rate."""

    ACTIVATE = "activate"
    DEACTIVATE = "deactivate"
    # 'rotate' parameter value (degrees clockwise) -> OpenCV rotation code
    ROTATIONS = {
        90: cv.ROTATE_90_CLOCKWISE,
        180: cv.ROTATE_180,
        270: cv.ROTATE_90_COUNTERCLOCKWISE
    }

    def __init__(self):
        super().__init__("camera_publisher")

        self.declare_parameter('jpeg_quality', 70)
        self.declare_parameter('show_window', False)
        self.declare_parameter('rotate', 0)

        self.active = False
        self.cap = None         # cv.VideoCapture while active
        self.timer = None       # frame timer while active (rate set by the fps in the command)

        self.bridge = CvBridge()
        self.status = self.create_publisher(String, 'camera/status', 10)
        self.pub = self.create_publisher(Image, 'camera/image_raw', 10)
        self.pub_compressed = self.create_publisher(CompressedImage, 'camera/image_raw/compressed', 10)
        self.command_sub = self.create_subscription(String, 'camera/command', self.on_command, 10)

    def p(self, name):
        """Current value of a parameter (read every time so live changes work)."""
        return self.get_parameter(name).value

    # ---------- commands ----------

    def on_command(self, msg):
        """Handle a message on /camera/command.

        Expected format (std_msgs/String, case and extra spaces ignored):
            "activate <device> <fps>"
                device  0, 1, ...   USB camera /dev/video0, /dev/video1
                        csi0, csi1  Jetson CSI camera, sensor-id 0 or 1
                fps     frames per second, e.g. 30 (decimals are truncated)
            "deactivate"  stop publishing and release the camera

        Sending "activate" while active restarts with the new device/fps.

        e.g. ros2 topic pub --once /camera/command std_msgs/String "{data: 'activate csi0 30'}"
        """
        words = msg.data.strip().lower().split()
        if not words:
            return

        if words[0] == self.ACTIVATE:
            if len(words) != 3:
                self.get_logger().warn("Usage: activate <device> <fps>   (device = 0, 1, ... or csi0, csi1)")
                return
            device = words[1]                       # keep as text ("0" or "csi0"), not int()
            try:
                fps = int(float(words[2]))          # whole number, GStreamer needs "30/1" not "30.0/1"
            except ValueError:
                self.get_logger().warn(f'Bad fps: {words[2]}')
                return
            self.activate(device, fps)

        elif words[0] == self.DEACTIVATE:
            self.deactivate()

        else:
            self.get_logger().warn(f'Unrecognized command: {msg.data}')

    def open_camera(self, device, fps):
        """Open a CSI camera (through the Jetson's image processor) or a USB camera.

        Returns (VideoCapture, description) for the requested device.
        Raises ValueError if the device text isn't 'csiN' or a number.
        """
        if device.startswith('csi'):
            sensor_id = int(device[3:] or 0)        # 'csi0' -> 0, 'csi1' -> 1
            # GStreamer pipeline: capture 720p from the CSI sensor (Argus), let the
            # Jetson's hardware converter (nvvidconv) scale it down, convert to BGR
            # for OpenCV. appsink keeps only the newest frame so we never lag behind.
            pipeline = (
                f'nvarguscamerasrc sensor-id={sensor_id} ! '
                f'video/x-raw(memory:NVMM),width=1280,height=720,framerate={fps}/1 ! '
                f'nvvidconv ! video/x-raw,width={WIDTH},height={HEIGHT},format=BGRx ! '
                f'videoconvert ! video/x-raw,format=BGR ! '
                f'appsink drop=true max-buffers=1 sync=false'
            )
            return cv.VideoCapture(pipeline, cv.CAP_GSTREAMER), f'CSI sensor {sensor_id}'

        index = int(device)                         # USB: '0' -> /dev/video0
        cap = cv.VideoCapture(index, cv.CAP_V4L2)
        cap.set(cv.CAP_PROP_FRAME_WIDTH, WIDTH)
        cap.set(cv.CAP_PROP_FRAME_HEIGHT, HEIGHT)
        return cap, f'/dev/video{index}'

    def activate(self, device, fps):
        """Open the camera and start publishing at fps. Restarts if already active."""
        self.deactivate()       # release any camera already open

        try:
            self.cap, name = self.open_camera(device, fps)
        except ValueError:
            self.get_logger().error(f'Bad device: {device}  (use 0, 1, ... or csi0, csi1)')
            return

        if not self.cap.isOpened():
            self.get_logger().error(f'Cannot open {name}')
            self.cap = None
            return

        self.timer = self.create_timer(1.0 / fps, self.grab)
        self.active = True
        self.status.publish(String(data='active'))
        self.get_logger().info(f'Camera ACTIVE: {name} at {fps} fps')

    def deactivate(self):
        """Stop the frame timer and release the camera. Safe to call when inactive."""
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None
        if self.cap is not None:
            self.cap.release()
            self.cap = None
        if self.active:
            self.get_logger().info("Camera INACTIVE")
            self.status.publish(String(data='inactive'))
        self.active = False
        if self.p('show_window'):
            cv.destroyAllWindows()

    # ---------- frames ----------

    def grab(self):
        """Timer callback: read one frame, resize/rotate it, and publish it."""
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn('No frame from camera', throttle_duration_sec=2.0)
            return

        if frame.shape[1] != WIDTH or frame.shape[0] != HEIGHT:     # some USB cameras ignore the size request
            frame = cv.resize(frame, (WIDTH, HEIGHT))

        rotation = self.ROTATIONS.get(int(self.p('rotate')))
        if rotation is not None:
            frame = cv.rotate(frame, rotation)

        # One timestamp for both messages so they can be matched up downstream
        stamp = self.get_clock().now().to_msg()

        # Only encode what someone is actually subscribed to (saves CPU on the Jetson)
        if self.pub.get_subscription_count() > 0:
            img = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            img.header.stamp = stamp
            img.header.frame_id = 'camera'
            self.pub.publish(img)

        if self.pub_compressed.get_subscription_count() > 0:
            ok, jpg = cv.imencode('.jpg', frame, [cv.IMWRITE_JPEG_QUALITY, int(self.p('jpeg_quality'))])
            if ok:
                msg = CompressedImage()
                msg.header.stamp = stamp
                msg.header.frame_id = 'camera'
                msg.format = 'jpeg'
                msg.data = jpg.tobytes()
                self.pub_compressed.publish(msg)

        if self.p('show_window'):
            cv.imshow('Raw Image', frame)
            cv.waitKey(1)


def main(args=None):
    """Entry point for 'ros2 run sub_py camera_publisher'."""
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.deactivate()       # release the camera device
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()