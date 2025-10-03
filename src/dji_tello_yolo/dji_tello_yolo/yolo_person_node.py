import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import os
os.environ["TORCH_CPP_LOG_LEVEL"] = "ERROR"

import cv2
from ultralytics import YOLO
import warnings
from dji_tello_interfaces.srv import TelloCmd

warnings.filterwarnings("ignore", category=UserWarning, module="torch")


class YoloPersonDetector(Node):
    """
    ROS2 Node for person detection using YOLOv8.
    - Subscribes to Tello's camera stream (/tello/image_raw).
    - Runs YOLOv8 inference (restricted to class 'person').
    - Displays annotated video frames via OpenCV.
    - Provides a ROS2 service (/tello_yolo) to enable/disable detection.
    """

    def __init__(self):
        super().__init__('yolo_person_detector')

        # Flag to enable/disable YOLO detection
        self.enable_ = False
        # Track if an OpenCV window is currently open
        self.window_open_ = False

        # QoS profile optimized for camera streams (low-latency, best effort)
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ROS2 -> Camera image subscriber
        self.subscription = self.create_subscription(
            Image,
            '/tello/image_raw',
            self.listener_callback,
            qos_profile
        )

        # ROS <-> OpenCV bridge
        self.bridge = CvBridge()

        # Load YOLOv8 model (nano version, running on CPU)
        self.model = YOLO("yolov8n.pt")

        self.get_logger().info("YOLOv8 Person Detector Node started")

        # ROS2 Service to toggle YOLO detection
        self.srv = self.create_service(
            TelloCmd,
            "tello_yolo",
            self.callback_srv
        )

    def listener_callback(self, msg):
        """
        Callback for incoming camera frames.
        Runs YOLO detection and displays annotated results.
        """
        if not self.enable_:
            return

        # Convert ROS2 Image -> OpenCV BGR frame
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # YOLOv8 inference (restrict to 'person' class = ID 0 in COCO dataset)
        results = self.model.predict(
            frame,
            imgsz=320,
            verbose=False,
            device="cpu",
            classes=[0]
        )

        # Draw annotations on the frame
        annotated_frame = results[0].plot()

        # Show results in OpenCV window (create only once)
        if not self.window_open_:
            cv2.namedWindow("YOLOv8 Person Detection")
            self.window_open_ = True

        cv2.imshow("YOLOv8 Person Detection", annotated_frame)

        # Handle ESC key to close
        key = cv2.waitKey(1)
        if key == 27:  # ESC
            self.enable_ = False
            if self.window_open_:
                cv2.destroyWindow("YOLOv8 Person Detection")
                self.window_open_ = False
            self.get_logger().info("YOLO detection stopped by ESC key")

        # Handle window closed manually 
        if cv2.getWindowProperty("YOLOv8 Person Detection", cv2.WND_PROP_VISIBLE) < 1:
            self.enable_ = False
            self.window_open_ = False
            self.get_logger().info("YOLO window closed manually (X)")

    def callback_srv(self, request, response):
        """
        Service callback for /tello_yolo.
        Toggles YOLO detection ON/OFF.
        """
        if request.command == "YOLO":
            self.enable_ = not self.enable_
            response.success = True

            if not self.enable_ and self.window_open_:
                cv2.destroyWindow("YOLOv8 Person Detection")
                self.window_open_ = False

            self.get_logger().info(
                f"YOLO detection {'enabled' if self.enable_ else 'disabled'}"
            )
        else:
            response.success = False
            self.get_logger().warn(f"Unknown command: {request.command}")

        return response


def main(args=None):
    rclpy.init(args=args)
    node = YoloPersonDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Graceful shutdown
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
