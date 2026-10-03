import os
import time
from platform import node

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Image

IMAGE_DIR = os.path.expanduser("~/dev/ros2_ws/img")


class ImageSub(Node):
    def __init__(self, name):
        super().__init__(name)

        self.get_logger().info("Image subscriber node started")
        os.makedirs(IMAGE_DIR, exist_ok=True)

        # 初始化 CvBridge
        self.bridge = CvBridge()
        self.frame_count = 0
        self.saved = 0

        # 注册参数
        self.declare_parameter("image_topic", "/camera_head/color/image_raw")
        image_topic = str(self.get_parameter("image_topic").value)

        # 订阅
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.subscription = self.create_subscription(
            Image, image_topic, self.image_callback, qos
        )

    def image_callback(self, msg):
        self.frame_count += 1

        # 转换成OpenCV图像
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Error processing image message: {e}")
            return

        # 显示图像
        cv2.imshow("Image", frame)
        self.get_logger().info(f"Processing frame {frame.shape}")
        key = cv2.waitKey(1) & 0xFF
        if key == ord("s"):
            tm = time.strftime("%Y%m%d_%H%M%S")
            image_path = os.path.join(IMAGE_DIR, f"frame_{self.saved}_{tm}.jpg")
            self.get_logger().info(f"Saving frame {self.saved}")
            cv2.imwrite(image_path, frame)
            self.saved += 1
        elif key == ord("q"):
            cv2.destroyAllWindows()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ImageSub("image_sub")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
