import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


class ImageSub22(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        # 订阅摄像机参数
        qos = QoSProfile(
            depth=1,
        )
        self.subscription = self.create_subscription(
            CameraInfo, "/camera_head/depth/camera_info", self.camera_info_callback, qos
        )

    def camera_info_callback(self, msg):
        self.get_logger().info(f"Received camera info: {msg}")
        self.destroy_subscription(self.subscription)


def main(args=None):
    rclpy.init(args=args)
    node = ImageSub22("ImageSub22")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
