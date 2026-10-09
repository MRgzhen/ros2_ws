import os
import time
from cmath import rect

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image

IMG_PATH = os.path.expanduser("~/dev/ros2_ws/img")
HSV_INIT = {"H_low": 0, "H_up": 10, "S_low": 80, "S_up": 255, "V_low": 60, "V_up": 255}


class ImageSub2(Node):
    def __init__(self, name):

        super().__init__(name)
        # 初始化变量
        self.image_count = 0

        # 注册生命参数
        self.declare_parameter("image_topic", "/camera_head/color/image_raw")
        image_topic = str(self.get_parameter("image_topic").value)

        # 创建订阅者
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.test = self.create_subscription(
            Image, image_topic, self.image_callback, qos
        )

        # 给定窗口大小
        cv2.namedWindow("Image", cv2.WINDOW_AUTOSIZE)
        cv2.resizeWindow("Image", 640, 480)

        # 初始化HSV参数
        cv2.namedWindow("HSV Controls")
        for key, value in HSV_INIT.items():
            upper = 179 if key.startswith("H") else 255
            cv2.createTrackbar(key, "HSV Controls", value, upper, lambda x: None)

        # 创建文件夹
        os.makedirs(IMG_PATH, exist_ok=True)

    def image_callback(self, msg):
        # 使用cv_bridge将ROS图像消息转换为OpenCV图像
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

        # 图像分割
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        pos = lambda k: cv2.getTrackbarPos(k, "HSV Controls")
        lower = np.array([pos("H_low"), pos("S_low"), pos("V_low")])
        upper = np.array([pos("H_up"), pos("S_up"), pos("V_up")])
        mask = cv2.inRange(hsv_image, lower, upper)
        # cv_image_new = cv2.bitwise_and(cv_image, cv_image, mask=mask)
        # cv2.imshow("HSV", hsv_image)
        # cv2.imshow("cv_image_new", cv_image_new)

        # 形态学去噪：开运算去掉零星白点，闭运算填补目标内部小洞
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        cv2.imshow("Mask", mask)

        # 轮廓检测
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnt = max(contours, key=cv2.contourArea)
        if cnt is not None:
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect).astype(np.int32)
            cv2.drawContours(cv_image, [box], -1, (0, 255, 0), 2)

            # 质心
            M = cv2.moments(cnt)
            cx, cy = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
            cv2.circle(cv_image, (cx, cy), 5, (0, 0, 255), -1)
            cv2.putText(
                cv_image,
                f"Center: ({cx}, {cy})",
                (cx - 50, cy - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 255),
                2,
            )
        cv2.imshow("Image", cv_image)
        # 保存图像到文件
        key = cv2.waitKey(1) & 0xFF
        if key == ord("s"):
            # 写入文件
            cv2.imwrite(
                os.path.join(
                    IMG_PATH,
                    f"image_{self.image_count}_{time.strftime('%Y%m%d_%H%M%S')}.jpg",
                ),
                cv_image,
            )
            self.image_count += 1
        elif key == ord("q"):
            cv2.destroyAllWindows()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ImageSub2("image_sub2")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
