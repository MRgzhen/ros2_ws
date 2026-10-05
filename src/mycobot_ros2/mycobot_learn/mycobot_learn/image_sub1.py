import os
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image

IMAGE_DIR = os.path.expanduser("~/dev/ros2_ws/img")

# HSV 滑动条初始值（先按仿真里的红色圆柱/可乐罐粗调，运行后用滑条微调）
# 注意：OpenCV 的 H 通道范围是 0~179，红色横跨两端（0~10 和 170~179），
#       单段 inRange 抓不全时可在两段之间切换试验
HSV_INIT = {"H_low": 0, "H_up": 10, "S_low": 80, "S_up": 255, "V_low": 60, "V_up": 255}


class ImageSub1(Node):
    """阶段四练习：HSV 颜色分割 + 轮廓分析，检测红色目标物。

    订阅彩色图 → HSV 阈值分割(inRange) → 形态学去噪 → findContours
    → 面积过滤取最大轮廓 → minAreaRect 画旋转检测框 + 质心

    交互：
        HSV Controls 窗口：6 根滑动条实时调 H/S/V 上下限
        Mask 窗口：       二值化结果（白=命中阈值），调参主要看它
        Image 窗口：      标注后的检测画面；s 保存，q 退出
    """

    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("Image subscriber node started")
        os.makedirs(IMAGE_DIR, exist_ok=True)

        # 初始化 CvBridge
        self.bridge = CvBridge()
        self.saved = 0

        # 注册参数
        self.declare_parameter("image_topic", "/camera_head/color/image_raw")
        image_topic = str(self.get_parameter("image_topic").value)
        # 轮廓面积阈值（原图像素，424x240 分辨率下小于它的轮廓视为噪声丢弃）
        self.declare_parameter("min_area", 200.0)
        min_area_value = self.get_parameter("min_area").value
        self.min_area = float(min_area_value) if min_area_value is not None else 200.0
        self.get_logger().info(f"subscribing: {image_topic}, min_area: {self.min_area}")

        # 订阅（相机端 BEST_EFFORT，订阅端必须一致才能收到）
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.subscription = self.create_subscription(
            Image, image_topic, self.image_callback, qos
        )

        # 创建 HSV 滑动条调参窗口：H 上限 179，S/V 上限 255
        cv2.namedWindow("HSV Controls")
        for bar, value in HSV_INIT.items():
            upper = 179 if bar.startswith("H") else 255
            cv2.createTrackbar(bar, "HSV Controls", value, upper, lambda x: None)

    def image_callback(self, msg):
        # 转换成 OpenCV 图像
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except (TypeError, ValueError, cv2.error) as e:
            self.get_logger().error(f"Error processing image message: {e}")
            return

        # ---- 1. HSV 阈值分割 ----
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        pos = lambda k: cv2.getTrackbarPos(k, "HSV Controls")
        lower = np.array([pos("H_low"), pos("S_low"), pos("V_low")])
        upper = np.array([pos("H_up"), pos("S_up"), pos("V_up")])
        mask = cv2.inRange(hsv, lower, upper)

        # 形态学去噪：开运算去掉零星白点，闭运算填补目标内部小洞
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # ---- 2. 轮廓分析：面积过滤 + 取最大轮廓为目标 ----
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = [c for c in contours if cv2.contourArea(c) >= self.min_area]

        if contours:
            target = max(contours, key=cv2.contourArea)

            # 最小外接旋转矩形（红框）
            rect = cv2.minAreaRect(target)
            box = np.intp(cv2.boxPoints(rect))
            cv2.drawContours(frame, [box], 0, (0, 0, 255), 2)

            # 质心（绿点 + 像素坐标）
            m = cv2.moments(target)
            if m["m00"] > 0:
                cx, cy = int(m["m10"] / m["m00"]), int(m["m01"] / m["m00"])
                cv2.circle(frame, (cx, cy), 4, (0, 255, 0), -1)
                cv2.putText(
                    frame,
                    f"({cx},{cy})",
                    (cx + 8, cy - 8),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    1,
                )
                self.get_logger().info(
                    f"target at ({cx}, {cy}), area {int(cv2.contourArea(target))}",
                    throttle_duration_sec=1.0,
                )

        # ---- 3. 显示（放大 2 倍便于观看）----
        h, w = frame.shape[:2]
        debug = cv2.resize(frame, (w * 2, h * 2), interpolation=cv2.INTER_LINEAR)
        cv2.imshow("Image", debug)
        cv2.imshow("Mask", mask)  # 白色=落入 HSV 阈值区，调参看这个窗口
        key = cv2.waitKey(1) & 0xFF
        if key == ord("s"):
            tm = time.strftime("%Y%m%d_%H%M%S")
            image_path = os.path.join(IMAGE_DIR, f"frame_sub1_{self.saved}_{tm}.jpg")
            cv2.imwrite(image_path, debug)
            self.get_logger().info(f"Saving frame {image_path}")
            self.saved += 1
        elif key == ord("q"):
            cv2.destroyAllWindows()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ImageSub1("image_sub1")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
