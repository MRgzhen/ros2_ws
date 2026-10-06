import os
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

IMAGE_DIR = os.path.expanduser("~/dev/ros2_ws/img")

# HSV 滑动条初始值（red_cylinder 为红色，H 大约 0~10；OpenCV 的 H 通道范围 0~179，
# 红色横跨两端 0~10 和 170~179，单段抓不全时可切换试验）
HSV_INIT = {"H_low": 0, "H_up": 10, "S_low": 80, "S_up": 255, "V_low": 60, "V_up": 255}


class ImageSub2(Node):
    """步骤 3 练习：solvePnP 求红色木棍（red_cylinder）6D 位姿。

    检测（同 image_sub1：HSV 分割 + 形态学 + 最大轮廓 minAreaRect）
    → 检测框 4 角近似物体正面矩形角点 → cv2.solvePnP → rvec/tvec
    → Rodrigues 转旋转矩阵，drawFrameAxes 在画面画 3D 轴

    物体尺寸来自 models/red_cylinder/model.sdf（radius 0.015 → 直径
    0.03m，length 0.35m）。圆柱是旋转体，绕自身轴的角度不可观，位姿
    以位置 + 轴向为主。若换测 mustard 瓶：H 改 20~35，尺寸 0.065x0.160
    （mesh 内部 0.2 缩放 × SDF scale 0.05 实测），用参数传入即可。

    验证（完成标准）：tvec 是相机光心系（x 右、y 下、z 朝前）下物体中心
    位置；与 Gazebo 实体树里 red_cylinder 的 pose 对照——直接比 |tvec|
    和世界系下（物体位置 − 相机光心位置）的模长，误差 <3cm 即通过。
    ⚠️ 仿真的 RGB 与 depth 天然对齐，真机要 align——概念知道即可。

    交互：
        HSV Controls 窗口：6 根滑动条实时调 H/S/V 上下限
        Mask 窗口：       二值化结果（白=命中阈值），调参主要看它
        Image 窗口：      检测框 + 3D 轴；s 保存，q 退出
    """

    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("Image subscriber node started")
        os.makedirs(IMAGE_DIR, exist_ok=True)

        # 初始化 CvBridge
        self.bridge = CvBridge()
        self.saved = 0

        # 相机内参（等 camera_info 到位后才开始解 PnP）
        self.K = None
        self.D = None

        # 注册参数
        self.declare_parameter("image_topic", "/camera_head/color/image_raw")
        image_topic = str(self.get_parameter("image_topic").value)
        self.declare_parameter("camera_info_topic", "/camera_head/depth/camera_info")
        camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        # 轮廓面积阈值（原图像素，424x240 分辨率下小于它的轮廓视为噪声丢弃）
        self.declare_parameter("min_area", 200.0)
        min_area_value = self.get_parameter("min_area").value
        self.min_area = float(min_area_value) if min_area_value is not None else 200.0
        # 物体正面矩形尺寸（米），solvePnP 的模型点用
        # （red_cylinder：直径 0.03 × 高 0.35；mustard 则为 0.065 × 0.160）
        self.declare_parameter("object_width", 0.03)
        self.declare_parameter("object_height", 0.35)
        obj_w = float(self.get_parameter("object_width").value)
        obj_h = float(self.get_parameter("object_height").value)
        # 模型点：物体中心为原点、正面朝相机（x 右、y 上），顺序与图像角点
        # 左上/右上/右下/左下 一一对应
        self.obj_points = np.array(
            [
                [-obj_w / 2, obj_h / 2, 0],  # 左上
                [obj_w / 2, obj_h / 2, 0],  # 右上
                [obj_w / 2, -obj_h / 2, 0],  # 右下
                [-obj_w / 2, -obj_h / 2, 0],  # 左下
            ],
            dtype=np.float32,
        )
        self.get_logger().info(
            f"subscribing: {image_topic}, {camera_info_topic}, "
            f"min_area: {self.min_area}, object: {obj_w}x{obj_h}m"
        )

        # 订阅（相机端 BEST_EFFORT，订阅端必须一致才能收到）
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.subscription = self.create_subscription(
            Image, image_topic, self.image_callback, qos
        )
        self.info_subscription = self.create_subscription(
            CameraInfo, camera_info_topic, self.camera_info_callback, qos
        )

        # 创建 HSV 滑动条调参窗口：H 上限 179，S/V 上限 255
        cv2.namedWindow("HSV Controls")
        for bar, value in HSV_INIT.items():
            upper = 179 if bar.startswith("H") else 255
            cv2.createTrackbar(bar, "HSV Controls", value, upper, lambda x: None)

    def camera_info_callback(self, msg):
        """拿一次内参就够（仿真里内参不变），之后取消订阅。"""
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        # gz 桥接出来的 D 可能为空数组，solvePnP 需要 5 个畸变系数
        self.D = np.array(msg.d, dtype=np.float64) if msg.d else np.zeros(5)
        fx, fy, cx, cy = msg.k[0], msg.k[1], msg.k[2], msg.k[5]
        self.get_logger().info(
            f"camera_info: fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}"
        )
        self.destroy_subscription(self.info_subscription)

    @staticmethod
    def order_box_points(pts):
        """4 个角点按 左上/右上/右下/左下 排序（图像坐标 y 向下）。

        cv2.boxPoints 返回的点本来就是环绕顺序（起点可能是任意角），
        所以只需找 x+y 最小的角（近似左上）轮转到开头，保持相邻关系。
        """
        pts = np.asarray(pts, dtype=np.float32)
        start = int(np.argmin(pts.sum(axis=1)))
        return np.roll(pts, -start, axis=0)

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

        if contours and self.K is not None:
            target = max(contours, key=cv2.contourArea)

            # 最小外接旋转矩形（红框）
            rect = cv2.minAreaRect(target)
            box = np.intp(cv2.boxPoints(rect))
            cv2.drawContours(frame, [box], 0, (0, 0, 255), 2)

            # ---- 3. solvePnP：检测框 4 角 → 6D 位姿 ----
            img_points = self.order_box_points(cv2.boxPoints(rect).astype(np.float32))
            ok, rvec, tvec = cv2.solvePnP(
                self.obj_points, img_points, self.K, self.D, flags=cv2.SOLVEPNP_IPPE
            )
            if ok:
                # 旋转矩阵（rvec 是旋转向量，Rodrigues 转矩阵备用）
                R, _ = cv2.Rodrigues(rvec)
                # 画 3D 轴验证姿态：蓝 x、绿 y、红 z，轴长 3cm
                cv2.drawFrameAxes(frame, self.K, self.D, rvec, tvec, 0.03)

                tx, ty, tz = tvec.flatten()
                dist = float(np.linalg.norm(tvec))
                cv2.putText(
                    frame,
                    f"t=({tx:.3f},{ty:.3f},{tz:.3f})m d={dist:.3f}m",
                    (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    (0, 255, 255),
                    1,
                )
                self.get_logger().info(
                    f"tvec=({tx:.3f}, {ty:.3f}, {tz:.3f}) m, |tvec|={dist:.3f} m",
                    throttle_duration_sec=1.0,
                )
        elif contours and self.K is None:
            self.get_logger().warn(
                "waiting for camera_info...", throttle_duration_sec=2.0
            )

        # ---- 4. 显示（放大 2 倍便于观看）----
        h, w = frame.shape[:2]
        debug = cv2.resize(frame, (w * 2, h * 2), interpolation=cv2.INTER_LINEAR)
        cv2.imshow("Image", debug)
        cv2.imshow("Mask", mask)  # 白色=落入 HSV 阈值区，调参看这个窗口
        key = cv2.waitKey(1) & 0xFF
        if key == ord("s"):
            tm = time.strftime("%Y%m%d_%H%M%S")
            image_path = os.path.join(IMAGE_DIR, f"frame_sub2_{self.saved}_{tm}.jpg")
            cv2.imwrite(image_path, debug)
            self.get_logger().info(f"Saving frame {image_path}")
            self.saved += 1
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
