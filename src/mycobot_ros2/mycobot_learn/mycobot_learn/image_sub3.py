import os
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

IMAGE_DIR = os.path.expanduser("~/dev/ros2_ws/img")

# HSV 滑动条初始值（red_cylinder 为红色，H 大约 0~10；OpenCV 的 H 通道范围 0~179，
# 红色横跨两端 0~10 和 170~179，单段抓不全时可切换试验）
HSV_INIT = {"H_low": 0, "H_up": 10, "S_low": 80, "S_up": 255, "V_low": 60, "V_up": 255}


class ImageSub3(Node):
    """步骤 4 练习：把 solvePnP 位姿广播进 TF（object_frame）。

    在 image_sub2（检测 + solvePnP）基础上加 TransformBroadcaster：
    每帧广播 camera_head_depth_optical_frame → object_frame，
    translation = tvec，rotation = 旋转矩阵转四元数（scipy）。

    ⚠️ optical frame 惯例是 z 朝前、y 朝下；object_frame 挂在 optical
    下面时，其 x/y/z 轴即模型点定义的 右/上/朝相机。要看 base_link 下
    的位姿，用 TF 树串联（RViz 直接显示即自动完成）。

    输出两路：① TF 广播 object_frame（原始检测值，RViz / tf2_echo 调试用）；
    ② /vision/detected_point（PointStamped，base_link 系 + calib_x/calib_y
    校准）——运动侧（arm_move_to_object）只订阅 ②，坐标换算和偏差校准
    都在视觉侧完成，运动节点不再查 TF / 算校准。

    物体尺寸来自 models/red_cylinder/model.sdf（radius 0.015 → 直径
    0.03m，length 0.35m）。若换测 mustard 瓶：H 改 20~35，尺寸 0.065x0.160
    （mesh 内部 0.2 缩放 × SDF scale 0.05 实测），用参数传入即可。

    验证（完成标准）：RViz Fixed Frame 选 world，开 TF 显示，看
    object_frame 坐标轴贴在木棍上；Gazebo 里拖动木棍后跟随。
    （RViz 要 use_sim_time:=true，否则 TF 时间戳对不上会不显示。）

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
        # TF 父帧：优先用 camera_info 自带的 frame_id，其次图像消息的
        self.parent_frame = "camera_head_depth_optical_frame"

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
        # 广播的物体坐标系名
        self.declare_parameter("object_frame", "object_frame")
        self.object_frame = str(self.get_parameter("object_frame").value)
        # 输出点坐标系与视觉偏差校准（solvePnP 的 y 实测偏高 ~2cm，随俯角变大）
        self.declare_parameter("base_frame", "base_link")
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.declare_parameter("calib_x", 0.0)
        self.calib_x = float(self.get_parameter("calib_x").value)
        self.declare_parameter("calib_y", 0.0)
        self.calib_y = float(self.get_parameter("calib_y").value)
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

        # TF 广播器
        self.tf_broadcaster = TransformBroadcaster(self)

        # TF 监听 + base_link 位置话题：solvePnP 结果换算成 base_link 系
        # 直接发给运动侧（arm_move_to_object 只订阅，不再查 TF / 算校准）
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.point_pub = self.create_publisher(
            PointStamped, "/vision/detected_point", 1
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
        # gz 桥接的 camera_info 自带的是 Gazebo 相机 link 名（REP103 朝向
        # x前y左z上，如 camera_head_link），而 solvePnP 的 tvec/rvec 是光学
        # 约定（z前x右y下）——只能挂光学系父帧，否则 TF 差 90° 旋转，物体
        # z 会被串到 1m 多高。只信 _optical_frame 后缀，其余保持 URDF 默认
        if msg.header.frame_id.endswith("_optical_frame"):
            self.parent_frame = msg.header.frame_id
        fx, fy, cx, cy = msg.k[0], msg.k[1], msg.k[2], msg.k[5]
        self.get_logger().info(
            f"camera_info: fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}, "
            f"frame: {self.parent_frame}"
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

    def publish_tf(self, stamp, rvec, tvec):
        """把 solvePnP 结果广播为 parent_frame → object_frame。"""
        ts = TransformStamped()
        ts.header.stamp = stamp
        ts.header.frame_id = self.parent_frame
        ts.child_frame_id = self.object_frame
        ts.transform.translation.x = float(tvec[0])
        ts.transform.translation.y = float(tvec[1])
        ts.transform.translation.z = float(tvec[2])
        # scipy 四元数顺序 [x, y, z, w]
        q = Rotation.from_matrix(cv2.Rodrigues(rvec)[0]).as_quat()
        ts.transform.rotation.x = float(q[0])
        ts.transform.rotation.y = float(q[1])
        ts.transform.rotation.z = float(q[2])
        ts.transform.rotation.w = float(q[3])
        self.tf_broadcaster.sendTransform(ts)

    def publish_base_point(self, stamp, tvec):
        """solvePnP 平移换算到 base_frame，发 /vision/detected_point。

        用最新可用的 optical→base_frame 变换（Time() 取 latest——单线程
        spin 里带时间戳查询会阻塞等新 TF，死等不到）；查不到时跳过本帧。
        calib_x/calib_y 在这里修视觉系统偏差。注意 TF 广播的 object_frame
        仍是原始检测值，与这里的校准点对比即可量出视觉偏差。
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.parent_frame,
                Time(),
                timeout=Duration(seconds=0.1),
            )
        except TransformException as e:
            self.get_logger().warn(
                f"{self.parent_frame}→{self.base_frame} 变换未就绪，跳过本帧: {e}",
                throttle_duration_sec=2.0,
            )
            return
        tr, rot = tf.transform.translation, tf.transform.rotation
        p = Rotation.from_quat(
            [rot.x, rot.y, rot.z, rot.w]
        ).as_matrix() @ tvec.flatten() + np.array([tr.x, tr.y, tr.z])
        out = PointStamped()
        out.header.stamp = stamp
        out.header.frame_id = self.base_frame
        out.point.x = float(p[0]) + self.calib_x
        out.point.y = float(p[1]) + self.calib_y
        out.point.z = float(p[2])
        self.point_pub.publish(out)
        self.get_logger().info(
            f"{self.base_frame}: ({out.point.x:.3f}, {out.point.y:.3f}, "
            f"{out.point.z:.3f}) m",
            throttle_duration_sec=1.0,
        )

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

                # ---- 4. 位姿进 TF：parent_frame → object_frame ----
                self.publish_tf(msg.header.stamp, rvec, tvec)
                # ---- 5. base_link 系位置发运动侧 ----
                self.publish_base_point(msg.header.stamp, tvec)
        elif contours and self.K is None:
            self.get_logger().warn(
                "waiting for camera_info...", throttle_duration_sec=2.0
            )

        # ---- 6. 显示（放大 2 倍便于观看）----
        h, w = frame.shape[:2]
        debug = cv2.resize(frame, (w * 2, h * 2), interpolation=cv2.INTER_LINEAR)
        cv2.imshow("Image", debug)
        cv2.imshow("Mask", mask)  # 白色=落入 HSV 阈值区，调参看这个窗口
        key = cv2.waitKey(1) & 0xFF
        if key == ord("s"):
            tm = time.strftime("%Y%m%d_%H%M%S")
            image_path = os.path.join(IMAGE_DIR, f"frame_sub3_{self.saved}_{tm}.jpg")
            cv2.imwrite(image_path, debug)
            self.get_logger().info(f"Saving frame {image_path}")
            self.saved += 1
        elif key == ord("q"):
            cv2.destroyAllWindows()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ImageSub3("image_sub3")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
