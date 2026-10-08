import os
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, TransformStamped
from moveit_msgs.msg import CollisionObject
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

IMAGE_DIR = os.path.expanduser("~/dev/ros2_ws/img")

# HSV 滑动条初始值（red_cylinder 为红色，H 大约 0~10；OpenCV 的 H 通道范围 0~179，
# 红色横跨两端 0~10 和 170~179，单段抓不全时可切换试验）
HSV_INIT = {"H_low": 0, "H_up": 10, "S_low": 80, "S_up": 255, "V_low": 60, "V_up": 255}


class ImageSub31(Node):
    """步骤 4 延伸练习：TF 广播（同 image_sub3）+ 物体进 MoveIt 规划场景。

    在 image_sub3 基础上加一路输出：把检测到的红圆柱转成 base_link 系下
    的 CYLINDER CollisionObject，发布到 /collision_object 话题；move_group
    的 PlanningScene 监听该话题，机械臂规划时会把木棍当真实障碍避让
    （呼应旧工作区 MTC demo 的结合方式：视觉输出碰撞体，运控侧避障）。

    坐标换算两步：
    - solvePnP 的 tvec/rvec 在光学系 → TF 查 planning_frame ← parent_frame，
      把位姿平移/旋转到 base_link（相机固定安装，此变换静态）
    - PnP 模型 y 轴是物体"高"方向，而 CYLINDER 原语的高沿自身 z 轴 →
      右乘 Rx(-90°) 对齐；tvec 指向正面矩形中心，沿物体 z 后退半径到轴心

    运行（本节点要 use_sim_time 才能对上 TF 时间戳；move_group 不在线时
    消息没人收，节点照常检测显示但不进规划场景）：
        robotm    # 终端1：Gazebo + MoveIt + RViz
        ros2 run mycobot_learn image_sub31 --ros-args -p use_sim_time:=true

    验证（完成标准）：RViz 里 PlanningScene → Collision Objects 勾选显示，
    半透明圆柱壳贴住 Gazebo 里的红木棍；拖动木棍后约 1s 跟随（MOVE 更新）。
    ros2 topic echo /collision_object 可看原始消息。

    已知注意：抓取时目标圆柱自身会挡住末端下降——MTC 的做法是抓取瞬间
    allowCollisions 放开；本练习先按 d 手动 REMOVE 让路，抓完可重启节点恢复。

    交互：
        HSV Controls 窗口：6 根滑动条实时调 H/S/V 上下限
        Mask 窗口：       二值化结果（白=命中阈值），调参主要看它
        Image 窗口：      检测框 + 3D 轴；s 保存，d 移除碰撞体，q 退出
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
        obj_w = float(self.get_parameter("object_width").value)
        self.object_height = float(self.get_parameter("object_height").value)
        obj_h = self.object_height
        # 圆柱半径 = 正面矩形宽的一半（模型点即按直径取的）
        self.object_radius = obj_w / 2.0
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
            f"min_area: {self.min_area}, object: {obj_w}x{self.object_height}m"
        )

        # ---- image_sub31 新增：碰撞体进规划场景的参数 ----
        # 碰撞体挂的规划坐标系（MoveIt 的 planning frame）
        self.declare_parameter("planning_frame", "base_link")
        self.planning_frame = str(self.get_parameter("planning_frame").value)
        # 碰撞体 ID（REMOVE / MOVE 都靠它定位）
        self.declare_parameter("collision_id", "red_cylinder")
        self.collision_id = str(self.get_parameter("collision_id").value)
        # 发碰撞体的最小间隔（秒）。5Hz 图像逐帧发会把规划场景刷爆，
        # MOVE 增量更新默认 1s 一次已足够平滑
        self.declare_parameter("scene_update_period", 1.0)
        self.scene_update_period = float(self.get_parameter("scene_update_period").value)
        self.last_scene_pub = 0.0
        self.object_in_scene = False

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

        # TF 广播器 + 查询缓冲（后者是把光学系位姿换到 planning_frame 用的）
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 碰撞体发布：move_group 的 PlanningSceneMonitor 订阅 /collision_object
        self.co_pub = self.create_publisher(CollisionObject, "/collision_object", 10)

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

    def publish_collision_object(self, stamp, rvec, tvec):
        """把 solvePnP 位姿转成 planning_frame 下的圆柱 CollisionObject 发给 MoveIt。"""
        # 1) 光学系 → planning_frame 的变换（Time() 取最新；相机固定安装为静态）
        try:
            tf = self.tf_buffer.lookup_transform(
                self.planning_frame, self.parent_frame, rclpy.time.Time()
            )
        except TransformException as e:
            self.get_logger().warn(
                f"TF {self.planning_frame}<-{self.parent_frame} 不可用: {e}",
                throttle_duration_sec=2.0,
            )
            return
        R_tf = Rotation.from_quat(
            [
                tf.transform.rotation.x,
                tf.transform.rotation.y,
                tf.transform.rotation.z,
                tf.transform.rotation.w,
            ]
        ).as_matrix()
        t_tf = np.array(
            [
                tf.transform.translation.x,
                tf.transform.translation.y,
                tf.transform.translation.z,
            ]
        )

        # 2) PnP 位姿修正：tvec 是正面矩形中心，沿物体 z（朝相机）后退半径
        #    到轴心；PnP 模型 y 是物体"高"方向，右乘 Rx(-90°) 让圆柱原语
        #    的 z 轴（原语高度方向）对上它
        R_pnp, _ = cv2.Rodrigues(rvec)
        p_parent = tvec.flatten() - R_pnp @ np.array([0.0, 0.0, self.object_radius])
        R_parent = R_pnp @ Rotation.from_euler("x", -90, degrees=True).as_matrix()

        # 3) 链起来：光学系位姿 → planning_frame 位姿
        p_base = t_tf + R_tf @ p_parent
        R_base = R_tf @ R_parent

        co = CollisionObject()
        co.header.frame_id = self.planning_frame
        co.header.stamp = stamp
        co.id = self.collision_id
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.CYLINDER
        # dimensions 下标即语义：[CYLINDER_HEIGHT, CYLINDER_RADIUS]
        prim.dimensions = [self.object_height, self.object_radius]
        co.primitives.append(prim)
        pose = Pose()
        pose.position.x = float(p_base[0])
        pose.position.y = float(p_base[1])
        pose.position.z = float(p_base[2])
        q = Rotation.from_matrix(R_base).as_quat()  # [x, y, z, w]
        pose.orientation.x = float(q[0])
        pose.orientation.y = float(q[1])
        pose.orientation.z = float(q[2])
        pose.orientation.w = float(q[3])
        co.primitive_poses.append(pose)
        # 首次 ADD，之后 MOVE 增量移动（MOVE 要求对象已在场景里）
        co.operation = CollisionObject.MOVE if self.object_in_scene else CollisionObject.ADD
        self.co_pub.publish(co)
        self.object_in_scene = True
        self.get_logger().info(
            f"CollisionObject '{self.collision_id}' @ "
            f"({p_base[0]:.3f}, {p_base[1]:.3f}, {p_base[2]:.3f}) m",
            throttle_duration_sec=2.0,
        )

    def remove_collision_object(self):
        """按键 d：把物体从规划场景移除（抓取时给末端让路）。"""
        co = CollisionObject()
        co.id = self.collision_id
        co.operation = CollisionObject.REMOVE
        self.co_pub.publish(co)
        self.object_in_scene = False
        self.get_logger().info(
            f"CollisionObject '{self.collision_id}' removed from planning scene"
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

                # ---- 5. 物体进 MoveIt 规划场景（节流，ADD/MOVE） ----
                now = time.monotonic()
                if now - self.last_scene_pub >= self.scene_update_period:
                    self.last_scene_pub = now
                    self.publish_collision_object(msg.header.stamp, rvec, tvec)
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
            image_path = os.path.join(IMAGE_DIR, f"frame_sub31_{self.saved}_{tm}.jpg")
            cv2.imwrite(image_path, debug)
            self.get_logger().info(f"Saving frame {image_path}")
            self.saved += 1
        elif key == ord("d"):
            self.remove_collision_object()
        elif key == ord("q"):
            cv2.destroyAllWindows()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ImageSub31("image_sub31")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
