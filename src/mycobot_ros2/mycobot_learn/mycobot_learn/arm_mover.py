"""机械臂运动公共封装（ArmMover）——视觉无关的库层。

把 MoveIt（moveit_py）+ TF 查询 + 夹爪 action 封装成一个类，任何脚
本 import 即可用；视觉侧只要往 TF 里广播一个 frame（如 object_frame），
或给出 base 系下的目标点，就能驱使机械臂运动到位、悬停下降、夹取。

对接契约（换 YOLO/SAM 等视觉实现时只需满足其一）：
    1. TF：   广播 child_frame = object_frame（范本见 image_sub3.publish_tf）
    2. 位姿： 直接把 PoseStamped（base_link 系）喂给 move_to_pose()

移植自旧工作区 C++ 实现（/home/gz/ros2_ws）：
    - move_to_point.cpp   → move_to_frame/hover_and_descend（限速 0.3、
      "no valid states" 容差经验见 arm_move_to_object 文档字符串）
    - attach_object_demo.cpp → attach_box_to_ee/detach_object
      （touch_links 列表照抄：gripper_base + 左右手指各 3 个 link）

用法骨架：
    rclpy.init()                     # 必须先 init，--ros-args 全局参数
    with ArmMover() as mover:        # （如 use_sim_time）才能传播进进程内所有节点
        mover.move_to_named("ready")
        mover.move_to_frame("object_frame", offset_xyz=(0, 0, 0.15))

⚠️ 只在仿真里验证过（Gazebo + move_group 由
   mycobot_280_gazebo_and_moveit.sh 拉起）；真机需另行审查。
"""

import os
import threading
import time

import rclpy
import tf2_ros
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import Pose, PoseStamped
from moveit.planning import MoveItPy, PlanRequestParameters
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    AttachedCollisionObject,
    CollisionObject,
    PlanningScene,
    PlanningSceneComponents,
    RobotState,
)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetPlanningScene
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from shape_msgs.msg import SolidPrimitive
from tf2_ros import TransformException

# 夹爪竖直朝下的目标姿态（pose_link=link6_flange、表达在 base 系，绕 x 轴 +90°，
# 由 ready 位姿 FK 实算）。注意与旧 C++ 的 (1,0,0,0) 不同：那边 pose_link 不是
# link6_flange。以 dry-run 在 RViz 里实看为准，不对就换这两个值组合。
DOWNWARD_QUAT = (0.70711, 0.0, 0.0, 0.70711)

# 夹爪命名状态 → gripper_controller 关节位置（来自 SRDF group_state）
GRIPPER_POSITIONS = {"open": 0.0, "half_closed": -0.34, "closed": -0.50}

# 附着物体时允许与之接触的 link（否则 MoveIt 认为手指和物体相撞，规划必败）
TOUCH_LINKS = [
    "gripper_base",
    "gripper_left1", "gripper_left2", "gripper_left3",
    "gripper_right1", "gripper_right2", "gripper_right3",
]


class ArmMover:
    """MoveItPy + TF + 夹爪的机械臂运动封装（库层，可复用于任意视觉实现）。

    前提：调用方先 rclpy.init()，再构造本类（保证 use_sim_time 等
    全局参数传播给进程内所有节点，含 moveit_py 内部的 rclcpp 节点）。
    """

    def __init__(
        self,
        *,
        node_name="arm_mover",
        arm_group="arm",
        gripper_group="gripper",
        pose_link="link6_flange",
        arm_controller_name="arm_controller",
        use_sim_time=True,
        planning_pipeline="ompl",
        planner_id="RRTConnectkConfigDefault",
        planning_time=5.0,
        max_velocity_scaling=0.3,
        max_acceleration_scaling=0.3,
        use_config_builder=True,
    ):
        self.pose_link = pose_link
        self.arm_group = arm_group
        self.arm_controller_name = arm_controller_name

        # ---- ① TF：独立节点 + 后台线程 spin（主线程跑运动，互不阻塞）----
        self._tf_node = Node(f"{node_name}_tf")
        self._tf_executor = SingleThreadedExecutor()
        self._tf_executor.add_node(self._tf_node)
        self._tf_thread = threading.Thread(
            target=self._spin_tf, daemon=True, name="arm_mover_tf"
        )
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self._tf_node)
        self._tf_thread.start()

        # ---- ② MoveItPy ----
        # 裸构造只能从话题拿到 URDF/SRDF，拿不到 kinematics/planning pipeline 参数
        # （它们只在 move_group 节点的参数服务器上），会报
        # "Failed to load planning pipelines"。因此默认直接读
        # mycobot_moveit_config 的 yaml 装配这些参数（URDF 仍从话题取）。
        # use_sim_time 也必须写进来：命令行的 --ros-args 只作用于 rclpy 节点，
        # moveit_cpp 内部 rclcpp 节点看不到，时钟不一致会导致执行前状态校验失败。
        # 注意要走 launch_params_filepaths 且根键用 /**：config_dict 内部用节点名
        # 做根键，与 use_sim_time 组合会触发 rclcpp qos_overrides 崩溃
        # （moveit2 issue #2220/#2940 的社区解法）。
        if use_config_builder:
            from moveit.utils import create_params_file_from_dict

            params_file = create_params_file_from_dict(
                self._build_config(use_sim_time), "/**"
            )
            self._moveit = MoveItPy(
                node_name=node_name, launch_params_filepaths=[params_file]
            )
        else:
            self._moveit = MoveItPy(node_name=node_name)
        self._arm = self._moveit.get_planning_component(arm_group)
        self._gripper = self._moveit.get_planning_component(gripper_group)

        # 规划参数（pybind 类必须传 MoveItPy 实例构造，标量属性再逐个赋值）
        self._plan_params = PlanRequestParameters(self._moveit, planning_pipeline)
        self._plan_params.planning_pipeline = planning_pipeline
        self._plan_params.planner_id = planner_id
        self._plan_params.planning_time = planning_time
        self._plan_params.max_velocity_scaling_factor = max_velocity_scaling
        self._plan_params.max_acceleration_scaling_factor = max_acceleration_scaling

        # ---- ③ 夹爪 action / 规划场景 service（专用节点，按需调用时临时 spin）----
        self._gripper_node = Node(f"{node_name}_gripper")
        self._gripper_client = ActionClient(
            self._gripper_node, GripperCommand, "/gripper_action_controller/gripper_cmd"
        )
        self._scene_node = Node(f"{node_name}_scene")
        self._scene_client = None  # ApplyPlanningScene 客户端，首次用时创建
        self._carto_client = None  # GetCartesianPath 客户端，首次用时创建
        self._traj_client = None   # FollowJointTrajectory 客户端，首次用时创建

        self._log = self._tf_node.get_logger()
        self._log.info(
            f"ArmMover 就绪: planning_frame={self.planning_frame}, "
            f"pose_link={self.pose_link}"
        )

    # ---------------------------------------------------------------- 初始化辅助
    def _spin_tf(self):
        """TF 线程主体：rclpy.shutdown() 时静默退出，不刷 traceback。"""
        try:
            self._tf_executor.spin()
        except ExternalShutdownException:
            pass

    @staticmethod
    def _build_config(use_sim_time=True):
        """直接读 mycobot_moveit_config 的 yaml，装配 kinematics/规划管线参数。

        等价于 MoveItConfigsBuilder 的装配结果，但不 import 它——它的连锁
        依赖 launch→lark 在 conda python 下不可用。URDF/SRDF 仍从话题获取，
        故此处不含 robot_description。

        Jazzy 的两个参数名坑（与 Humble 不同，均经源码/实跑确认）：
        - pipeline 列表读嵌套参数 planning_pipelines.pipeline_names
        - 轨迹执行读 moveit_controller_manager / moveit_simple_controller_manager
          （不配则 execute 时找不到控制器，报 No controller_names specified）
        """
        import yaml

        from ament_index_python.packages import get_package_share_directory

        share = get_package_share_directory("mycobot_moveit_config")
        robot_cfg = os.path.join(share, "config", "mycobot_280")

        def load(path):
            with open(path, encoding="utf-8") as fh:
                return yaml.safe_load(fh)

        cfg = {
            "use_sim_time": bool(use_sim_time),
            "robot_description_kinematics": load(os.path.join(robot_cfg, "kinematics.yaml")),
            # 关节限速要挂在 robot_description_planning 前缀下才会被 RobotModel 读取
            # （顶层 joint_limits 键无效；TOTG 补轨迹时差加速度限值即此因）
            "robot_description_planning": load(os.path.join(robot_cfg, "joint_limits.yaml")),
            "planning_pipelines": {"pipeline_names": ["ompl"], "namespace": ""},
            "ompl": load(os.path.join(share, "config", "ompl_planning.yaml")),
            # Gazebo PID 对 joint3 等大扭矩关节的稳态误差可超过默认 0.01 rad，
            # 执行前校验会误报 "start point deviates from current robot state"
            "trajectory_execution": {"allowed_start_tolerance": 0.05},
        }
        # 控制器配置顶层两键直接并入（moveit_controllers.yaml 同 move_group 布局）
        cfg.update(load(os.path.join(robot_cfg, "moveit_controllers.yaml")))
        return cfg

    # ---------------------------------------------------------------- 属性
    @property
    def planning_frame(self):
        """规划坐标系（URDF 根，本机器人为 base_link）——动态获取不硬编码。"""
        return self._moveit.get_robot_model().model_frame

    # ---------------------------------------------------------------- TF
    def wait_for_frame(self, frame_id, timeout_sec=10.0):
        """阻塞等待 frame_id 出现在 TF 树（即视觉节点已上线），超时返回 False。"""
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self._tf_buffer.can_transform(self.planning_frame, frame_id, Time()):
                return True
            time.sleep(0.1)
        return False

    def lookup_frame_pose(self, frame_id, target_frame=None, timeout_sec=2.0):
        """查询 frame_id 在 target_frame（默认规划系）下的当前位姿。"""
        target = target_frame or self.planning_frame
        deadline = time.monotonic() + timeout_sec
        while True:
            try:
                t = self._tf_buffer.lookup_transform(target, frame_id, Time())
            except TransformException:
                if time.monotonic() >= deadline:
                    raise RuntimeError(f"TF 查询 {target}→{frame_id} 超时失败") from None
                time.sleep(0.1)
            else:
                ps = PoseStamped()
                ps.header.frame_id = t.header.frame_id
                ps.header.stamp = t.header.stamp
                ps.pose.position.x = t.transform.translation.x
                ps.pose.position.y = t.transform.translation.y
                ps.pose.position.z = t.transform.translation.z
                ps.pose.orientation = t.transform.rotation
                return ps

    def get_current_pose(self, link=None):
        """指定 link（默认 pose_link）在规划系下的当前位姿。"""
        return self.lookup_frame_pose(link or self.pose_link)

    # ---------------------------------------------------------------- 运动
    def _plan(self, component):
        """规划并抽出 RobotTrajectory（Jazzy 的 plan() 返回 MotionPlanResponse）。

        失败返回 None：plan() 对配置类错误抛异常、对不可达目标返回空响应。
        """
        try:
            resp = component.plan(self._plan_params)
        except Exception as e:
            self._log.warn(f"规划异常: {e}")
            return None
        traj = getattr(resp, "trajectory", None)
        if traj is None:
            code = getattr(resp, "error_code", resp)
            self._log.warn(f"规划失败: error_code={getattr(code, 'val', code)}")
            return None
        return traj

    def plan_to_pose(self, pose_stamped, link_name=None, start_configuration=None):
        """从当前状态（或 start_configuration 命名状态）规划到指定位姿。

        返回轨迹（失败返回 None），不执行。start_configuration 供 dry-run
        模拟实跑起点（如 "ready"）——不改真机状态，仅设规划起点；从 home
        直接规划竖直朝下位姿会 goal 采样饿死（KDL 已知坑）。
        """
        if not pose_stamped.header.frame_id:
            pose_stamped.header.frame_id = self.planning_frame
        if start_configuration is None:
            self._arm.set_start_state_to_current_state()
        else:
            self._arm.set_start_state(configuration_name=start_configuration)
        # Jazzy 版 moveit_py 没有 set_goal_pose，位姿目标走 set_goal_state 重载
        self._arm.set_goal_state(
            pose_stamped_msg=pose_stamped, pose_link=link_name or self.pose_link
        )
        return self._plan(self._arm)

    def plan_first_feasible(self, poses, link_name=None, start_configuration=None,
                            planning_time=None):
        """依次尝试位姿列表，返回第一个规划成功的 (轨迹, 位姿, 序号)；全败 (None, None, -1)。

        抓取朝向采样用（MTC GenerateGraspPose 的 plan-probe 等价物）：
        候选按优先级排序喂进来，哪个先规划出解就用哪个。
        planning_time 可临时缩短单候选规划时限（探测大量候选时防止
        不可达目标各烧满默认 5s）。
        """
        saved_time = self._plan_params.planning_time
        if planning_time is not None:
            self._plan_params.planning_time = float(planning_time)
        try:
            for i, pose in enumerate(poses):
                traj = self.plan_to_pose(pose, link_name, start_configuration)
                if traj is not None:
                    self._log.info(f"抓取候选 {i + 1}/{len(poses)} 规划成功")
                    return traj, pose, i
        finally:
            self._plan_params.planning_time = saved_time
        return None, None, -1

    def execute(self, trajectory):
        """执行 plan_to_pose 返回的轨迹（经 arm_controller 的 action）。"""
        if trajectory is None:
            return False
        try:
            status = self._moveit.execute(trajectory, controllers=[self.arm_controller_name])
            # ExecutionStatus 是 pybind 包装，字符串枚举名在 .status 属性里
            if str(status.status) != "SUCCEEDED":
                self._log.warn(f"轨迹执行状态: {status.status}")
                return False
            return True
        except Exception as e:
            self._log.error(f"轨迹执行失败: {e}")
            return False

    def move_to_pose(self, pose_stamped, link_name=None):
        """规划 + 执行，一步到位姿。"""
        traj = self.plan_to_pose(pose_stamped, link_name)
        if traj is None:
            self._log.warn("规划失败：目标不可达或姿态无解？")
            return False
        return self.execute(traj)

    def move_cartesian(self, target_pose, link_name=None, max_step=0.005,
                       fraction_min=0.95):
        """从当前位姿直线插补到 target_pose 并执行（抓取接近/退出段专用）。

        经 move_group 的 /compute_cartesian_path 服务插补：OMPL 自由空间路径
        在目标物附近可能甩动，直线段物理可控、碰撞沿路径逐点校验。
        返回 True = 插补覆盖率达标且执行成功。
        """
        if self._carto_client is None:
            # Jazzy 这套 move_group 的笛卡尔服务挂在根命名空间
            # （/compute_cartesian_path），旧名 /move_group/get_cartesian_path 兜底
            for name in ("/compute_cartesian_path", "/move_group/get_cartesian_path"):
                cand = self._scene_node.create_client(GetCartesianPath, name)
                if cand.wait_for_service(timeout_sec=1.0):
                    self._carto_client = cand
                    break
                self._scene_node.destroy_client(cand)
            if self._carto_client is None:
                self._log.error("compute_cartesian_path 服务不可达（move_group 在跑吗？）")
                return False
        req = GetCartesianPath.Request()
        req.header.frame_id = target_pose.header.frame_id or self.planning_frame
        req.group_name = self.arm_group
        req.link_name = link_name or self.pose_link
        req.waypoints = [target_pose.pose]
        req.max_step = float(max_step)
        req.jump_threshold = 10.0
        req.avoid_collisions = True
        req.max_velocity_scaling_factor = self._plan_params.max_velocity_scaling_factor
        req.max_acceleration_scaling_factor = self._plan_params.max_acceleration_scaling_factor
        future = self._carto_client.call_async(req)
        rclpy.spin_until_future_complete(self._scene_node, future, timeout_sec=10.0)
        if not future.done():
            self._log.error("get_cartesian_path 超时")
            return False
        resp = future.result()
        if resp.error_code.val != 1:  # MoveItErrorCodes::SUCCESS
            self._log.error(f"笛卡尔插补失败: error_code={resp.error_code.val}")
            return False
        if resp.fraction < fraction_min:
            self._log.warn(f"笛卡尔插补仅覆盖 {resp.fraction * 100:.0f}%，放弃执行")
            return False
        return self._execute_joint_trajectory(resp.solution.joint_trajectory)

    def _execute_joint_trajectory(self, joint_traj):
        """把关节轨迹直发 arm_controller 执行（绕过 MoveIt 执行前校验）。"""
        pts = joint_traj.points
        if not pts:
            self._log.error("空轨迹，无法执行")
            return False
        if pts[-1].time_from_start.sec == 0 and pts[-1].time_from_start.nanosec == 0:
            # 服务端未做时间参数化时按 0.2 rad/s 匀速补时间戳
            t, prev = 0.0, [0.0] * len(joint_traj.joint_names)
            for pt in pts:
                dmax = max((abs(c - p) for c, p in zip(pt.positions, prev)), default=0.0)
                t += dmax / 0.2
                pt.time_from_start.sec = int(t)
                pt.time_from_start.nanosec = int((t % 1) * 1e9)
                prev = list(pt.positions)
        if self._traj_client is None:
            self._traj_client = ActionClient(
                self._scene_node, FollowJointTrajectory,
                f"/{self.arm_controller_name}/follow_joint_trajectory"
            )
        if not self._traj_client.wait_for_server(timeout_sec=2.0):
            self._log.error("arm_controller 的 FollowJointTrajectory 服务不可达")
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = joint_traj
        future = self._traj_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self._scene_node, future, timeout_sec=5.0)
        if not future.done() or not future.result().accepted:
            self._log.warn("轨迹目标未被受理")
            return False
        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self._scene_node, result_future, timeout_sec=60.0)
        if not result_future.done():
            self._log.error("轨迹执行超时")
            return False
        error = result_future.result().result.error_code
        if error != FollowJointTrajectory.Result.SUCCESSFUL:
            self._log.error(f"轨迹执行错误码: {error}")
            return False
        return True

    def move_to_named(self, state_name, group="arm"):
        """运动到命名状态（home/ready，见 SRDF group_state）。"""
        component = self._arm if group == "arm" else self._gripper
        component.set_start_state_to_current_state()
        component.set_goal_state(configuration_name=state_name)
        try:
            traj = self._plan(component)
        except Exception as e:
            self._log.warn(f"move_to_named({state_name}) 规划异常: {e}")
            return False
        return self.execute(traj)

    def move_to_frame(self, frame_id, offset_xyz=(0.0, 0.0, 0.0), orientation_xyzw=None,
                      timeout_sec=2.0):
        """把 pose_link 移到某 TF frame 的位置 + 偏移（偏移沿规划系轴向），姿态默认竖直朝下。"""
        target = self.lookup_frame_pose(frame_id, timeout_sec=timeout_sec)
        goal = self.goal_pose(
            target.pose.position.x + offset_xyz[0],
            target.pose.position.y + offset_xyz[1],
            target.pose.position.z + offset_xyz[2],
            orientation_xyzw,
        )
        return self.move_to_pose(goal)

    def hover_and_descend(self, frame_id, hover_height=0.06,
                          descend_to_offset_xyz=(0.0, 0.0, 0.0), orientation_xyzw=None):
        """组合动作：先到抓取点正上方悬停，再竖直下降到位（抓取安全流程）。"""
        hover_offset = (
            descend_to_offset_xyz[0],
            descend_to_offset_xyz[1],
            descend_to_offset_xyz[2] + hover_height,
        )
        if not self.move_to_frame(frame_id, hover_offset, orientation_xyzw):
            return False
        return self.move_to_frame(frame_id, descend_to_offset_xyz, orientation_xyzw)

    # ---------------------------------------------------------------- 夹爪
    def move_gripper(self, state_name, timeout_sec=5.0):
        """执行夹爪命名状态（open/half_closed/closed），带超时保护。

        夹住物体时 GripperCommand 常达不到容差导致 goal 不返回——超时但
        目标已被接受时按"夹住受阻"处理，返回 True。
        """
        if state_name not in GRIPPER_POSITIONS:
            self._log.error(f"未知夹爪状态 {state_name}，可选: {list(GRIPPER_POSITIONS)}")
            return False
        if not self._gripper_client.wait_for_server(timeout_sec=2.0):
            self._log.error("夹爪 action 服务不可达（/gripper_action_controller/gripper_cmd）")
            return False

        goal = GripperCommand.Goal()
        # Jazzy 的 goal 是嵌套结构：command 才是 control_msgs/GripperCommand
        # 消息（直接 goal.position 会 AttributeError，被 os._exit 吞成静默退出）
        goal.command.position = GRIPPER_POSITIONS[state_name]
        goal.command.max_effort = 0.0
        future = self._gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self._gripper_node, future, timeout_sec=timeout_sec)
        if not future.done():
            self._log.warn("夹爪目标未被受理")
            return False
        handle = future.result()
        if not handle.accepted:
            self._log.warn("夹爪目标被拒绝")
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self._gripper_node, result_future, timeout_sec=timeout_sec)
        if not result_future.done():
            self._log.info("夹爪未到目标位（夹住物体受阻），按到位处理")
            handle.cancel_goal_async()
            return True
        result = result_future.result().result
        if result.reached_goal:
            return True
        if result.stalled:
            self._log.info("夹爪 stalled（夹住物体），按到位处理")
            return True
        self._log.warn(f"夹爪未到位: position={result.position:.3f}")
        return False

    def open_gripper(self):
        return self.move_gripper("open")

    def close_gripper(self):
        return self.move_gripper("closed")

    # ------------------------------------------------------- 附着搬运（MoveIt 层）
    def attach_box_to_ee(self, object_id, size_xyz, pose_stamped):
        """在规划场景中生成一个长方体并附着到夹爪（移植 attach_object_demo）。

        附着后 MoveIt 把它当作机械臂的一部分做碰撞检测，可以规划"搬运"
        轨迹。注意：这只影响 MoveIt/RViz，Gazebo 物理层不会因此真吸住物体。
        """
        obj = CollisionObject()
        obj.header.frame_id = pose_stamped.header.frame_id or self.planning_frame
        obj.id = object_id
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [float(s) for s in size_xyz]
        obj.primitives = [box]
        obj.primitive_poses = [pose_stamped.pose]
        obj.operation = CollisionObject.ADD

        aco = AttachedCollisionObject()
        aco.link_name = "gripper_base"
        aco.object = obj
        aco.touch_links = list(TOUCH_LINKS)

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state = RobotState()
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [aco]
        return self._apply_scene(scene)

    def detach_object(self, object_id):
        """把附着物体从夹爪摘下并从规划场景移除（物体会在 RViz 中消失）。"""
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state = RobotState()
        scene.robot_state.is_diff = True
        aco = AttachedCollisionObject()
        aco.object.id = object_id
        aco.object.operation = CollisionObject.REMOVE
        scene.robot_state.attached_collision_objects = [aco]
        world = CollisionObject()
        world.id = object_id
        world.operation = CollisionObject.REMOVE
        scene.world.collision_objects = [world]
        return self._apply_scene(scene)

    def add_cylinder(self, object_id, x, y, z_center, height, radius):
        """在规划场景中添加竖直圆柱碰撞体（视觉检测的物体），臂身路径将绕开它。"""
        obj = CollisionObject()
        obj.header.frame_id = self.planning_frame
        obj.id = object_id
        cyl = SolidPrimitive()
        cyl.type = SolidPrimitive.CYLINDER
        # dimensions 顺序：[CYLINDER_HEIGHT, CYLINDER_RADIUS]
        cyl.dimensions = [float(height), float(radius)]
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = x, y, z_center
        obj.primitives = [cyl]
        obj.primitive_poses = [pose]
        obj.operation = CollisionObject.ADD
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [obj]
        return self._apply_scene(scene)

    def remove_object(self, object_id):
        """从规划场景移除碰撞物体（物体不存在时也是合法操作）。"""
        obj = CollisionObject()
        obj.id = object_id
        obj.operation = CollisionObject.REMOVE
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [obj]
        return self._apply_scene(scene)

    def allow_collision(self, object_id, link_names):
        """放行 object_id 与 link_names 之间的碰撞检测（MTC allowCollisions 等价）。

        必须先读当前 ACM、合并新条目后整表回写——diff 场景里的 ACM 是
        整表替换语义，直接发只含新条目的表会抹掉机器人自碰撞放行，
        后果是所有规划全灭。
        """
        client = getattr(self, "_get_scene_client", None)
        if client is None:
            for name in ("/move_group/get_planning_scene", "/get_planning_scene"):
                cand = self._scene_node.create_client(GetPlanningScene, name)
                if cand.wait_for_service(timeout_sec=1.0):
                    client = cand
                    break
                self._scene_node.destroy_client(cand)
            if client is None:
                self._log.error("get_planning_scene 服务不可达（move_group 在跑吗？）")
                return False
            self._get_scene_client = client
        # Jazzy 坑：Request.components 是 PlanningSceneComponents 消息而非裸
        # uint32——直接赋整数会在 call_async 序列化时 C 层 assert 崩掉进程
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self._scene_node, future, timeout_sec=5.0)
        if not future.done():
            self._log.error("get_planning_scene 超时")
            return False
        acm = future.result().scene.allowed_collision_matrix
        names = list(acm.entry_names)
        allowed = set(link_names)
        if object_id not in names:
            for entry in acm.entry_values:
                entry.enabled.append(False)  # 旧行补新列，默认禁止
            row = AllowedCollisionEntry()
            row.enabled = [(n in allowed) for n in names] + [True]
            acm.entry_values.append(row)
            names.append(object_id)
        else:
            idx = names.index(object_id)
            for i, entry in enumerate(acm.entry_values):
                if names[i] in allowed:
                    entry.enabled[idx] = True
            row = acm.entry_values[idx]
            for i, n in enumerate(names):
                if n in allowed:
                    row.enabled[i] = True
        acm.entry_names = names
        scene = PlanningScene()
        scene.is_diff = True
        scene.allowed_collision_matrix = acm
        return self._apply_scene(scene)

    def _apply_scene(self, scene):
        if self._scene_client is None:
            self._scene_client = self._scene_node.create_client(
                ApplyPlanningScene, "/apply_planning_scene"
            )
        if not self._scene_client.wait_for_service(timeout_sec=5.0):
            self._log.error("/apply_planning_scene 服务不可达（move_group 在跑吗？）")
            return False
        req = ApplyPlanningScene.Request()
        req.scene = scene
        future = self._scene_client.call_async(req)
        rclpy.spin_until_future_complete(self._scene_node, future, timeout_sec=5.0)
        if not future.done():
            self._log.error("apply_planning_scene 超时")
            return False
        return future.result().success

    # ---------------------------------------------------------------- 内部
    def goal_pose(self, x, y, z, orientation_xyzw=None):
        """构造规划系下的目标位姿（默认竖直朝下），供外部直接喂给 move_to_pose。"""
        q = orientation_xyzw or DOWNWARD_QUAT
        goal = PoseStamped()
        goal.header.frame_id = self.planning_frame
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = x, y, z
        goal.pose.orientation.x, goal.pose.orientation.y = q[0], q[1]
        goal.pose.orientation.z, goal.pose.orientation.w = q[2], q[3]
        return goal

    # ---------------------------------------------------------------- 生命周期
    def shutdown(self):
        self._tf_executor.shutdown()
        self._tf_thread.join(timeout=2.0)
        self._gripper_client.destroy()
        if self._traj_client is not None:
            self._traj_client.destroy()
        if self._carto_client is not None:
            self._scene_node.destroy_client(self._carto_client)
        self._gripper_node.destroy_node()
        self._scene_node.destroy_node()
        self._tf_node.destroy_node()
        shutdown_moveit = getattr(self._moveit, "shutdown", None)
        if callable(shutdown_moveit):
            shutdown_moveit()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.shutdown()
        return False
