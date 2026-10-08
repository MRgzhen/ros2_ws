"""演示②：视觉引导抓取——移动到检测物体并夹取（完整流程）。

在 ArmMover 公共封装上跑通"视觉 → 运动 → 夹取"整条链路。视觉侧对接
两种来源（参数 target_source）：
    tf    等 TF 里出现 object_frame（image_sub3 / 未来的 YOLO、SAM 同名广播即可）
    topic 等 /vision/detected_point 一条 PointStamped（base_link 系，旧 color_detector 习惯）

流程（side，V2.1）：取目标 → 场景建障碍 → 原位张爪 → 从当前位置直接
规划到 pre-grasp（绕轴采样选向，不经 ready）→ 直线插补接近到抓取位 →
夹爪闭合 → 直线退回（→ 可选：附着物体并提起，MoveIt 层面）→ home。
top_down 仍走 ready → 悬停 → 竖直下降（V1，矮物体备选）。

抓取策略（grasp_mode）：
    side（默认，V2）水平侧抓，对齐旧 MTC demo 的几何：夹爪横着接近木棍、
        手指夹侧壁、棍顶从手腕旁边让开——0.35m 长棍唯一可行的抓法
        （顶抓时棍顶 z=0.35 必穿手腕，flange 天花板 0.22，物理无解）。
        接近方向默认沿 base +X 轴（approach_yaw_deg=0，腕在棍子靠臂
        一侧、从机械臂与木棍之间直线进入，腕 r≈0.14 避开基座死区与
        棍顶），手指沿 ±y 对称跨棍。侧抓姿态 q = Rz(yaw+90°)·Rx(90°)：
        手 z 水平指向棍（approach）、手 x 水平⊥approach（手指闭合方向，
        由夹爪 URDF 手指关节轴全为 z、闭合沿 x 推出）、手 y 竖直。
    top_down    竖直顶抓（V1，只适用于矮物体如 red_cylinder_short）：
        对准轴心下降，手指跨两侧夹上部侧壁。

运行（先 robotm 启动 Gazebo+MoveIt，另一终端跑 image_sub3）：
    ros2 run mycobot_learn arm_move_to_object --ros-args -p use_sim_time:=true
    # 先 dry-run 在 RViz 里查目标点和朝向，再实跑：
    ros2 run mycobot_learn arm_move_to_object --ros-args -p use_sim_time:=true \
        -p dry_run:=true
    ros2 run mycobot_learn arm_move_to_object --ros-args -p use_sim_time:=true \
        -p grasp_z_offset:=0.10

验证（完成标准）：① dry-run 轨迹终点在木棒轴心 ±1cm、夹爪竖直朝下；
② 实跑无碰撞，悬停→下降平滑；③ 夹爪闭合停在圆柱侧壁。

标定提示：grasp_z_offset 用 tf2_echo base_link object_frame 读物体
中心高度后调；tcp_offset 是 flange 原点到指尖的距离（RViz 里标一次）；
calib_x/calib_y 修视觉系统偏差（tf2_echo 读数对比 Gazebo 真值，实测
y 偏高 ~2cm，跑时传 -p calib_y:=-0.02）。
已知坑（旧 C++ move_to_point 踩过）：严格竖直姿态下 KDL 可能自碰撞导致
"no valid states for goal tree"——本节点默认从 ready（已竖直朝下）出发
规避；仍失败时试调 grasp_yaw 或加大 planning_time。
"""

import math
import os
import sys
import time
import traceback

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile
from scipy.spatial.transform import Rotation

from mycobot_learn.arm_mover import ArmMover, DOWNWARD_QUAT, TOUCH_LINKS


def main(args=None):
    rclpy.init(args=args)
    node = Node("arm_move_to_object")

    # ---------------- 参数 ----------------
    p = lambda name, default: node.declare_parameter(name, default).value
    target_source = str(p("target_source", "tf"))            # tf / topic
    object_frame = str(p("object_frame", "object_frame"))    # 与 image_sub3 一致
    tf_timeout = float(p("tf_timeout", 10.0))
    grasp_mode = str(p("grasp_mode", "side"))                # side=V2 水平侧抓 / top_down=V1 顶抓
    approach_backoff = float(p("approach_backoff", 0.06))    # 侧抓 pre-grasp 后退量（=直线接近段长度）
    # 接近方向（世界系方位角）：默认 0°=沿 base +X 轴夹——腕在棍子靠臂
    # 一侧（正 -x 向），夹爪朝 +x 直线伸入，手指沿 ±y 跨棍两侧；腕 r≈0.14
    # 避开基座死区（径向方案腕 r≈0.05 压基座，KDL 饿死）。
    approach_yaw_deg = float(p("approach_yaw_deg", 0.0))
    # 相对 approach_yaw_deg 的微调候选（规划失败依次尝试）
    yaw_offsets_deg = str(p("yaw_offsets_deg", "0.0 -15.0 15.0 -30.0 30.0"))
    lift_height = float(p("lift_height", 0.10))              # lift_after_grasp 竖直上提量
    hover_height = float(p("hover_height", 0.06))            # 悬停点高于抓取点
    grasp_z_offset = float(p("grasp_z_offset", 0.0))         # 抓取点相对物体 z 偏移
    tcp_offset = float(p("tcp_offset", 0.10))                # flange 原点→指尖
    grasp_yaw = float(p("grasp_yaw", 0.0))                   # 绕竖直轴微调（圆柱对称一般不动）
    max_reach = float(p("max_reach", 0.30))                  # 工作空间护栏（留视觉 y 偏差余量）
    z_min = float(p("z_min", 0.03))
    z_max = float(p("z_max", 0.45))
    calib_x = float(p("calib_x", 0.0))                       # 视觉偏差校准（TF 读数 + 此项）
    calib_y = float(p("calib_y", 0.0))                       # 实测 y 系统性偏高 ~+0.02，传 -0.02
    object_height = float(p("object_height", 0.35))          # 场景圆柱尺寸（对齐 red_cylinder 模型）
    object_radius = float(p("object_radius", 0.015))
    scene_margin = float(p("scene_margin", 0.02))            # 场景圆柱半径外扩：吸收视觉误差+轨迹跟踪偏差
    dry_run = bool(p("dry_run", False))
    do_grasp = bool(p("do_grasp", True))
    lift_after_grasp = bool(p("lift_after_grasp", False))    # 附着提起（MoveIt 层）
    attach_size = [float(v) for v in p("attach_size", [0.03, 0.03, 0.35])]

    # 目标姿态 = 绕 z 转 grasp_yaw × 竖直朝下
    q = (Rotation.from_euler("z", grasp_yaw) * Rotation.from_quat(DOWNWARD_QUAT)).as_quat()

    try:
        with ArmMover() as mover:
            # ---------------- 1. 取目标 ----------------
            if target_source == "topic":
                point = _wait_detected_point(node, tf_timeout)
                if point is None:
                    node.get_logger().error("等 /vision/detected_point 超时，退出")
                    return
                ox, oy, oz = point.x, point.y, point.z
            else:
                if not mover.wait_for_frame(object_frame, tf_timeout):
                    node.get_logger().error(
                        f"等 TF {object_frame} 超时——视觉节点(image_sub3)在跑吗？"
                    )
                    return
                obj = mover.lookup_frame_pose(object_frame)
                pos = obj.pose.position
                # 视觉校准：solvePnP 的 y 系统性偏高（spawn y=0 时 TF 读 +0.015~+0.023）
                ox, oy, oz = pos.x + calib_x, pos.y + calib_y, pos.z
            node.get_logger().info(f"目标物体位置: ({ox:.3f}, {oy:.3f}, {oz:.3f}) m")

            # ---------------- 2. 工作空间护栏 ----------------
            radius = math.hypot(ox, oy)
            grasp_z = oz + grasp_z_offset
            if radius > max_reach or not z_min <= grasp_z <= z_max:
                node.get_logger().error(
                    f"抓取点 (r={radius:.3f}m, z={grasp_z:.3f}m) 超出护栏"
                    f"（r≤{max_reach}, {z_min}≤z≤{z_max}）。"
                    "把物体放机械臂正前方 15-20cm 处再试。"
                )
                return

            # ---------------- 2.5 目标物体进规划场景 ----------------
            # 物体只在 Gazebo 里，MoveIt 不知道它存在，自由空间路径会横扫
            # 过去蹭倒它。按视觉 xy（含校准）加竖直圆柱做障碍，并放行夹爪
            # 各 link 与它接触（MTC allowCollisions 同款）——臂身绕行、手指
            # 可跨。先移除再添加，避免上次运行残留的旧位置挡路。
            scene_obj = "grasp_target"
            mover.remove_object(scene_obj)
            mover.add_cylinder(scene_obj, ox, oy, object_height / 2.0,
                               object_height, object_radius + scene_margin)
            mover.allow_collision(scene_obj, TOUCH_LINKS)

            # ---------------- 3. 抓取位姿候选 ----------------
            if grasp_mode == "side":
                # V2.4 水平侧抓：接近方向固定沿 base +X 轴（腕在棍子靠臂
                # 一侧），沿"腕→物体"水平直线接近
                base_yaw = math.radians(approach_yaw_deg)
                offsets = [math.radians(float(s)) for s in yaw_offsets_deg.split()]
                pregrasp_goals, grasp_goals, yaws = [], [], []
                for dyaw in offsets:
                    yaw = base_yaw + dyaw
                    # 侧抓姿态（推导见文件头）：手 z 水平指向物体、手 x 水平⊥approach
                    q_side = (Rotation.from_euler("z", yaw + math.pi / 2)
                              * Rotation.from_quat(DOWNWARD_QUAT)).as_quat()
                    approach = (math.cos(yaw), math.sin(yaw), 0.0)
                    # TCP 对准物体中心：flange = 抓取点 - tcp_offset·approach
                    gx = ox - tcp_offset * approach[0]
                    gy = oy - tcp_offset * approach[1]
                    grasp_goals.append(mover.goal_pose(gx, gy, grasp_z, tuple(q_side)))
                    pregrasp_goals.append(mover.goal_pose(
                        gx - approach_backoff * approach[0],
                        gy - approach_backoff * approach[1], grasp_z, tuple(q_side)))
                    yaws.append(yaw)
                descend_goal = hover_goal = None
            else:
                tool_z = grasp_z + tcp_offset  # 目标是"指尖到抓取点"，flange 再抬高 tcp_offset
                descend_goal = mover.goal_pose(ox, oy, tool_z, tuple(q))
                hover_goal = mover.goal_pose(ox, oy, tool_z + hover_height, tuple(q))

            if dry_run:
                # side 与实跑同源：从当前状态直接探测（不再经 ready 中转）
                if grasp_mode == "side":
                    traj, _, _ = mover.plan_first_feasible(grasp_goals)
                else:
                    traj = mover.plan_to_pose(descend_goal, start_configuration="ready")
                node.get_logger().info(
                    "dry-run 规划成功，请在 RViz 查看轨迹终点与朝向"
                    if traj is not None
                    else "dry-run 规划失败：目标不可达或姿态无解"
                )
                return

            # ---------------- 4. 执行抓取流程 ----------------
            if grasp_mode == "side":
                # V2.1：不经 ready 大翻身，从当前位直接规划到 pre-grasp
                # （ready→侧抓的大翻身是 OMPL 甩上去绕行的路径，会扫过棍顶）
                node.get_logger().info("1/3 张开夹爪（原位）")
                mover.open_gripper()
                node.get_logger().info("2/3 从当前位置直接规划 → pre-grasp（正对方向为主）")
                traj, _, idx = mover.plan_first_feasible(pregrasp_goals)
                if traj is None:
                    node.get_logger().error(
                        f"所有接近方向均不可达（候选 {yaw_offsets_deg}），"
                        "试 -p approach_yaw_deg:=-90.0 换另一侧，或把物体放近一点"
                    )
                    return
                node.get_logger().info(
                    f"选用接近方向 yaw={math.degrees(yaws[idx]):.1f}°（第 {idx + 1} 候选）")
                if not mover.execute(traj):
                    return
                node.get_logger().info("3/3 直线接近到抓取位（夹棍侧壁）")
                if not mover.move_cartesian(grasp_goals[idx]):
                    node.get_logger().warn("直线接近失败，尝试 OMPL 兜底")
                    if not mover.move_to_pose(grasp_goals[idx]):
                        return
                grasp_goal_final = grasp_goals[idx]
            else:
                node.get_logger().info("1/5 运动到 ready")
                if not mover.move_to_named("ready"):
                    return
                node.get_logger().info("2/5 张开夹爪")
                mover.open_gripper()
                node.get_logger().info("3/5 悬停 → 下降到抓取点")
                node.get_logger().info("3/5 悬停 → 下降到抓取点")
                if target_source == "tf":
                    # hover_and_descend 内部会重新查 TF，xy 校准要随偏移一起传入
                    ok = mover.hover_and_descend(
                        object_frame, hover_height,
                        (calib_x, calib_y, grasp_z_offset + tcp_offset), tuple(q)
                    )
                else:
                    ok = mover.move_to_pose(hover_goal) and mover.move_to_pose(descend_goal)
                if not ok:
                    return
                grasp_goal_final = descend_goal

            if do_grasp:
                node.get_logger().info("闭合夹爪")
                mover.close_gripper()
                if grasp_mode == "side":
                    node.get_logger().info("直线退回 pre-grasp（靠摩擦带住木棍）")
                    mover.move_cartesian(pregrasp_goals[idx])

            if lift_after_grasp:
                node.get_logger().info("附加物体并竖直提起（MoveIt 层搬运）")
                obj_pose = mover.goal_pose(ox, oy, grasp_z)  # 物体中心（附着体位置）
                mover.attach_box_to_ee("grasped_object", attach_size, obj_pose)
                # 侧抓已退到 pre-grasp，以它为基准竖直上提；顶抓从抓取位提
                base_pose = pregrasp_goals[idx] if grasp_mode == "side" else grasp_goal_final
                bp = base_pose.pose.position
                bo = base_pose.pose.orientation
                mover.move_cartesian(mover.goal_pose(
                    bp.x, bp.y, bp.z + lift_height,
                    (bo.x, bo.y, bo.z, bo.w)))
                mover.detach_object("grasped_object")

            node.get_logger().info("回 home")
            mover.move_to_named("home")
    except Exception:
        # 必须先打出 traceback：finally 里的 os._exit(0) 会把异常静默吞掉
        traceback.print_exc()
    except KeyboardInterrupt:
        print("中断退出")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        # moveit_py 的 C++ 析构与 rclpy 关闭顺序冲突会段错误（moveit2 已知问题），
        # 刷完缓冲直接退出，换取干净的退出码
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(0)


def _wait_detected_point(node, timeout_sec):
    """等 /vision/detected_point 一条消息，返回 point 部分，超时返回 None。"""
    got = {}

    def cb(msg: PointStamped):
        got["msg"] = msg

    node.create_subscription(PointStamped, "/vision/detected_point", cb, QoSProfile(depth=1))
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline and "msg" not in got:
        rclpy.spin_once(node, timeout_sec=0.1)
    return got["msg"].point if "msg" in got else None


if __name__ == "__main__":
    main()
