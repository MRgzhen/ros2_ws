"""演示②：视觉引导抓取——移动到检测物体并夹取（完整流程）。

在 ArmMover 公共封装上跑通"视觉 → 运动 → 夹取"整条链路。目标来源单一：
等 /vision/detected_point 一条 PointStamped（base_link 系）——image_sub3
把 solvePnP 结果换算到 base_link 并做 calib_x/calib_y 校准后直接发布，
本节点不查 TF、不算校准（换 YOLO/SAM 时同名同格式发这个话题即可对接）。

流程（side，V2.1）：取目标 → 场景建障碍 → 原位张爪 → 从当前位置直接
规划到 pre-grasp（绕轴采样选向，不经 ready）→ 直线插补接近到抓取位 →
夹爪闭合 → 直线退回（→ 可选：附着物体并提起，MoveIt 层面）→ home。
top_down 仍走 ready → 悬停 → 竖直下降（V1，矮物体备选）。

抓取策略（grasp_mode）：
    side（默认，V2）水平侧抓，对齐旧 MTC demo 的几何：夹爪横着接近木棍、
        手指夹侧壁、棍顶从手腕旁边让开——0.35m 长棍唯一可行的抓法
        （顶抓时棍顶 z=0.35 必穿手腕，flange 天花板 0.22，物理无解）。
        接近方向默认 135°（base_link +y 与 -x 平分角，approach_yaw_deg
        =135：夹爪往左后方伸入、腕在木棍右前方），手指沿接近方向垂线
        对称跨棍。法兰目标位姿的构造——含 URDF 安装旋转修正、高度补偿
        （gripper_base 相对法兰转了 1.579 rad，不修正则手指竖直朝下、
        指尖戳到检测点下方）——统一收口在 grasp_geometry（腕下翻构型，
        可达性见其注释），本节点不算几何。
    top_down    竖直顶抓（V1，只适用于矮物体如 red_cylinder_short）：
        对准轴心下降，手指跨两侧夹上部侧壁。

运行（先 robotm 启动 Gazebo+MoveIt，另一终端跑 image_sub3）：
    ros2 run mycobot_learn arm_move_to_object --ros-args -p use_sim_time:=true
    # 先 dry-run 在 RViz 里查目标点和朝向，再实跑：
    ros2 run mycobot_learn arm_move_to_object --ros-args -p use_sim_time:=true \
        -p dry_run:=true
    ros2 run mycobot_learn arm_move_to_object --ros-args -p use_sim_time:=true \
        -p grasp_z_offset:=0.10

验证（完成标准）：① dry-run 指尖贴抓取点，side 手指水平指向棍 /
top_down 夹爪竖直朝下；② 实跑无碰撞，接近段平滑；③ 夹爪闭合停在圆柱侧壁。

标定提示：grasp_z_offset 用 tf2_echo base_link object_frame 读物体
中心高度后调；tcp_offset 是 flange 原点到指尖的距离（RViz 里标一次）；
视觉偏差校准 calib_x/calib_y 已挪到 image_sub3（跑视觉节点时传
-p calib_y:=-0.02），本节点拿到的 /vision/detected_point 已是校准后
的 base_link 位置。
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

from mycobot_learn.arm_mover import TOUCH_LINKS, ArmMover
from mycobot_learn.grasp_geometry import side_grasp_pair, top_down_pair


def main(args=None):
    rclpy.init(args=args)
    node = Node("arm_move_to_object")

    # ---------------- 参数 ----------------
    p = lambda name, default: node.declare_parameter(name, default).value
    point_timeout = float(p("point_timeout", 10.0))  # 等 /vision/detected_point
    grasp_mode = str(p("grasp_mode", "side"))  # side=V2 水平侧抓 / top_down=V1 顶抓
    approach_backoff = float(
        p("approach_backoff", 0.06)
    )  # 侧抓 pre-grasp 后退量（=直线接近段长度）
    # 接近方向（世界系方位角）：默认 135°=base_link +y 与 -x 平分角——
    # 夹爪沿该方向直线伸向木棍，腕在木棍右前方，手指沿接近方向垂线跨
    # 棍两侧。候选 yaw_offsets_deg 在此基础上微调。
    approach_yaw_deg = float(p("approach_yaw_deg", 135.0))
    # 相对 approach_yaw_deg 的微调候选（规划失败依次尝试）
    yaw_offsets_deg = str(p("yaw_offsets_deg", "0.0 -15.0 15.0 -30.0 30.0"))
    lift_height = float(p("lift_height", 0.10))  # lift_after_grasp 竖直上提量
    hover_height = float(p("hover_height", 0.06))  # 悬停点高于抓取点
    grasp_z_offset = float(p("grasp_z_offset", 0.0))  # 抓取点相对物体 z 偏移
    tcp_offset = float(p("tcp_offset", 0.10))  # flange 原点→指尖
    grasp_yaw = float(p("grasp_yaw", 0.0))  # 绕竖直轴微调（圆柱对称一般不动）
    max_reach = float(p("max_reach", 0.30))  # 工作空间护栏（留视觉 y 偏差余量）
    z_min = float(p("z_min", 0.03))
    z_max = float(p("z_max", 0.45))
    object_height = float(
        p("object_height", 0.35)
    )  # 场景圆柱尺寸（对齐 red_cylinder 模型）
    object_radius = float(p("object_radius", 0.015))
    scene_margin = float(
        p("scene_margin", 0.005)
    )  # 场景圆柱半径外扩：吸收视觉误差+轨迹跟踪偏差
    dry_run = bool(p("dry_run", False))
    do_grasp = bool(p("do_grasp", True))
    lift_after_grasp = bool(p("lift_after_grasp", False))  # 附着提起（MoveIt 层）
    attach_size = [float(v) for v in p("attach_size", [0.03, 0.03, 0.35])]

    try:
        with ArmMover() as mover:
            # ---------------- 1. 取目标：/vision/detected_point（base_link 系，视觉侧已校准） ----------------
            det = _wait_detected_point(node, point_timeout)
            if det is None:
                node.get_logger().error(
                    "等 /vision/detected_point 超时——视觉节点(image_sub3)在跑吗？"
                )
                return
            frame = det.header.frame_id or "base_link"
            if frame != "base_link":
                node.get_logger().warn(
                    f"检测点 frame 是 {frame}（应 base_link），目标可能偏移"
                )
            ox, oy, oz = det.point.x, det.point.y, det.point.z
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
            mover.add_cylinder(
                scene_obj,
                ox,
                oy,
                object_height / 2.0,
                object_height,
                object_radius + scene_margin,
            )
            mover.allow_collision(scene_obj, TOUCH_LINKS)

            # ---------------- 3. 抓取位姿候选（构造收口在 grasp_geometry） ----------------
            if grasp_mode == "side":
                # V2.4 水平侧抓：接近方向从 approach_yaw_deg 起采样
                # （默认 135°），沿"腕→物体"水平直线接近
                base_yaw = math.radians(approach_yaw_deg)
                offsets = [math.radians(float(s)) for s in yaw_offsets_deg.split()]
                pregrasp_goals, grasp_goals, yaws = [], [], []
                for dyaw in offsets:
                    yaw = base_yaw + dyaw
                    (pre_xyz, pre_q), (gr_xyz, gr_q) = side_grasp_pair(
                        ox, oy, grasp_z, yaw, tcp_offset, approach_backoff
                    )
                    grasp_goals.append(mover.goal_pose(*gr_xyz, gr_q))
                    pregrasp_goals.append(mover.goal_pose(*pre_xyz, pre_q))
                    yaws.append(yaw)
                descend_goal = hover_goal = None
            else:
                (hov_xyz, hov_q), (des_xyz, des_q) = top_down_pair(
                    ox, oy, grasp_z, tcp_offset, hover_height, math.radians(grasp_yaw)
                )
                descend_goal = mover.goal_pose(*des_xyz, des_q)
                hover_goal = mover.goal_pose(*hov_xyz, hov_q)

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
                node.get_logger().info(
                    "2/3 从当前位置直接规划 → pre-grasp（正对方向为主）"
                )
                traj, _, idx = mover.plan_first_feasible(pregrasp_goals)
                if traj is None:
                    node.get_logger().error(
                        f"所有接近方向均不可达（候选 {yaw_offsets_deg}），"
                        "试 -p approach_yaw_deg:=-45.0 换另一侧，或把物体放近一点"
                    )
                    return
                node.get_logger().info(
                    f"选用接近方向 yaw={math.degrees(yaws[idx]):.1f}°（第 {idx + 1} 候选）"
                )
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
                if not (
                    mover.move_to_pose(hover_goal) and mover.move_to_pose(descend_goal)
                ):
                    return
                grasp_goal_final = descend_goal

            if do_grasp:
                node.get_logger().info("闭合夹爪")
                mover.close_gripper()
                if grasp_mode == "side":
                    node.get_logger().info("直线退回 pre-grasp（靠摩擦带住木棍）")
                    if not mover.move_cartesian(pregrasp_goals[idx]):
                        node.get_logger().warn("直线退回失败，尝试 OMPL 兜底")
                        if not mover.move_to_pose(pregrasp_goals[idx]):
                            # 夹着棍子横扫回 home 会把工作区掀了——停在原地更安全
                            node.get_logger().error("退回失败，停在原地不回 home")
                            return

            if lift_after_grasp:
                node.get_logger().info("附加物体并竖直提起（MoveIt 层搬运）")
                obj_pose = mover.goal_pose(ox, oy, grasp_z)  # 物体中心（附着体位置）
                mover.attach_box_to_ee("grasped_object", attach_size, obj_pose)
                # 侧抓已退到 pre-grasp，以它为基准竖直上提；顶抓从抓取位提
                base_pose = (
                    pregrasp_goals[idx] if grasp_mode == "side" else grasp_goal_final
                )
                bp = base_pose.pose.position
                bo = base_pose.pose.orientation
                mover.move_cartesian(
                    mover.goal_pose(
                        bp.x, bp.y, bp.z + lift_height, (bo.x, bo.y, bo.z, bo.w)
                    )
                )
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
    """等 /vision/detected_point 一条消息，返回整条 PointStamped，超时返回 None。"""
    got = {}

    def cb(msg: PointStamped):
        got["msg"] = msg

    node.create_subscription(
        PointStamped, "/vision/detected_point", cb, QoSProfile(depth=1)
    )
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline and "msg" not in got:
        rclpy.spin_once(node, timeout_sec=0.1)
    return got["msg"] if "msg" in got else None


if __name__ == "__main__":
    main()
