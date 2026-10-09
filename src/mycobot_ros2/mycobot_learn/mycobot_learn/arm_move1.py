"""演示③：定点抓取——给 base_link 系目标位置，机械臂移动过去夹取。

arm_move_to_object 的去视觉简化版：目标点直接从参数读（不等 TF/话题），
流程收敛为竖直顶抓一条链路，适合快速验证 MoveIt 链路、标定抓取高度
和 tcp_offset。

流程：ready → 张爪 → 悬停（抓取点上方 hover_height）→ 竖直直线下降
→ 闭爪 → 回 home。顶抓规划从 ready 起步，规避 KDL 严格竖直姿态的
"no valid states" 坑（见 arm_move_to_object 文件头已知坑）。

运行（先 robotm 启动 Gazebo+MoveIt）：
    ros2 run mycobot_learn arm_move1 --ros-args -p use_sim_time:=true \
        -p target_x:=0.15 -p target_y:=0.0 -p target_z:=0.05
    # 先 dry-run 在 RViz 里查悬停点轨迹，再实跑：
    ros2 run mycobot_learn arm_move1 --ros-args -p use_sim_time:=true \
        -p target_x:=0.15 -p target_z:=0.05 -p dry_run:=true

注意：竖直顶抓只适合矮物体（如 red_cylinder_short）；0.35m 长棍顶抓
时棍顶必穿手腕，物理无解，那种情况用 arm_move_to_object 的 side 侧抓。
"""

import math
import os
import sys
import traceback

import rclpy
from rclpy.node import Node

from mycobot_learn.arm_mover import ArmMover


def main(args=None):
    rclpy.init(args=args)
    node = Node("arm_move1")

    # ---------------- 参数 ----------------
    p = lambda name, default: node.declare_parameter(name, default).value
    target_x = float(p("target_x", 0.15))  # 抓取点（base_link 系，指尖要到的位置）
    target_y = float(p("target_y", 0.0))
    target_z = float(p("target_z", 0.05))
    tcp_offset = float(p("tcp_offset", 0.10))  # flange 原点→指尖

    max_reach = float(p("max_reach", 0.30))  # 工作空间护栏
    z_min = float(p("z_min", 0.03))
    z_max = float(p("z_max", 0.45))
    dry_run = bool(p("dry_run", False))
    do_grasp = bool(p("do_grasp", True))

    try:
        with ArmMover() as mover:
            # ---------------- 1. 工作空间护栏 ----------------
            radius = math.hypot(target_x, target_y)
            if radius > max_reach or not z_min <= target_z <= z_max:
                node.get_logger().error(
                    f"抓取点 (r={radius:.3f}m, z={target_z:.3f}m) 超出护栏"
                    f"（r≤{max_reach}, {z_min}≤z≤{z_max}）。"
                    "把目标放机械臂正前方 15-20cm 处再试。"
                )
                return

            # ---------------- 2. 目标位姿 ----------------
            # 目标是"指尖到抓取点"：flange 再抬高 tcp_offset，姿态默认竖直朝下
            tool_z = target_z + tcp_offset
            hover_goal = mover.goal_pose(target_x, target_y, tool_z)

            if dry_run:
                traj = mover.plan_to_pose(hover_goal, start_configuration="ready")
                node.get_logger().info(
                    "dry-run 规划成功，请在 RViz 查看悬停点轨迹终点"
                    if traj is not None
                    else "dry-run 规划失败：目标不可达或姿态无解"
                )
                return

            # ---------------- 3. 抓取流程 ----------------
            node.get_logger().info(
                f"1/5 运动到 ready（目标 ({target_x:.3f}, {target_y:.3f}, {target_z:.3f}) m）"
            )
            if not mover.move_to_named("ready"):
                return
            node.get_logger().info("2/5 张开夹爪")
            mover.open_gripper()
            node.get_logger().info("3/5 悬停到抓取点上方")
            if not mover.move_to_pose(hover_goal):
                return
            node.get_logger().info("4/5 竖直下降到抓取点")
            if do_grasp:
                node.get_logger().info("5/5 闭合夹爪")
                mover.close_gripper()

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


if __name__ == "__main__":
    main()
