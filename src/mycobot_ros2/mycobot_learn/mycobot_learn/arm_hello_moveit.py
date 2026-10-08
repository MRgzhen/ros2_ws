"""演示①：第一个 MoveIt 程序（旧项目 mycobot_moveit_demos/hello_moveit 的
Python 等价物），用来验证 moveit_py 链路通不通。

流程：ready → base_link 系一个固定点位（竖直朝下）→ home。
不涉及视觉，跑通即可进行下一步 arm_move_to_object。

运行（先 robotm 启动 Gazebo+MoveIt）：
    ros2 run mycobot_learn arm_hello_moveit --ros-args -p use_sim_time:=true

验证（完成标准）：三段运动依次平滑执行，无碰撞报错，最后回 home。
"""

import os
import sys
import traceback

import rclpy

from mycobot_learn.arm_mover import ArmMover, DOWNWARD_QUAT


def main(args=None):
    rclpy.init(args=args)
    try:
        with ArmMover() as mover:
            print("== 1/3 运动到 ready ==")
            assert mover.move_to_named("ready"), "ready 段失败"

            print("== 2/3 运动到固定点位（臂前方 15cm、高 20cm、竖直朝下）==")
            goal = mover.goal_pose(0.15, 0.0, 0.20, DOWNWARD_QUAT)
            assert mover.move_to_pose(goal), "点位段失败"

            print("== 3/3 回 home ==")
            assert mover.move_to_named("home"), "home 段失败"
            print("全部完成 ✔")
    except Exception:
        # 必须先打出 traceback：finally 里的 os._exit(0) 会把异常静默吞掉
        traceback.print_exc()
    except KeyboardInterrupt:
        print("中断退出")
    finally:
        rclpy.shutdown()
        # moveit_py 的 C++ 析构与 rclpy 关闭顺序冲突会段错误（moveit2 已知问题），
        # 刷完缓冲直接退出，换取干净的退出码
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(0)


if __name__ == "__main__":
    main()
