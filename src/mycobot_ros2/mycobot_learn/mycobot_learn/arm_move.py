"""hello_moveit.cpp（mycobot_moveit_demos）的 Python 等价物，面向对象版。

ArmMove 类封装 MoveItPy，按职责分方法：规划 / 移动 / 夹爪。
不依赖本包其他模块，只用 moveit_py 官方 API。与 C++ 原文的对应：
  MoveGroupInterface(node, "arm") → ArmMove 内的 PlanningComponent("arm")
  setPlanningPipelineId / setPlannerId / setPlanningTime / 速度加速度缩放
    → PlanRequestParameters（Jazzy 的 plan() 走参数对象，无单独 setter）
  setPoseTarget → set_goal_state（Jazzy 版 moveit_py 无 set_goal_pose）
  plan + execute → PlanningComponent.plan + MoveItPy.execute

运行（先 robotm 启动 Gazebo+MoveIt）：
    ros2 run mycobot_learn arm_move
"""

import os
import sys
import traceback
from time import sleep

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import GripperCommand
from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy, PlanRequestParameters
from moveit.utils import create_params_file_from_dict
from rclpy.action import ActionClient
from rclpy.clock import Clock

# sim 时钟必须写进参数文件：命令行 --ros-args 只作用于 rclpy 节点，
# moveit_cpp 内部 rclcpp 节点看不到，时钟不一致会导致执行前状态校验失败
USE_SIM_TIME = True

# 夹爪命名状态 → gripper_controller 关节位置（数值来自 SRDF group_state）
GRIPPER_POSITIONS = {"open": 0.0, "half_closed": -0.34, "closed": -0.50}


def build_moveit_params(use_sim_time=USE_SIM_TIME):
    """读 mycobot_moveit_config 的 yaml，装配 planning pipeline 等参数。

    MoveItPy 裸构造只能从话题拿 URDF/SRDF，拿不到 kinematics/规划管线参数
    （它们只在 move_group 节点的参数服务器上），会报
    "Failed to load planning pipelines"，故必须显式装配。
    """
    share = get_package_share_directory("mycobot_moveit_config")
    robot_cfg = os.path.join(share, "config", "mycobot_280")

    def load(path):
        with open(path, encoding="utf-8") as fh:
            return yaml.safe_load(fh)

    cfg = {
        "use_sim_time": bool(use_sim_time),
        "robot_description_kinematics": load(
            os.path.join(robot_cfg, "kinematics.yaml")
        ),
        # 关节限速要挂在 robot_description_planning 前缀下才会被 RobotModel 读取
        "robot_description_planning": load(
            os.path.join(robot_cfg, "joint_limits.yaml")
        ),
        "planning_pipelines": {"pipeline_names": ["ompl"], "namespace": ""},
        "ompl": load(os.path.join(share, "config", "ompl_planning.yaml")),
        # Gazebo PID 对大扭矩关节的稳态误差可超默认 0.01 rad，
        # 放大容差防执行前校验误报 "start point deviates from current state"
        "trajectory_execution": {"allowed_start_tolerance": 0.05},
    }
    # 控制器配置顶层两键直接并入（moveit_controllers.yaml 同 move_group 布局）
    cfg.update(load(os.path.join(robot_cfg, "moveit_controllers.yaml")))
    return create_params_file_from_dict(cfg, "/**")


class ArmMove:
    """MoveItPy 机械臂封装：规划 / 移动 / 夹爪。

    自包含实现，不依赖本包其他模块。前提：调用方先 rclpy.init()。
    """

    def __init__(
        self,
        *,
        node_name="arm_move",
        planning_pipeline="ompl",
        planner_id="RRTConnectkConfigDefault",
        planning_time=1.0,
        max_velocity_scaling=1.0,
        max_acceleration_scaling=1.0,
        use_sim_time=USE_SIM_TIME,
    ):
        self._moveit = MoveItPy(
            node_name=node_name,
            launch_params_filepaths=[build_moveit_params(use_sim_time)],
        )
        self._arm = self._moveit.get_planning_component("arm")
        self._gripper = self._moveit.get_planning_component("gripper")

        # 规划参数（pybind 类必须传 MoveItPy 实例构造，标量属性再逐个赋值）
        self._plan_params = PlanRequestParameters(self._moveit, planning_pipeline)
        self._plan_params.planning_pipeline = planning_pipeline
        self._plan_params.planner_id = planner_id
        self._plan_params.planning_time = planning_time
        self._plan_params.max_velocity_scaling_factor = max_velocity_scaling
        self._plan_params.max_acceleration_scaling_factor = max_acceleration_scaling

        # 节点名不能与 MoveItPy 内部节点同名，否则 rosout 报
        # "Publisher already registered for node name"
        self._logger = rclpy.create_node(f"{node_name}_log").get_logger()

    # ---------------------------------------------------------------- 规划
    def plan_to_pose(self, pose_stamped, pose_link="gripper_base"):
        """从当前状态规划到指定位姿，返回轨迹（失败 None，不执行）。"""
        self._arm.set_start_state_to_current_state()
        # Jazzy 版 moveit_py 没有 set_goal_pose，位姿目标走 set_goal_state
        self._arm.set_goal_state(pose_stamped_msg=pose_stamped, pose_link=pose_link)
        return self._plan(self._arm, "臂位姿")

    def _plan(self, component, desc):
        """规划并抽出轨迹（MotionPlanResponse 里 trajectory 为空即失败）。"""
        resp = component.plan(self._plan_params)
        traj = getattr(resp, "trajectory", None)
        if traj is None:
            code = getattr(resp, "error_code", resp)
            self._logger.error(
                f"{desc}规划失败！error_code={getattr(code, 'val', code)}"
            )
        return traj

    def _execute(self, traj, controller, desc):
        """经指定控制器执行轨迹，按执行状态返回布尔。"""
        status = self._moveit.execute(traj, controllers=[controller])
        if str(status.status) != "SUCCEEDED":
            self._logger.warning(f"{desc}执行状态: {status.status}")
            return False
        return True

    # ---------------------------------------------------------------- 移动
    def move_to_pose(self, pose_stamped, pose_link="gripper_base"):
        """规划 + 执行，一步到指定位姿。"""
        traj = self.plan_to_pose(pose_stamped, pose_link)
        if traj is None:
            return False
        return self._execute(traj, "arm_controller", "臂运动")

    def move_to_state(self, state_name):
        """规划 + 执行，一步到臂命名状态（home / ready）。"""
        self._arm.set_start_state_to_current_state()
        self._arm.set_goal_state(configuration_name=state_name)
        traj = self._plan(self._arm, f"臂到 {state_name}")
        if traj is None:
            return False
        return self._execute(traj, "arm_controller", f"臂到 {state_name}")

    # ---------------------------------------------------------------- 夹爪
    def move_gripper(self, state_name):
        """夹爪到命名状态（open / half_closed / closed）。"""
        self._gripper.set_start_state_to_current_state()
        self._gripper.set_goal_state(configuration_name=state_name)
        traj = self._plan(self._gripper, f"夹爪 {state_name}")
        if traj is None:
            return False
        return self._execute(traj, "gripper_action_controller", f"夹爪 {state_name}")

    # ---------------------------------------------------------------- 其他
    def log_plan_config(self):
        """打印当前规划配置（对应 C++ 原文的三条 INFO 日志）。"""
        self._logger.info(f"Planning pipeline: {self._plan_params.planning_pipeline}")
        self._logger.info(f"Planner ID: {self._plan_params.planner_id}")
        self._logger.info(f"Planning time: {self._plan_params.planning_time:.2f}")


def main():
    rclpy.init()
    try:
        robot = ArmMove()
        robot.log_plan_config()
        robot.move_gripper("open")

        target = PoseStamped()
        target.header.frame_id = "base_link"
        target.header.stamp = Clock().now().to_msg()
        target.pose.position.x = 0.061
        target.pose.position.y = -0.176
        target.pose.position.z = 0.168
        target.pose.orientation.x = 1.0
        target.pose.orientation.y = 0.0
        target.pose.orientation.z = 0.0
        target.pose.orientation.w = 0.0
        if not robot.move_to_pose(target):
            print("移动到目标位姿失败 ❌")
            robot.move_to_state("home")
            return
        print("移动到目标位姿 ✔")
        sleep(3)

        robot.move_gripper("closed")
        print("夹爪闭合 ✔")
        sleep(3)

        robot.move_to_state("home")
        print("全部完成 ✔")
        sleep(3)
    except KeyboardInterrupt:
        print("中断退出")
    except Exception:
        # 必须先打出 traceback：finally 里的 os._exit(0) 会把异常静默吞掉
        traceback.print_exc()
    finally:
        rclpy.shutdown()
        # moveit_py 的 C++ 析构与 rclpy 关闭顺序冲突会段错误（moveit2 已知问题），
        # 刷完缓冲直接退出，换取干净的退出码
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(0)


if __name__ == "__main__":
    main()
