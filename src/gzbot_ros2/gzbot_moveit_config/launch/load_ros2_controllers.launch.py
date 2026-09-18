# ==================== import 速查表 ====================
# 常用基础
from launch import LaunchDescription
from launch_ros.actions import Node
# 定时触发 / 事件联动（控制器按顺序接力启动）
from launch.actions import TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
# 路径拼接
from launch.substitutions import PathJoinSubstitution
# 获取功能包下 share 目录路径
from launch_ros.substitutions import FindPackageShare

# ==================== 模块级常量 ====================
PACKAGE_NAME = 'gzbot_moveit_config'                 # 功能包名
ROS_CONTROLLERS_FILENAME = 'ros2_controllers.yaml'  # ros2_control 控制器参数文件


def generate_launch_description():
    # ---------- 默认文件路径 ----------
    # 控制器参数文件：<包>/config/gzbot/ros2_controllers.yaml
    pkg_share_path = FindPackageShare(PACKAGE_NAME)
    default_ros2_controllers_file = PathJoinSubstitution(
        [pkg_share_path, 'config', 'gzbot', ROS_CONTROLLERS_FILENAME])

    # ---------- spawner 辅助函数（闭包，直接用上面的路径变量）----------
    # spawner：加载并激活指定控制器；--param-file 为该控制器指定参数文件，
    # 加载完成后 spawner 进程自动退出（后面用这个"退出事件"做接力）
    def spawner(name):
        return Node(
            package='controller_manager',
            executable='spawner',
            arguments=[name, '--param-file', default_ros2_controllers_file],
            output='screen')

    # 三个控制器：关节状态广播器 / 机械臂关节组控制器 / 夹爪控制器
    start_joint_state_broadcaster_cmd = spawner("joint_state_broadcaster")
    start_arm_controller_cmd = spawner("arm_controller")
    start_gripper_action_controller_cmd = spawner("gripper_controller")

    # 延时 10 秒再启动 joint_state_broadcaster，
    # 给 controller_manager（gz 仿真侧）留出初始化时间
    delayed_start = TimerAction(
        period=10.0,
        actions=[start_joint_state_broadcaster_cmd]
    )

    # 事件链第 1 棒：joint_state_broadcaster 的 spawner 退出后 → 启动 arm_controller
    load_arm_controller_cmd = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_joint_state_broadcaster_cmd,
            on_exit=[start_arm_controller_cmd]))

    # 事件链第 2 棒：arm_controller 的 spawner 退出后 → 启动 gripper_controller
    load_gripper_action_controller_cmd = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_arm_controller_cmd,
            on_exit=[start_gripper_action_controller_cmd]))

    return LaunchDescription([
        delayed_start,
        load_arm_controller_cmd,
        load_gripper_action_controller_cmd
    ])
