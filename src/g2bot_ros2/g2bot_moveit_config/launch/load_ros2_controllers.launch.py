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
PACKAGE_NAME = 'g2bot_moveit_config'                # 功能包名
ROS_CONTROLLERS_FILENAME = 'ros2_controllers.yaml'  # ros2_control 控制器参数文件


def generate_launch_description():
    # ---------- 默认文件路径 ----------
    # 控制器参数文件：<包>/config/g2bot/ros2_controllers.yaml
    pkg_share_path = FindPackageShare(PACKAGE_NAME)
    default_ros2_controllers_file = PathJoinSubstitution(
        [pkg_share_path, 'config', 'g2bot', ROS_CONTROLLERS_FILENAME])

    # ---------- spawner 辅助函数（闭包，直接用上面的路径变量）----------
    # spawner：加载并激活指定控制器；--param-file 为该控制器指定参数文件，
    # 加载完成后 spawner 进程自动退出（后面用这个"退出事件"做接力）
    def spawner(name):
        return Node(
            package='controller_manager',
            executable='spawner',
            arguments=[name, '--param-file', default_ros2_controllers_file],
            output='screen')

    # 控制器：关节状态广播器 / 躯干 / 头部 / 左臂 / 右臂 / 左夹爪 / 右夹爪
    start_joint_state_broadcaster_cmd = spawner("joint_state_broadcaster")
    start_body_controller_cmd = spawner("body_controller")
    start_head_controller_cmd = spawner("head_controller")
    start_left_arm_controller_cmd = spawner("left_arm_controller")
    start_right_arm_controller_cmd = spawner("right_arm_controller")
    start_gripper_l_controller_cmd = spawner("gripper_l_controller")
    start_gripper_r_controller_cmd = spawner("gripper_r_controller")

    # 延时 10 秒再启动 joint_state_broadcaster，
    # 给 controller_manager（gz 仿真侧）留出初始化时间
    delayed_start = TimerAction(
        period=10.0,
        actions=[start_joint_state_broadcaster_cmd]
    )

    # 事件链接力（前一个 spawner 退出后启动下一个）：
    # joint_state_broadcaster → body → head → 左臂 → 右臂 → 左夹爪 → 右夹爪
    load_body_controller_cmd = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_joint_state_broadcaster_cmd,
            on_exit=[start_body_controller_cmd]))

    load_head_controller_cmd = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_body_controller_cmd,
            on_exit=[start_head_controller_cmd]))

    load_left_arm_controller_cmd = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_head_controller_cmd,
            on_exit=[start_left_arm_controller_cmd]))

    load_right_arm_controller_cmd = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_left_arm_controller_cmd,
            on_exit=[start_right_arm_controller_cmd]))

    load_gripper_l_controller_cmd = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_right_arm_controller_cmd,
            on_exit=[start_gripper_l_controller_cmd]))

    load_gripper_r_controller_cmd = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_gripper_l_controller_cmd,
            on_exit=[start_gripper_r_controller_cmd]))

    return LaunchDescription([
        delayed_start,
        load_body_controller_cmd,
        load_head_controller_cmd,
        load_left_arm_controller_cmd,
        load_right_arm_controller_cmd,
        load_gripper_l_controller_cmd,
        load_gripper_r_controller_cmd
    ])
