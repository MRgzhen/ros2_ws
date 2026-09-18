# ==================== import 速查表 ====================
# 常用基础
from launch import LaunchDescription
from launch_ros.actions import Node
# 参数声明与获取
from launch.actions import DeclareLaunchArgument
# 获取功能包下 share 目录路径
from launch_ros.substitutions import FindPackageShare
# 条件执行相关（rviz 开关、仿真/实物切换）
from launch.conditions import IfCondition
# xacro 转 robot_description 相关
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

# ==================== 模块级常量 ====================
PACKAGE_NAME = 'gzbot_description'           # 功能包名
URDF_FILENAME = 'gzbot.urdf.xacro'           # 机器人模型文件（xacro）
RVIZ_CONFIG_FILENAME = 'gzbot_config.rviz'   # RViz 配置文件


def generate_launch_description():
    # ---------- 默认文件路径 ----------
    pkg_share_description = FindPackageShare(PACKAGE_NAME)
    default_urdf_model_path = PathJoinSubstitution(
        [pkg_share_description, 'urdf', 'robot', URDF_FILENAME])
    default_rviz_config_path = PathJoinSubstitution(
        [pkg_share_description, 'rviz', RVIZ_CONFIG_FILENAME])

    # ---------- 启动参数（LaunchConfiguration）----------
    use_jsp = LaunchConfiguration('use_jsp')
    jsp_gui = LaunchConfiguration('jsp_gui')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_gazebo = LaunchConfiguration('use_gazebo')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    urdf_model = LaunchConfiguration('urdf_model')

    # ---------- 声明启动参数 ----------
    # 是否启动 joint_state_publisher（命令行版）
    declare_use_jsp_cmd = DeclareLaunchArgument(
        name='use_jsp',
        default_value='false',
        choices=['true', 'false'],
        description='启动 joint_state_publisher（命令行版）')
    # 是否启动 joint_state_publisher_gui（GUI 版，与 use_jsp 相互独立）
    declare_jsp_gui_cmd = DeclareLaunchArgument(
        name='jsp_gui',
        default_value='true',
        choices=['true', 'false'],
        description='启动 joint_state_publisher_gui（与 use_jsp 独立控制）')
    # 是否使用仿真时钟（Gazebo）
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        choices=['true', 'false'],
        description='true 时使用仿真（Gazebo）时钟')
    # 是否启用 Gazebo 仿真（透传给 xacro，控制仿真专用配置）
    declare_use_gazebo_cmd = DeclareLaunchArgument(
        name='use_gazebo',
        default_value='false',
        choices=['true', 'false'],
        description='true 时 xacro 启用 Gazebo 仿真相关配置')
    # RViz 配置文件路径
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='RViz 配置文件的完整路径')
    # URDF 模型文件路径
    declare_urdf_model_path_cmd = DeclareLaunchArgument(
        name='urdf_model',
        default_value=default_urdf_model_path,
        description='机器人 URDF（xacro）文件的绝对路径')

    # 用 xacro 命令实时解析模型，并透传 use_gazebo 参数
    robot_description_content = ParameterValue(
        Command(['xacro', ' ', urdf_model, ' ',
                 'use_gazebo:=', use_gazebo]),
        value_type=str)

    # ---------- 节点 ----------
    # robot_state_publisher：订阅关节状态，发布各 link 的 3D 位姿（TF）
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content}])

    # joint_state_publisher（命令行版）：发布 URDF 中非固定关节的关节状态
    start_joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_jsp))

    # joint_state_publisher_gui（GUI 版）：滑块手动控制关节状态，
    # 与命令行版相互独立，jsp_gui:=true 时即使 use_jsp:=false 也会启动
    start_joint_state_publisher_gui_cmd = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(jsp_gui))

    # RViz 可视化
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}])

    return LaunchDescription([
        # 启动参数声明
        declare_jsp_gui_cmd,
        declare_use_jsp_cmd,
        declare_use_sim_time_cmd,
        declare_use_gazebo_cmd,
        declare_rviz_config_file_cmd,
        declare_urdf_model_path_cmd,
        # 节点
        start_robot_state_publisher_cmd,
        start_joint_state_publisher_cmd,
        start_joint_state_publisher_gui_cmd,
        start_rviz_cmd
    ])
