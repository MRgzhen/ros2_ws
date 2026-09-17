import os

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
from launch.substitutions import Command,PathJoinSubstitution,LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # Define filenames
    urdf_package = 'gzbot_description'
    urdf_filename = 'gzbot.urdf.xacro'
    rviz_config_filename = 'gzbot_config.rviz'

    # Set paths to important files
    pkg_share_description = FindPackageShare(urdf_package)
    default_urdf_model_path = PathJoinSubstitution(
        [pkg_share_description, 'urdf', 'robot', urdf_filename])
    default_rviz_config_path = PathJoinSubstitution(
        [pkg_share_description, 'rviz', rviz_config_filename])

     # Launch configuration variables
    use_jsp = LaunchConfiguration('use_jsp')
    jsp_gui = LaunchConfiguration('jsp_gui')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    urdf_model = LaunchConfiguration('urdf_model')

   # Declare the launch arguments
    declare_jsp_gui_cmd = DeclareLaunchArgument(
        name='jsp_gui',
        default_value='true',
        choices=['true', 'false'],
        description='Flag to enable joint_state_publisher_gui')
    declare_use_jsp_cmd = DeclareLaunchArgument(
        name='use_jsp',
        default_value='false',
        choices=['true', 'false'],
        description='Enable the joint state publisher')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    declare_use_use_gazebo_cmd = DeclareLaunchArgument(
        name='use_gazebo',
        default_value='false',
        description='Use gazebo if true')
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use')

    declare_urdf_model_path_cmd = DeclareLaunchArgument(
        name='urdf_model',
        default_value=default_urdf_model_path,
        description='Absolute path to robot urdf file')

    robot_description_content = ParameterValue(Command([
        'xacro', ' ', urdf_model, ' ',
        'use_gazebo:=', LaunchConfiguration('use_gazebo')
    ]), value_type=str)


    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content}])

    # Publish the joint state values for the non-fixed joints in the URDF file.
    start_joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_jsp))

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    start_joint_state_publisher_gui_cmd = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(jsp_gui))

    # Launch RViz
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}])

    return LaunchDescription([
        declare_jsp_gui_cmd,
        declare_use_jsp_cmd,
        declare_use_sim_time_cmd,
        declare_use_use_gazebo_cmd,
        declare_rviz_config_file_cmd,
        declare_urdf_model_path_cmd,

        start_robot_state_publisher_cmd,
        start_joint_state_publisher_cmd,
        start_joint_state_publisher_gui_cmd,
        start_rviz_cmd
    ])
