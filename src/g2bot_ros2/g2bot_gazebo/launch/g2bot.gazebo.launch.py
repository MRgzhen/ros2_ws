#!/usr/bin/env python3
"""
Launch Gazebo simulation with the G2 (genie) robot.

结构移植自 gzbot_gazebo 的 gzbot.gazebo.launch.py：
起 gz 仿真 → robot_state_publisher → 接力加载 ros2 控制器 → spawn 机器人 → 桥接时钟。

:author: gz
:date: September 20, 2026
"""

import os
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

PACKAGE_NAME_GAZEBO = 'g2bot_gazebo'
PACKAGE_NAME_DESCRIPTION = 'g2bot_description'
PACKAGE_NAME_MOVEIT = 'g2bot_moveit_config'
ROS_GZ_BRIDGE_CONFIG_FILE_PATH = 'config/ros_gz_bridge.yaml'
GAZEBO_WORLDS_PATH = 'worlds'
GAZEBO_MODELS_PATH = 'models'
DEFAULT_WORLD_FILE = 'sim_env.world'


def generate_launch_description():

    # Set the path to different files and folders
    pkg_share_gazebo = FindPackageShare(
        package=PACKAGE_NAME_GAZEBO).find(PACKAGE_NAME_GAZEBO)
    pkg_share_description = FindPackageShare(
        package=PACKAGE_NAME_DESCRIPTION).find(PACKAGE_NAME_DESCRIPTION)
    pkg_share_moveit = FindPackageShare(
        package=PACKAGE_NAME_MOVEIT).find(PACKAGE_NAME_MOVEIT)
    default_ros_gz_bridge_config_file_path = os.path.join(
        pkg_share_gazebo, ROS_GZ_BRIDGE_CONFIG_FILE_PATH)
    pkg_ros_gz_sim = FindPackageShare(package='ros_gz_sim').find('ros_gz_sim')
    gazebo_models_path = PathJoinSubstitution([
        FindPackageShare(PACKAGE_NAME_GAZEBO),
        GAZEBO_MODELS_PATH
    ])
    world_path = PathJoinSubstitution([
        FindPackageShare(PACKAGE_NAME_GAZEBO),
        GAZEBO_WORLDS_PATH,
        DEFAULT_WORLD_FILE
    ])
    # Set the pose configuration variables
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    roll = LaunchConfiguration('roll')
    pitch = LaunchConfiguration('pitch')
    yaw = LaunchConfiguration('yaw')

    # 定义参数
    robot_name = LaunchConfiguration('robot_name')
    jsp_gui = LaunchConfiguration('jsp_gui')
    use_gazebo = LaunchConfiguration('use_gazebo')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    load_controllers = LaunchConfiguration('load_controllers')
    # genie 模型配置（透传给 robot_state_publisher → xacro）
    body = LaunchConfiguration('body')
    arm = LaunchConfiguration('arm')
    lgripper = LaunchConfiguration('lgripper')
    rgripper = LaunchConfiguration('rgripper')

    # Declare the launch arguments
    declare_robot_name_cmd = DeclareLaunchArgument(
            'robot_name',
            default_value='g2bot',
            description='Name of the robot'
    )
    declare_jsp_gui_cmd = DeclareLaunchArgument(
        'jsp_gui',
        default_value='false',
        description='Flag to enable joint_state_publisher_gui'
    )
    declare_use_gazebo_cmd = DeclareLaunchArgument(
        'use_gazebo',
        default_value='true',
        description='Flag to enable Gazebo'
    )
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Flag to enable RViz'
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Flag to use simulation time'
    )
    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='true',
        description='Flag to enable robot state publisher'
    )
    declare_load_controllers_cmd = DeclareLaunchArgument(
        'load_controllers',
        default_value='true',
        description='Flag to enable loading of ROS 2 controllers'
    )
    declare_body_cmd = DeclareLaunchArgument(
        'body', default_value='t2',
        description='genie 躯干变体（t1/t2/t2v2 小写，透传给 xacro）')
    declare_arm_cmd = DeclareLaunchArgument(
        'arm', default_value='crs',
        description='genie 手臂变体（透传给 xacro）')
    declare_lgripper_cmd = DeclareLaunchArgument(
        'lgripper', default_value='omnipicker',
        description='左夹爪类型（透传给 xacro）')
    declare_rgripper_cmd = DeclareLaunchArgument(
        'rgripper', default_value='omnipicker',
        description='右夹爪类型（透传给 xacro）')

     # Pose arguments
    declare_x_cmd = DeclareLaunchArgument(
            name='x',
            default_value='0.0',
            description='x component of initial position, meters')

    declare_y_cmd = DeclareLaunchArgument(
            name='y',
            default_value='0.0',
            description='y component of initial position, meters')

    declare_z_cmd = DeclareLaunchArgument(
            name='z',
            default_value='0.1',
            description='z component of initial position, meters')

    declare_roll_cmd = DeclareLaunchArgument(
            name='roll',
            default_value='0.0',
            description='roll angle of initial orientation, radians')

    declare_pitch_cmd = DeclareLaunchArgument(
            name='pitch',
            default_value='0.0',
            description='pitch angle of initial orientation, radians')

    declare_yaw_cmd = DeclareLaunchArgument(
            name='yaw',
            default_value='0.0',
            description='yaw angle of initial orientation, radians')

    # Include Robot State Publisher launch file if enabled
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share_description, 'launch',
                         'robot_state_publisher.launch.py')
        ]),
        launch_arguments={
            'jsp_gui': jsp_gui,
            'use_gazebo': use_gazebo,
            'use_rviz': use_rviz,
            'use_sim_time': use_sim_time,
            'body': body,
            'arm': arm,
            'lgripper': lgripper,
            'rgripper': rgripper
        }.items(),
        condition=IfCondition(use_robot_state_pub)
    )
    # Include ROS 2 Controllers launch file if enabled
    load_controllers_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_share_moveit, 'launch', 'load_ros2_controllers.launch.py')]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
        condition=IfCondition(load_controllers)
    )

    # Set Gazebo model path
    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        gazebo_models_path)
    # 模型 mesh 资源路径：gz 把 package://g2bot_description/meshes/... 解析为
    # model://<包名>/meshes/...，需把包 share 的父目录（含 g2bot_description/）
    # 加入 GZ_SIM_RESOURCE_PATH，否则机器人白模/碰撞体缺失
    set_env_vars_mesh_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.dirname(pkg_share_description))

    # Start Gazebo
    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments=[('gz_args', [' -r -v 4 ', world_path])])

    # Bridge ROS topics and Gazebo messages for establishing communication
    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
                'config_file': default_ros_gz_bridge_config_file_path,
        }],
        output='screen'
    )

    # Spawn the robot
    start_gazebo_ros_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
                '-topic', '/robot_description',
                '-name', robot_name,
                '-allow_renaming', 'true',
                '-x', x,
                '-y', y,
                '-z', z,
                '-R', roll,
                '-P', pitch,
                '-Y', yaw
        ])

    return LaunchDescription([
        # 启动参数声明
        declare_robot_name_cmd,
        declare_jsp_gui_cmd,
        declare_use_gazebo_cmd,
        declare_use_rviz_cmd,
        declare_use_sim_time_cmd,
        declare_use_robot_state_pub_cmd,
        declare_load_controllers_cmd,
        declare_body_cmd,
        declare_arm_cmd,
        declare_lgripper_cmd,
        declare_rgripper_cmd,
        declare_x_cmd,
        declare_y_cmd,
        declare_z_cmd,
        declare_roll_cmd,
        declare_pitch_cmd,
        declare_yaw_cmd,
        # 启动节点
        robot_state_publisher_cmd,
        load_controllers_cmd,
        set_env_vars_resources,
        set_env_vars_mesh_resources,
        start_gazebo_cmd,
        start_gazebo_ros_bridge_cmd,
        start_gazebo_ros_spawner_cmd
    ])
