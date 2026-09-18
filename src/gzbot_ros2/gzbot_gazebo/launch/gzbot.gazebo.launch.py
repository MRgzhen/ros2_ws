#!/usr/bin/env python3
"""
Launch Gazebo simulation with a robot.

This launch file sets up a complete ROS 2 simulation environment with Gazebo for
a myCobot robot.

:author: Addison Sears-Collins
:date: November 16, 2024
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

PACKAGE_NAME_GAZEBO = 'gzbot_gazebo'
PACKAGE_NAME_DESCRIPTION = 'gzbot_description'
PACKAGE_NAME_MOVEIT = 'gzbot_moveit_config'
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
    use_camera = LaunchConfiguration('use_camera')
    use_gazebo = LaunchConfiguration('use_gazebo')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    load_controllers = LaunchConfiguration('load_controllers')

    # Declare the launch arguments
    declare_robot_name_cmd = DeclareLaunchArgument(
            'robot_name',
            default_value='gzbot',
            description='Name of the robot'
    )
    declare_jsp_gui_cmd = DeclareLaunchArgument(
        'jsp_gui',
        default_value='false',
        description='Flag to enable joint_state_publisher_gui'
    )
    declare_use_camera_cmd = DeclareLaunchArgument(
        'use_camera',
        default_value='false',
        description='Flag to enable camera'
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
            default_value='0.05',
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
            'use_camera': use_camera,
            'use_gazebo': use_gazebo,
            'use_rviz': use_rviz,
            'use_sim_time': use_sim_time
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
                '-name', 'robot_name',
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
        declare_use_camera_cmd,
        declare_use_gazebo_cmd,
        declare_use_rviz_cmd,
        declare_use_sim_time_cmd,
        declare_use_robot_state_pub_cmd,
        declare_load_controllers_cmd,
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
        start_gazebo_cmd,
        start_gazebo_ros_bridge_cmd,
        start_gazebo_ros_spawner_cmd
    ])