#!/bin/bash
# Single script to launch the myCobot with Gazebo and ROS 2 Controllers

cleanup() {
    echo "Cleaning up..."
    sleep 5.0
    # 精确匹配进程，避免误杀命令行含 /home/gz、ros2_ws 路径的进程（如 VSCode）
    pkill -9 -f "ros2 launch"
    pkill -9 -f "ros2 run"
    pkill -9 -f "gz sim"
    pkill -9 -x gz
    pkill -9 -x ign
    pkill -9 -x gzserver
    pkill -9 -x gzclient
    pkill -9 -x rviz2
    pkill -9 -f "bt_navigator|nav_to_pose|assisted_teleop|cmd_vel_relay|robot_state_publisher|joint_state_publisher|move_to_free|autodock|cliff_detection|move_group|basic_navigator"
}

# Set up cleanup trap
trap 'cleanup' SIGINT SIGTERM

echo "Launching Gazebo simulation..."
ros2 launch mycobot_gazebo mycobot.gazebo.launch.py \
    load_controllers:=true \
    world_file:=pick_and_place_demo.world \
    use_camera:=true \
    use_rviz:=true \
    use_robot_state_pub:=true \
    use_sim_time:=true \
    x:=0.0 \
    y:=0.0 \
    z:=0.03 \
    roll:=0.0 \
    pitch:=0.0 \
    yaw:=0.0
