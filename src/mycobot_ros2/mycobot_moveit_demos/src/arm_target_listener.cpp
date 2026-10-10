/**
 * @file arm_target_listener.cpp
 * @brief 订阅 /arm_target_pose 上的末端目标位姿，用 MoveGroupInterface 规划并执行
 *
 * 从 mycobot_mtc_pick_place_demo 的思路抽取的"最简 MoveIt"版：
 * 不做感知、不做 MTC 多阶段任务，只保留"收到位姿 → 移动过去"。
 * 结构 = hello_moveit.cpp + 订阅循环；执行器用官方 MoveGroupInterface
 * 教程推荐的后台 spin 线程模式（主线程做构造/规划/执行）。
 *
 * 消息：geometry_msgs/PoseStamped（话题 arm_target_pose）
 *   - frame_id 为空时按 base_link 处理
 *   - 四元数全零时用参数 default_orientation_x/y/z/w 补齐
 *     （默认 (1,0,0,0)，同 hello_moveit）
 *   - 一条消息触发一次移动；规划/执行期间晚到的消息直接丢弃
 *
 * 运行（先 robotm 启动 Gazebo+MoveIt）：
 *   ros2 run mycobot_moveit_demos arm_target_listener
 * 发送端（另开终端，或直接 ros2 topic pub）：
 *   ros2 run mycobot_learn arm_target_pub --ros-args -p x:=0.1 -p y:=0.15 -p z:=0.15
 */

#include <chrono>
#include <future>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char * argv[])
{
  // Start up ROS 2
  rclcpp::init(argc, argv);

  // Creates a node named "arm_target_listener"
  auto const node = std::make_shared<rclcpp::Node>(
    "arm_target_listener",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Creates a "logger" that we can use to print out information or error messages
  auto const logger = rclcpp::get_logger("arm_target_listener");

  // 默认目标姿态（消息四元数全零时使用），默认值同 hello_moveit
  double qx = 1.0, qy = 0.0, qz = 0.0, qw = 0.0;
  node->get_parameter_or("default_orientation_x", qx, qx);
  node->get_parameter_or("default_orientation_y", qy, qy);
  node->get_parameter_or("default_orientation_z", qz, qz);
  node->get_parameter_or("default_orientation_w", qw, qw);

  // 官方 MoveGroupInterface 教程模式：后台线程 spin，主线程做构造/规划/执行
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  // Create the MoveIt MoveGroup Interface（规划执行仍在 move_group 节点里，本节点只是客户端）
  using moveit::planning_interface::MoveGroupInterface;
  auto arm_group_interface = MoveGroupInterface(node, "arm");

  // Specify a planning pipeline to be used for further planning
  arm_group_interface.setPlanningPipelineId("ompl");

  // Specify a planner to be used for further planning
  arm_group_interface.setPlannerId("RRTConnectkConfigDefault");

  // Specify the maximum amount of time in seconds to use when planning
  arm_group_interface.setPlanningTime(1.0);

  // Set scaling factors for maximum joint velocity / acceleration
  arm_group_interface.setMaxVelocityScalingFactor(1.0);
  arm_group_interface.setMaxAccelerationScalingFactor(1.0);

  RCLCPP_INFO(logger, "Planning pipeline: %s", arm_group_interface.getPlanningPipelineId().c_str());
  RCLCPP_INFO(logger, "Planner ID: %s", arm_group_interface.getPlannerId().c_str());
  RCLCPP_INFO(logger, "Planning time: %.2f", arm_group_interface.getPlanningTime());

  while (rclcpp::ok())
  {
    // 每轮新建订阅：规划/执行期间订阅不存在，晚到的消息直接丢弃
    std::promise<geometry_msgs::msg::PoseStamped> target_promise;
    auto target_future = target_promise.get_future();
    auto sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "arm_target_pose", rclcpp::QoS(10),
      [&target_promise](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        target_promise.set_value(*msg);
      });

    RCLCPP_INFO(logger, "等待 /arm_target_pose 上的目标位姿…");
    // 带超时等待，保证 Ctrl+C 能退出
    while (rclcpp::ok() &&
           target_future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready)
    {
    }
    if (!rclcpp::ok())
    {
      break;
    }
    sub.reset();

    // 补齐消息的缺省字段
    auto target = target_future.get();
    if (target.header.frame_id.empty())
    {
      target.header.frame_id = "base_link";
    }
    target.header.stamp = node->now();
    if (target.pose.orientation.x == 0.0 && target.pose.orientation.y == 0.0 &&
        target.pose.orientation.z == 0.0 && target.pose.orientation.w == 0.0)
    {
      target.pose.orientation.x = qx;
      target.pose.orientation.y = qy;
      target.pose.orientation.z = qz;
      target.pose.orientation.w = qw;
      RCLCPP_INFO(logger, "消息姿态全零，用默认姿态 (%.3f, %.3f, %.3f, %.3f)", qx, qy, qz, qw);
    }
    RCLCPP_INFO(logger, "新目标: (%.3f, %.3f, %.3f)",
                target.pose.position.x, target.pose.position.y, target.pose.position.z);

    // Set a target pose for the end effector of the arm
    arm_group_interface.setPoseTarget(target);

    // Create a plan to that target pose
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto const success = static_cast<bool>(arm_group_interface.plan(plan));

    // Execute the plan
    if (success)
    {
      arm_group_interface.execute(plan);
    }
    else
    {
      RCLCPP_ERROR(logger, "Planning failed!");
    }
  }

  // Shut down ROS 2 cleanly when we're done
  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}
