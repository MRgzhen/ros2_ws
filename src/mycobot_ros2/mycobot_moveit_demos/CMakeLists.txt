cmake_minimum_required(VERSION 3.8)
project(mycobot_moveit_demos)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

# 订阅末端目标位姿并用 MoveGroupInterface 移动（配合 mycobot_learn 的 arm_target_pub）
add_executable(arm_target_listener src/arm_target_listener.cpp)
target_compile_features(arm_target_listener PUBLIC cxx_std_17)
ament_target_dependencies(arm_target_listener moveit_ros_planning_interface rclcpp geometry_msgs)

# hello_moveit 教程 demo
add_executable(hello_moveit src/hello_moveit.cpp)
target_compile_features(hello_moveit PUBLIC cxx_std_17)
ament_target_dependencies(hello_moveit moveit_ros_planning_interface rclcpp geometry_msgs)

install(
  TARGETS arm_target_listener hello_moveit
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
