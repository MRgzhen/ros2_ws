cmake_minimum_required(VERSION 3.20)
project(gzbot_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_manager REQUIRED)
find_package(gz_ros2_control REQUIRED)
find_package(gripper_controllers REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(xacro REQUIRED)

# 安装 config / launch / models / worlds 到 share 目录
# （config 里有 ros_gz_bridge.yaml，漏装会导致 parameter_bridge 读空配置刷屏报错）
install(
  DIRECTORY config launch models worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 未添加版权声明，跳过 copyright 检查
  set(ament_cmake_copyright_FOUND TRUE)
  # 跳过 cpplint 检查
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
