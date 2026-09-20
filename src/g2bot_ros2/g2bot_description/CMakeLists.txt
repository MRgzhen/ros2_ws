cmake_minimum_required(VERSION 3.8)
project(g2bot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# 安装 launch / rviz / urdf / meshes 到 share 目录
# （meshes 为 G2/T2 + arm/crs + gripper/omnipicker 子集，genie xacro 按原相对路径引用）
install(
  DIRECTORY launch rviz urdf meshes
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
