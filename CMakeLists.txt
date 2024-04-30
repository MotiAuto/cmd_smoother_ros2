cmake_minimum_required(VERSION 3.5)
project(cmd_smoother_ros2)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(target_cmd_smoother_ros2 SHARED
  DIRECTORY src
)

target_include_directories(target_cmd_smoother_ros2
  SYSTEM PUBLIC
)

rclcpp_components_register_node(target_cmd_smoother_ros2
  PLUGIN "cmd_smoother_ros2::CmdSmootherROS2"
  EXECUTABLE cmd_smoother_ros2_node
)


ament_auto_package(
  INSTALL_TO_SHARE
)