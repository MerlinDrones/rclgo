cmake_minimum_required(VERSION 3.8)
project(rclgo_test_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Edge.msg"
  "msg/Inner.msg"
  "srv/Op.srv"
  "action/Run.action"
  DEPENDENCIES builtin_interfaces geometry_msgs
)

ament_package()
