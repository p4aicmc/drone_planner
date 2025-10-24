cmake_minimum_required(VERSION 3.5)

project(harpia_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GeneratePath.srv"
  "srv/StrInOut.srv"
  "action/MoveTo.action"
  "action/ExecutePlan.action"
  "action/ActionCaller.action"
  DEPENDENCIES builtin_interfaces std_msgs action_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
