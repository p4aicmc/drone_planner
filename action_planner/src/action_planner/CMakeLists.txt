cmake_minimum_required(VERSION 3.5)
project(action_planner)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(action_planner_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Importable python module: `from action_planner.action_executor_base import ActionExecutorBase`
ament_python_install_package(${PROJECT_NAME})

# Planners (prebuilt binaries + wrapper scripts). Permissions come from the source tree.
install(DIRECTORY solver
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS)

install(DIRECTORY launch examples
  DESTINATION share/${PROJECT_NAME})

# ROS executables (`ros2 run action_planner <name>`)
install(PROGRAMS
  scripts/action_planner_node.py
  scripts/mission_controller.py
  scripts/command_interface.py
  scripts/lifecycle_manager.py
  scripts/action_planner_cli.py
  scripts/timed_action_node.py
  scripts/static_problem_provider.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(action_planner_msgs lifecycle_msgs action_msgs std_msgs std_srvs)

ament_package()
