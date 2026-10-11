cmake_minimum_required(VERSION 3.5)
project(drone_planner)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_planner_msgs REQUIRED)
find_package(action_planner REQUIRED)
find_package(drone_planner_msgs REQUIRED)

set(dependencies
    rclpy
    geometry_msgs
    mavros_msgs
    lifecycle_msgs
    std_srvs
    sensor_msgs
    action_planner_msgs
    action_planner
    drone_planner_msgs
)

# Importable python module (drone_planner.drone_command_action_base)
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY launch pddl libs data DESTINATION share/${PROJECT_NAME})
install(DIRECTORY libs DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/drone_control.py
  scripts/route_executor.py
  scripts/path_planner.py
  scripts/data_server.py
  scripts/problem_generator.py
  scripts/go_to.py
  scripts/take_image.py
  scripts/recharge_battery.py
  scripts/take_off.py
  scripts/land.py
  scripts/map_updater.py
  scripts/drone_marker.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies})

ament_package()
