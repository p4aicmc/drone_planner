cmake_minimum_required(VERSION 3.5)
project(route_executor2)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_action REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(harpia_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
    rclcpp
    rclcpp_action
    rclpy
    geometry_msgs
    mavros_msgs
    lifecycle_msgs
    std_srvs
    harpia_msgs
    sensor_msgs
)

install(DIRECTORY launch pddl libs data solver scripts DESTINATION share/${PROJECT_NAME})
install(DIRECTORY libs DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/route_executor.py
  scripts/path_planner.py
  scripts/data_server.py
  scripts/lifecycle_manager.py
  scripts/problem_generator.py
  scripts/mission_controller.py
  scripts/action_planner.py
  scripts/go_to.py
  scripts/take_image.py
  scripts/recharge_battery.py
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_dependencies(${dependencies})

ament_package()
