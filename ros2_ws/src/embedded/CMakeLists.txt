cmake_minimum_required(VERSION 3.20)
project(embedded)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)

# serial bridge
add_executable(serial_bridge src/serial_bridge.cpp)
ament_target_dependencies(serial_bridge rclcpp geometry_msgs std_msgs nav_msgs tf2 tf2_ros)
install(TARGETS serial_bridge DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)
add_executable(wheel_odometry src/wheel_odometry.cpp)
ament_target_dependencies(wheel_odometry rclcpp std_msgs nav_msgs geometry_msgs tf2 tf2_ros)
install(TARGETS wheel_odometry DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS
  src/imu_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME imu_node
)
install(PROGRAMS
  scripts/serial_bridge_py.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME serial_bridge_py
)
ament_package()
