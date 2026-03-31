cmake_minimum_required(VERSION 3.20)
project(perception)

# Use C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Compiler warnings
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

#
# Fake Laser Scan Node
#
add_executable(fake_laser_scan
  src/fake_laser_scan.cpp
)

target_link_libraries(fake_laser_scan PUBLIC
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs_library
)

target_include_directories(fake_laser_scan PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

#
# Fake IMU Node
#
add_executable(fake_imu
  src/fake_imu.cpp
)

target_link_libraries(fake_imu PUBLIC
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs_library
)

target_include_directories(fake_imu PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

#
# Install executables
#
install(TARGETS
  fake_laser_scan
  fake_imu
  DESTINATION lib/${PROJECT_NAME}
)

#
# Install launch files
#
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()