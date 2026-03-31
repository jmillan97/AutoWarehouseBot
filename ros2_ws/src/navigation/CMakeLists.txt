cmake_minimum_required(VERSION 3.20)
project(navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  maps
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()