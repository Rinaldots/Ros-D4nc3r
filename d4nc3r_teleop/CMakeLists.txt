cmake_minimum_required(VERSION 3.8)
project(d4nc3r_teleop)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    launch
  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()
