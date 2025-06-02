cmake_minimum_required(VERSION 3.8)
project(d4nc3r_handler)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(
  DIRECTORY
    config
    launch
    urdf

  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()
