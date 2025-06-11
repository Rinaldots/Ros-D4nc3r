cmake_minimum_required(VERSION 3.8)
project(d4nc3r_handler)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED) 

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)


add_executable(yolo_to_twist_node src/yolo_to_twist.cpp)

ament_target_dependencies(yolo_to_twist_node
  rclcpp
  geometry_msgs
  visualization_msgs
  nav_msgs
)

install(
  DIRECTORY
    config
    launch
  DESTINATION
    share/${PROJECT_NAME}/
)

install(TARGETS yolo_to_twist_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
