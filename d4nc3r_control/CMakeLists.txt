cmake_minimum_required(VERSION 3.8)
project(d4nc3r_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Compilar a interface de hardware
add_library(d4nc3r_hardware_interface SHARED
  src/d4nc3r_hardware_interface.cpp
)
ament_target_dependencies(d4nc3r_hardware_interface
  rclcpp
  hardware_interface
  pluginlib
  std_msgs
  rclcpp_lifecycle
)
target_include_directories(d4nc3r_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Exportar o plugin para ros2_control
pluginlib_export_plugin_description_file(hardware_interface src/d4nc3r_hardware_interface_plugin.xml)
# O arquivo d4nc3r_hardware_interface_plugin.xml será criado abaixo

install(TARGETS
  d4nc3r_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# ... install launch, config files etc. ...


install(
  DIRECTORY
    config
    launch
  DESTINATION
    share/${PROJECT_NAME}/
)

ament_package()
