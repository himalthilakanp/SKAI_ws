cmake_minimum_required(VERSION 3.8)

project(skai_hardware)



find_package(ament_cmake REQUIRED)

find_package(hardware_interface REQUIRED)

find_package(pluginlib REQUIRED)

find_package(rclcpp REQUIRED)

find_package(rclcpp_lifecycle REQUIRED)

find_package(std_msgs REQUIRED)



add_library(

  skai_hardware
  SHARED
  src/skai_hardware.cpp
  src/odrive_driver.cpp

)



target_include_directories(

  skai_hardware

  PRIVATE include

)



ament_target_dependencies(

  skai_hardware

  hardware_interface

  pluginlib

  rclcpp

  rclcpp_lifecycle

  std_msgs

)



pluginlib_export_plugin_description_file(

  hardware_interface

  skai_hardware.xml

)



install(

  TARGETS skai_hardware

  DESTINATION lib

)



install(

  DIRECTORY include/

  DESTINATION include/

)



install(

  FILES skai_hardware.xml

  DESTINATION share/${PROJECT_NAME}

)



ament_package()
