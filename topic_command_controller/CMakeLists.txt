cmake_minimum_required(VERSION 3.8)
project(topic_command_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -O2")

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/topic_command_controller.cpp
)

pluginlib_export_plugin_description_file(controller_interface topic_command_controller_plugin.xml)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
