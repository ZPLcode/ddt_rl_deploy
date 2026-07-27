cmake_minimum_required(VERSION 3.8)
project(topic_command_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

pluginlib_export_plugin_description_file(controller_interface topic_command_controller_plugin.xml)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/topic_command_controller.cpp
)

ament_auto_package()
