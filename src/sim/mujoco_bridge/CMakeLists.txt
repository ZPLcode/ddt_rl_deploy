cmake_minimum_required(VERSION 3.16)
project(mujoco_bridge)

cmake_policy(SET CMP0072 NEW)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(controller_manager REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glfw3 REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(joint_limits REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Threads REQUIRED)
find_package(urdf REQUIRED)

# Expose the pip-installed MuJoCo library as an internal CMake target. Keeping
# this shim here makes mujoco_bridge the only MuJoCo ROS package in the tree.
find_package(Python3 REQUIRED COMPONENTS Interpreter)
execute_process(
  COMMAND ${Python3_EXECUTABLE} -c
          "import os, mujoco; print(os.path.dirname(mujoco.__file__)); print(mujoco.__version__)"
  OUTPUT_VARIABLE MUJOCO_PIP_INFO
  OUTPUT_STRIP_TRAILING_WHITESPACE
  RESULT_VARIABLE MUJOCO_PIP_RESULT
)
if(NOT MUJOCO_PIP_RESULT EQUAL 0)
  message(FATAL_ERROR "pip package 'mujoco' is not importable")
endif()
string(REPLACE "\n" ";" MUJOCO_PIP_INFO "${MUJOCO_PIP_INFO}")
list(GET MUJOCO_PIP_INFO 0 MUJOCO_PIP_DIR)
list(GET MUJOCO_PIP_INFO 1 MUJOCO_PIP_VERSION)
set(MUJOCO_LIBRARY "${MUJOCO_PIP_DIR}/libmujoco.so.${MUJOCO_PIP_VERSION}")
set(MUJOCO_INCLUDE_DIR "${MUJOCO_PIP_DIR}/include")
if(NOT EXISTS "${MUJOCO_LIBRARY}" OR
   NOT EXISTS "${MUJOCO_INCLUDE_DIR}/mujoco/mujoco.h")
  message(FATAL_ERROR "Incomplete pip MuJoCo installation at ${MUJOCO_PIP_DIR}")
endif()
add_library(mujoco::mujoco SHARED IMPORTED GLOBAL)
set_target_properties(mujoco::mujoco PROPERTIES
  IMPORTED_LOCATION "${MUJOCO_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_INCLUDE_DIR}"
)

# MuJoCo's simulate UI uses lodepng. The repository vendors it so builds never
# need to fetch source from the network.
set(LODEPNG_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../vendor/lodepng")
if(DEFINED FETCHCONTENT_SOURCE_DIR_LODEPNG)
  set(LODEPNG_DIR "${FETCHCONTENT_SOURCE_DIR_LODEPNG}")
endif()
add_library(lodepng STATIC
  "${LODEPNG_DIR}/lodepng.cpp"
  "${LODEPNG_DIR}/lodepng.h"
)
set_target_properties(lodepng PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(lodepng PUBLIC "${LODEPNG_DIR}")

# MuJoCo simulate application and its plugin API.
add_executable(mujoco_sim
  src/sim_app/glfw_adapter.cc
  src/sim_app/glfw_dispatch.cc
  src/sim_app/platform_ui_adapter.cc
  src/sim_app/simulate.cc
  src/sim_app/main.cc
)
target_link_libraries(mujoco_sim
  mujoco::mujoco glfw lodepng Threads::Threads
)
ament_target_dependencies(mujoco_sim
  ament_index_cpp
  controller_manager
  pluginlib
  rclcpp
)
target_include_directories(mujoco_sim PRIVATE
  include
  src/sim_app
)

set(MUJOCO_CONTROL_DEPENDENCIES
  control_toolbox
  controller_manager
  hardware_interface
  joint_limits
  pluginlib
  rclcpp
  rosgraph_msgs
  sensor_msgs
  urdf
)

# ros2_control hardware implementation used by the robot URDF.
add_library(mujoco_system_plugins SHARED
  src/ros2_control/mujoco_system.cpp
)
ament_target_dependencies(mujoco_system_plugins ${MUJOCO_CONTROL_DEPENDENCIES})
target_link_libraries(mujoco_system_plugins mujoco::mujoco)
target_include_directories(mujoco_system_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)
pluginlib_export_plugin_description_file(
  mujoco_bridge mujoco_system_plugins.xml)

# Physics plugin loaded by the bundled simulate application.
add_library(mujoco_ros2_control-system SHARED
  src/ros2_control/mujoco_ros2_control_plugin.cpp
  src/ros2_control/mujoco_ros2_control.cpp
)
ament_target_dependencies(
  mujoco_ros2_control-system ${MUJOCO_CONTROL_DEPENDENCIES})
target_link_libraries(mujoco_ros2_control-system mujoco::mujoco)
target_include_directories(mujoco_ros2_control-system PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)
pluginlib_export_plugin_description_file(
  mujoco_bridge mujoco_ros2_control_plugins.xml)

# Keep the upstream standalone executable available for diagnostics.
add_executable(mujoco_ros2_control
  src/ros2_control/mujoco_ros2_control_node.cpp
  src/ros2_control/mujoco_rendering.cpp
  src/ros2_control/mujoco_ros2_control.cpp
  src/ros2_control/mujoco_cameras.cpp
)
ament_target_dependencies(mujoco_ros2_control ${MUJOCO_CONTROL_DEPENDENCIES})
target_link_libraries(mujoco_ros2_control mujoco::mujoco glfw)
target_include_directories(mujoco_ros2_control PRIVATE
  include
  ${EIGEN3_INCLUDE_DIR}
)

install(TARGETS
  mujoco_system_plugins
  mujoco_ros2_control-system
  LIBRARY DESTINATION lib
)
install(TARGETS
  mujoco_sim
  mujoco_ros2_control
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# Install a symlink matching MuJoCo's SONAME. ament's library path hook for the
# plugin libraries above makes this directory available at runtime.
install(CODE "
  file(MAKE_DIRECTORY \"\${CMAKE_INSTALL_PREFIX}/lib\")
  execute_process(COMMAND \${CMAKE_COMMAND} -E create_symlink
    \"${MUJOCO_LIBRARY}\"
    \"\${CMAKE_INSTALL_PREFIX}/lib/libmujoco.so.${MUJOCO_PIP_VERSION}\")
")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(
  controller_manager
  hardware_interface
  pluginlib
  rclcpp
)
ament_package()
