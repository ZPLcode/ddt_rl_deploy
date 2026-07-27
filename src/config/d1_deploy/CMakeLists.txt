cmake_minimum_required(VERSION 3.8)
project(d1_deploy)

find_package(ament_cmake REQUIRED)

# The package owns the single source of truth for D1 deployment: controller
# manager parameters, policy parameters, and the ONNX models they reference.
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
