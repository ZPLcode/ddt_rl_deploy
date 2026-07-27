cmake_minimum_required(VERSION 3.16)
project(mujoco VERSION 3.3.0 LANGUAGES NONE)

# Locate the pip-installed mujoco package at configure time.
find_package(Python3 REQUIRED COMPONENTS Interpreter)
execute_process(
  COMMAND ${Python3_EXECUTABLE} -c "import os, mujoco; print(os.path.dirname(mujoco.__file__))"
  OUTPUT_VARIABLE MUJOCO_PIP_DIR
  OUTPUT_STRIP_TRAILING_WHITESPACE
  RESULT_VARIABLE _mj_rc
)
if(NOT _mj_rc EQUAL 0 OR NOT IS_DIRECTORY "${MUJOCO_PIP_DIR}")
  message(FATAL_ERROR "pip 'mujoco' not importable (rc=${_mj_rc}, dir='${MUJOCO_PIP_DIR}'). "
                      "Run: pip show mujoco")
endif()

set(MUJOCO_VERSION     "${PROJECT_VERSION}")
set(MUJOCO_LIBRARY     "${MUJOCO_PIP_DIR}/libmujoco.so.${MUJOCO_VERSION}")
set(MUJOCO_INCLUDE_DIR "${MUJOCO_PIP_DIR}/include")

if(NOT EXISTS "${MUJOCO_LIBRARY}")
  message(FATAL_ERROR "Missing ${MUJOCO_LIBRARY} -- pip mujoco version mismatch?")
endif()
if(NOT EXISTS "${MUJOCO_INCLUDE_DIR}/mujoco/mujoco.h")
  message(FATAL_ERROR "Missing ${MUJOCO_INCLUDE_DIR}/mujoco/mujoco.h")
endif()
message(STATUS "mujoco shim: lib=${MUJOCO_LIBRARY}")
message(STATUS "mujoco shim: include=${MUJOCO_INCLUDE_DIR}")

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/mujocoConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/mujocoConfig.cmake
  @ONLY
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/mujocoConfigVersion.cmake
  VERSION ${MUJOCO_VERSION}
  COMPATIBILITY SameMajorVersion
)

# Config-mode search finds share/mujoco/cmake/mujocoConfig.cmake under the prefix.
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/mujocoConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/mujocoConfigVersion.cmake
  DESTINATION share/mujoco/cmake
)

# Give colcon a lib/ so its ld_library_path hook puts the .so on LD_LIBRARY_PATH
# at runtime. Symlink (do NOT copy) the pip .so -> single source of truth.
install(CODE "
  file(MAKE_DIRECTORY \"\${CMAKE_INSTALL_PREFIX}/lib\")
  execute_process(COMMAND \${CMAKE_COMMAND} -E create_symlink
    \"${MUJOCO_LIBRARY}\"
    \"\${CMAKE_INSTALL_PREFIX}/lib/libmujoco.so.${MUJOCO_VERSION}\")
")
