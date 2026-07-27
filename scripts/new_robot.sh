#!/usr/bin/env bash
# Scaffold a new robot. Creates:
#   src/description/<name>_description/   an ament package (must be a package:
#                                         xacro $(find ...) + package:// meshes +
#                                         mujoco model_package all resolve through it)
#   src/config/<name>/                    a plain folder for deploy.yaml + *.onnx
#                                         (NOT a package — resolved by path)
#   ./scripts/new_robot.sh myrobot
set -eo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

NAME="${1:?usage: new_robot.sh <robot_name>}"
DESC="$REPO/src/description/${NAME}_description"
CONF="$REPO/src/config/$NAME"

[ -e "$DESC" ] && { echo "error: $DESC already exists" >&2; exit 1; }
[ -e "$CONF" ] && { echo "error: $CONF already exists" >&2; exit 1; }

mkdir -p "$DESC/meshes" "$DESC/mujoco" "$DESC/xacro" "$CONF"

# description package — installs the model dirs onto the ament share path so
# $(find <name>_description) and package://<name>_description keep resolving.
cat > "$DESC/CMakeLists.txt" <<EOF
cmake_minimum_required(VERSION 3.5)
project(${NAME}_description)
find_package(ament_cmake REQUIRED)
install(DIRECTORY meshes mujoco xacro DESTINATION share/\${PROJECT_NAME})
ament_package()
EOF

cat > "$DESC/package.xml" <<EOF
<?xml version="1.0"?>
<package format="3">
  <name>${NAME}_description</name>
  <version>0.0.0</version>
  <description>${NAME} robot model (meshes + xacro + MuJoCo XML).</description>
  <maintainer email="you@example.com">you</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF

echo "scaffolded '${NAME}':"
echo "  src/description/${NAME}_description/  (CMakeLists.txt + package.xml + xacro/ mujoco/ meshes/)"
echo "  src/config/${NAME}/                   (empty folder — no package)"
echo
echo "next:"
echo "  1. model -> src/description/${NAME}_description/"
echo "       xacro/robot.xacro + xacro/ros2control.xacro"
echo "         (use \$(find ${NAME}_description) for includes, package://${NAME}_description for meshes)"
echo "       mujoco/scene.xml + mujoco/robot.xml   meshes/*.STL"
echo "  2. config -> src/config/${NAME}/"
echo "       cp src/config/d1/deploy.yaml src/config/${NAME}/   # then edit joints/policies"
echo "       + the *.onnx it references"
echo "  3. build & run:"
echo "       ./scripts/setup.sh"
echo "       ./scripts/run_sim.sh --robot ${NAME}"
echo "       ./scripts/run_policy.sh <policy> ${NAME}"
