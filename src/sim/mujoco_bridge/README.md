# mujoco_bridge

Self-contained MuJoCo backend for `ddt_rl_deploy`.

This single ROS 2 package contains:

- the MuJoCo `simulate` application and physics-plugin interface;
- the `ros2_control` physics and hardware plugins;
- controller configuration and the backend launch file;
- an internal CMake adapter for the pip-installed MuJoCo library.

Build it with:

```bash
./scripts/setup.sh
```

Run it with:

```bash
./scripts/run_sim.sh --backend mujoco
```

The public plugin class names retain their upstream
`mujoco_sim_ros2`/`mujoco_ros2_control` C++ namespaces for compatibility, but
there are no longer separate ROS packages with those names.

The upstream `mujoco_sim_ros2` notes are retained under `docs/`.
