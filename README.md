# ddt_rl_deploy

English | [中文](README_CN.md)

RL policy deployment for DDT robots. Self-contained repository: sim2sim and sim2real share one codebase — the application depends only on ROS 2 topics; only the peer at the other end differs (local sim vs. onboard unit).



## Robots

Supports **d1** — policies `rl_flat` / `rl_flat_lab` / `rl_rough_lab`, sim2sim and sim2real.

Tested on Ubuntu 22.04 · ROS 2 Humble · Python 3.10.

## Quickstart

```bash
# Prerequisite: system ROS 2 Humble installed
./scripts/setup.sh                          # pip deps + colcon build (default: self-contained mujoco)
                                            # add gazebo/webots: --with-gazebo | --with-webots | --with-all (see Dependencies)

# sim2sim (three terminals):
./scripts/run_sim.sh                        # Mujoco (default); --backend gazebo|webots, --robot, --terrain (webots)
./scripts/run_policy.sh rl_flat_lab d1      # policy (see Robots for the list)
./scripts/run_teleop.sh                     # teleop: ws fwd/back · ad turn · qe strafe · rf height · space stop · x quit

# sim2real: place the remote in 08 SDK Mode, run only the last two commands;
#           the peer becomes the onboard unit — code unchanged
```

## Layout

```
src/description/  robot models, one ament package per robot: meshes + xacro + MuJoCo XML
src/config/       per-robot folder (NOT a package): <robot>/deploy.yaml + ONNX policies
src/control/      ddt_msgs + topic controller + Python policy/teleop application
src/sim/          MuJoCo (self-contained) + optional Gazebo / Webots bridges
scripts/          setup · run_sim · run_policy · run_teleop · new_robot
vendor/           lodepng source dependency for the MuJoCo build
```

## Adding a robot

Run `./scripts/new_robot.sh <robot>` to scaffold both locations, then drop in
files — no CMakeLists to hand-write. Then run with `--robot <robot>`. No code
changes **if** the policy uses observations the engine already has (see
`_OBS_SPECS` in `src/control/inference/policy_engine.py`); a new observation type
means one entry there plus a matching `_obs_*` method.

- **`src/description/<robot>_description/`** — an ament package (model on `src/description/d1_description`):
  `xacro/robot.xacro` + `xacro/ros2control.xacro` (declares the per-joint
  position/velocity/effort/kp/kd command interfaces + the `trunk_imu` sensor),
  and `mujoco/scene.xml` + `robot.xml` (IMU sensors
  `trunk_quat`/`trunk_gyro`/`trunk_accel`). Must be a package: xacro `$(find …)`,
  `package://` meshes and the MuJoCo `model_package` all resolve through it.
- **`src/config/<robot>/`** — a plain folder (no package): one `deploy.yaml`
  shared by every simulator and `rl_inference`, plus the referenced `*.onnx`
  policies. Resolved by path (`src/config/<robot>/deploy.yaml`), so nothing to build.

```bash
./scripts/new_robot.sh <robot>          # scaffold description package + config folder
# ... add model files + deploy.yaml + onnx ...
./scripts/setup.sh
./scripts/run_sim.sh --robot <robot>
./scripts/run_policy.sh <policy> <robot>
```

## Dependencies

System **ROS 2 Humble** + pip (`requirements.txt`, handled by `setup.sh`). The default
(MuJoCo) backend is self-contained — no external workspace required.

**Optional sim backends** — off by default; install the simulator, then build it in:

| Backend | Build | Extra install |
|---|---|---|
| MuJoCo | `setup.sh` (default) | none (pip `mujoco`) |
| Gazebo | `setup.sh --with-gazebo` | gazebo classic + `ros-humble-gazebo-ros2-control` |
| Webots | `setup.sh --with-webots` | Webots R2025a + `ros-humble-webots-ros2` |
| Both | `setup.sh --with-all` | both of the above |

Then: `run_sim.sh --backend gazebo|webots` (webots also takes `--terrain empty_world|stairs|uneven`).

## Troubleshooting

- **Real robot never discovered across hosts**: `env.sh` sets `ROS_LOCALHOST_ONLY=1` (loopback only). When the onboard unit is on a separate host, comment that line out (use a dedicated `ROS_DOMAIN_ID` instead) or discovery silently fails.

## Known limitations

- Simulation spawns in a standing pose and actively holds it until the first policy
  joint command arrives. There is still no stand-up or self-righting motion: real
  hardware must be placed in a stand-ready pose, and an inverted robot needs a manual reset.
- Train→deploy config is still copied manually; automatic manifest export is TODO.
