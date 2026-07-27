# ddt_rl_deploy

English | [中文](README_CN.md)

RL policy deployment for DDT robots. Self-contained repository: sim2sim and sim2real share one codebase — the application depends only on ROS 2 topics; only the peer at the other end differs (local sim vs. onboard unit).



## Robots

Supports **d1** — policies `rl_flat` / `rl_flat_lab` / `rl_rough_lab`, sim2sim and sim2real.

Tested on Ubuntu 22.04 · ROS 2 Humble · Python 3.10.

## Quickstart

```bash
# Prerequisite: system ROS 2 Humble installed
./scripts/setup.sh                          # pip deps + colcon build

# sim2sim (three terminals):
./scripts/run_sim.sh                        # Mujoco (--backend webots to switch, --robot to change model)
./scripts/run_policy.sh rl_flat_lab d1      # policy (see Robots for the list)
./scripts/run_teleop.sh                     # teleop: ws fwd/back · ad turn · qe strafe · rf height · space stop · x quit

# sim2real: place the remote in 08 SDK Mode, run only the last two commands;
#           the peer becomes the onboard unit — code unchanged
```

## Layout

```
deploy/     Python app: policy_engine (brain) + rl_inference (ROS shell) + joy_mapping + teleop
src/robot/  our ROS pkgs: ddt_msgs, topic_command_controller (passthrough), <robot>_description (URDF/xacro + MJCF)
src/sim/    MuJoCo sim2sim stack (mujoco_sim_ros2 + mujoco_ros2_control + mujoco_bridge); skipped on a real-robot build
config/     <robot>/controllers.yaml + *.onnx — per-robot policy, read by rl_inference
scripts/    shell launchers: setup · run_sim · run_policy · run_teleop
vendor/     lodepng (source dep for the MuJoCo build)
```

## Adding a robot

Add two files, then run with `--robot <robot>`. No code changes **if** the policy
uses observations the engine already has (see `_OBS_SPECS` in
`deploy/policy_engine.py`); a new observation type means one entry there plus a
matching `_obs_*` method.

- **`src/robot/<robot>_description/`** — an ament package (model on `src/robot/d1_description`):
  `xacro/robot.xacro` + `xacro/ros2control.xacro` (declares the per-joint
  position/velocity/effort/kp/kd command interfaces + the `trunk_imu` sensor),
  and `mujoco/scene.xml` + `robot.xml` (IMU sensors
  `trunk_quat`/`trunk_gyro`/`trunk_accel`).
- **`config/<robot>/`** — `controllers.yaml` (policy params rl_inference reads:
  `joints`, per-policy obs / scales / gains) + the `*.onnx`.

```bash
./scripts/run_sim.sh --robot <robot>
./scripts/run_policy.sh <policy> <robot>
```

## Dependencies

System **ROS 2 Humble** + pip (`requirements.txt`, handled by `setup.sh`). No external workspace required.

## Troubleshooting

- **Real robot never discovered across hosts**: `env.sh` sets `ROS_LOCALHOST_ONLY=1` (loopback only). When the onboard unit is on a separate host, comment that line out (use a dedicated `ROS_DOMAIN_ID` instead) or discovery silently fails.

## Known limitations

- No scripted stand-up: the node starts directly in RL mode, so the robot must begin from a stand-ready pose (sim spawn pose / a manually held robot); an inverted robot requires a manual reset.
- Webots d1 stand shows sim2sim contact difference — **Mujoco is the reference for policy validation**.
- Train→deploy config is still copied manually; automatic manifest export is TODO.
