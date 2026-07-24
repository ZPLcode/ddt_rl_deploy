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
deploy/       policy_engine.py (pure-function brain) + rl_inference.py (ROS shell)
sim/          simbase.py (shared backend core) + mujoco_sim.py + webots/
config/       controllers.yaml + *.onnx (per robot); _template/ is the skeleton for new robots
models/       robot models (URDF primary + derived Mujoco MJCF): d1
scripts/      setup · run_sim · run_policy · run_teleop
src/ddt_msgs/ message contract, the only package that requires a build
```

## Adding a robot

Add two files, then run with `--robot <robot>`. No code changes **if** the policy
uses observations the engine already has (see `_OBS_SPECS` in
`deploy/policy_engine.py`); a new observation type means one entry there plus a
matching `_obs_*` method.

- **`config/<robot>/`** — copy `config/_template/controllers.yaml` and edit it:
  `joints` (MJCF actuator order), `wheel_indices`, `transform_up.stand_jpos`
  (resting stand pose), and one per-policy block (observation names, scales and
  gains from your training run). Put the `*.onnx` in the same folder.
- **`models/<robot>_description/`** — `urdf/robot.urdf` (+ meshes) and, for the
  Mujoco backend, `mujoco/robot.xml` + `scene.xml` (IMU sensors
  `trunk_quat`/`trunk_gyro`/`trunk_accel`; actuator order must match `joints`).

```bash
./scripts/run_sim.sh --robot <robot>
./scripts/run_policy.sh <policy> <robot>
```

## Dependencies

System **ROS 2 Humble** + pip (`requirements.txt`, handled by `setup.sh`). No external workspace required.

## Troubleshooting

- **Real robot never discovered across hosts**: `env.sh` sets `ROS_LOCALHOST_ONLY=1` (loopback only). When the onboard unit is on a separate host, comment that line out (use a dedicated `ROS_DOMAIN_ID` instead) or discovery silently fails.

## Known limitations

- Stand-up excludes self-righting (an inverted robot requires a manual reset).
- Webots d1 stand shows sim2sim contact difference — **Mujoco is the reference for policy validation**.
- Train→deploy config is still copied manually; automatic manifest export is TODO.
