# ddt_rl_deploy

English | [中文](README_CN.md)

RL policy deployment for DDT robots. Self-contained repository: sim2sim and sim2real share one codebase — the application depends only on ROS 2 topics; only the peer at the other end differs (local sim vs. onboard unit).



## Robots

`d1` is the deliverable. Everything else is experimental or model-only.

| Robot | Policies | sim2sim | sim2real | Status |
|---|---|:-:|:-:|---|
| **d1** | `rl_flat` / `rl_flat_lab` / `rl_rough_lab` | ✓ | ✓ | Supported |
| **d1h** | — | ○ | ○ | Model only — add config + onnx to enable |
| **tita** | — | ○ | ○ | Model only — add config + onnx to enable |

Tested on Ubuntu 22.04 · ROS 2 Humble · Python 3.10.

## Quickstart

```bash
# Prerequisite: system ROS 2 Humble installed
./scripts/setup.sh                          # pip deps + colcon build

# sim2sim (three terminals):
./scripts/run_sim.sh                        # Mujoco (--backend webots to switch, --robot to change model)
./scripts/run_policy.sh rl_flat_lab d1      # policy (--list for options)
./scripts/run_teleop.sh                     # teleop: ws fwd/back · ad turn · qe strafe · rf height · space stop · x quit

# sim2real: place the remote in 08 SDK Mode, run only the last two commands;
#           the peer becomes the onboard unit — code unchanged
```

## Layout

```
deploy/       policy_engine.py (pure-function brain) + rl_inference.py (ROS shell). Dataflow: DATAFLOW.md
sim/          simbase.py (shared backend core) + mujoco_sim.py + webots/. Topic contract: BACKEND.md
config/       controllers.yaml + *.onnx (per robot); _template/ is the skeleton for new robots
models/       robot models (URDF primary + derived Mujoco MJCF): d1 / d1h / tita / d1cargo_out
scripts/      setup · run_sim · run_policy · run_teleop
src/ddt_msgs/ message contract, the only package that requires a build
```

## Adding a robot

No code changes: add a `config/<robot>/` (controllers.yaml + onnx) and `models/<robot>_description/`.
Template is `config/_template/`; steps in [docs/ADD_ROBOT.md](docs/ADD_ROBOT.md).

## Dependencies

System **ROS 2 Humble** + pip (`requirements.txt`, handled by `setup.sh`). No external workspace required.

## Troubleshooting

- **Real robot never discovered across hosts**: `env.sh` sets `ROS_LOCALHOST_ONLY=1` (loopback only). When the onboard unit is on a separate host, comment that line out (use a dedicated `ROS_DOMAIN_ID` instead) or discovery silently fails.

## Known limitations

- Stand-up excludes self-righting (an inverted robot requires a manual reset).
- Webots d1 stand shows sim2sim contact difference — **Mujoco is the reference for policy validation**.
- Train→deploy config is still copied manually; automatic manifest export is TODO.
