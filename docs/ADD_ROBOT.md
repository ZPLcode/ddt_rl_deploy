# Adding a new robot

Owing to the config-driven design, **adding a robot requires no code changes** —
add one model + one config. This is the fill-in checklist. Examples use
`<robot>` as the model name (e.g. `d1h`).

## Requirements

- The robot **URDF** (+ meshes).
- The trained policy **ONNX** + its **metadata** (obs order, per-field scales,
  num_obs/num_actions/history_len) — exported from the training side. These are
  the `# [TRAIN]` fields; incorrect values feed the policy garbage input.

## Three steps

### 1. Model → `models/<robot>_description/`

| File | Used by | Note |
|---|---|---|
| `urdf/robot.urdf` (+ `meshes/`) | pybullet / real robot / MJCF conversion | Main model; meshes use the relative path `../meshes/` |
| `mujoco/robot.xml` + `mujoco/scene.xml` | **mujoco backend** | Convert MJCF from the URDF; **add IMU sensors manually** `trunk_quat`/`trunk_gyro`/`trunk_accel` + actuators. ⚠️ **MJCF actuator order must == the config `joints` order** (the mujoco backend asserts on startup and aborts on mismatch) |
| `sim/webots/protos/<ROBOT>.proto` + `sim/webots/worlds/<robot>.wbt` | webots backend (optional) | Generate the proto with `urdf2webots` + add IMU manually; copy the world from [d1.wbt](../sim/webots/worlds/d1.wbt) and swap the model name (`run.sh --robot <robot>` finds the world by name). ⚠️ Two conversion pitfalls, both shown in [D1.proto](../sim/webots/protos/D1.proto): **wheel collision-cylinder axis** (URDF axis Z vs Webots default Y — add `rotation 1 0 0 1.5708`); **joint stabilization layer** (per joint `dampingConstant 0.1` + `staticFriction 0.2` to match the MJCF defaults block; without it, oscillation) |

### 2. Config → `config/<robot>/`

```bash
cp -r config/_template config/<robot>
```

Edit [config/_template/controllers.yaml](../config/_template/controllers.yaml),
replacing each `# <FILL>` with your robot's values:
- `joints` — all joint names, **in training / MJCF actuator order** (names
  containing `foot` are auto-treated as wheels; otherwise use `wheel_indices`).
- `transform_up.stand_jpos` — **the real standing pose** ([SIM] simbase reads it
  as the hold pose; standup targets it too).
- `wheel_indices` / `torque_limit` — from the URDF.
- One block per policy (list names in `rl_policy_names`, add a same-named block
  below), fill `policy_path` + the `[TRAIN]` fields.

Then place the `*.onnx` into `config/<robot>/`.

### 3. Run

```bash
# Sim (start the backend in a separate terminal):
./scripts/run_sim.sh --backend mujoco --robot <robot>
# Deploy App:
./scripts/run_policy.sh <policy> <robot>          # <policy> from rl_policy_names
```

`--robot` is passed through to the backend by
[run_sim.sh](../scripts/run_sim.sh); no script edits needed.

## The three most common causes of failure to stand

1. **`joints` order ≠ MJCF actuator order** → mujoco assertion error (this is
   intended — align them rather than working around the mismatch).
2. **`stand_jpos` written symmetric** → an asymmetric robot (front/rear legs
   differ) pitches over. On d1 the front thigh is `0.8`, rear `1.0` — do not set
   them equal.
3. **Wrong IMU convention** (quaternion `w,x,y,z` + body-frame gyro) → silent
   divergence. Mounted level, projected gravity should be ≈ `[0,0,-1]`.
   Cross-check against the mujoco backend.

## Validation checklist

```bash
# Source the env first, or the import fails outright (simbase, ddt_msgs not on path):
source env.sh                 # (or: source install/setup.bash)
# spec loads from config:
python3 -c "import sys; sys.path.insert(0,'sim'); from simbase import spec; s=spec('<robot>'); print(len(s.joint_names), list(s.hold_pose))"
```
- [ ] `spec('<robot>')` prints the right joint count + stand_jpos.
- [ ] `run_sim.sh --backend mujoco --robot <robot>` starts with **no assertion error**.
- [ ] `num_obs` / `num_actions` match the ONNX (the App dim-checks and refuses to run on mismatch).
- [ ] With the policy loaded, `standup → rl`, pitch/roll converge without diverging.

## Architecture (why no code changes)

- `spec(robot)` in [sim/simbase.py](../sim/simbase.py) reads `joints` +
  `stand_jpos` from `config/<robot>/controllers.yaml` → any `SimBackend` subclass
  receives them automatically, nothing hard-coded.
- Backends themselves are **discovered by adding a file**: `sim/<name>_sim.py`
  (pure Python) or `sim/<name>/run.sh` (framework), see
  [sim/BACKEND.md](../sim/BACKEND.md).
- [deploy/rl_inference.py](../deploy/rl_inference.py) reads the policy block from
  the same config — **sim and deploy share one source of truth**, no drift.
