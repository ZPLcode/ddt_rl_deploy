# Sim / real backend contract

The deploy App (`deploy/rl_inference.py`) is **backend-agnostic**: it depends only
on the ROS 2 topic contract below. Any backend — a simulator, or a real robot's
onboard unit — integrates by satisfying the same contract, **with no changes to the
App**. This is how sim==real is achieved.

This is the specification for authors of a new backend. It plays the same interface
role as a robot-interface header, but lives at the process boundary — a ROS topic
contract, not an interface class compiled into the App.

## The three backend responsibilities

| # | Responsibility | topic | Type |
|---|---|---|---|
| 1 | **Publish joint state** | `joint_states` | `sensor_msgs/JointState` |
| 2 | **Publish IMU** | `imu_sensor_broadcaster/imu` | `sensor_msgs/Imu` |
| 3 | **Receive joint commands and execute them** | `command/joint_command` | `ddt_msgs/JointControlCommand` |

Plus one implicit requirement: **advance in real time (1× wall clock)** (see below).

---

## 1. `joint_states` (backend publishes)

- `name[]`: joint names, **must match the `joints` list in
  `config/<robot>/controllers.yaml`** (the App matches by name; order is not
  enforced, but the names must correspond).
- `position[]` / `velocity[]`: per-joint angle / angular velocity. **Required**.
- `effort[]`: optional (App ignores it).
- `header.stamp`: fill with physics time where available (the App runs on wall-clock
  timing and does not depend on it, but it aids debugging).
- **QoS**: `BEST_EFFORT` / `VOLATILE` / `KEEP_LAST(1)` (matches the App's
  subscription; RELIABLE publishers also work).

## 2. `imu_sensor_broadcaster/imu` (backend publishes)

- `orientation`: body attitude quaternion. The App reads `w, x, y, z` (the ROS
  message fields are x,y,z,w — fill by field name). **Required**.
- `angular_velocity`: body-frame angular velocity (gyro). **Required**.
- `linear_acceleration`: optional (App ignores it; filling it is harmless).
- `header.frame_id`: `trunk_imu` recommended.
- QoS: same as `joint_states`.

## 3. `command/joint_command` (backend receives and executes)

The App sends one MIT 5-tuple per joint; the backend runs PD + feedforward:

```
τ_joint = effort + kp · (position − q) + kd · (velocity − dq)
```

- Fields: `header, name[], kp[], kd[], position[], velocity[], effort[]` (matched by `name`).
- **Legs**: position target + kp/kd → position PD.
- **Wheel (P_V)**: velocity target + kd (kp=0) → velocity control.
- **Wheel (P)**: effort already computed by the App, kp=kd=0 → apply torque directly.
- The backend **recomputes every physics step from the latest cached command**
  (zero-order hold); command arrival cadence does not affect torque smoothness.
- QoS: `RELIABLE` / `KEEP_LAST(10)` (matches the App's publisher).

---

## Conventions (commonly missed)

- **Real-time throttle**: the backend must throttle physics to ≈1× wall clock
  (e.g. `sleep(dt − step_cost)`). The App locks 50Hz inference with a wall-clock
  timer, **which assumes the state source is real-time**; running non-real-time
  (headless full-speed) corrupts the policy rate. Real robots satisfy this
  inherently; sims must throttle themselves.
- **Namespace**: if the env var `ROBOT_NS` is set, all topics take a
  `<ROBOT_NS>/` prefix (the App adds it too; both sides must agree).
- **Initial pose**: before the first command arrives, the backend should PD-hold
  a stable pose (the robot must not collapse before the App connects). The App's
  standup mode lifts it from the current measured pose.
- **A wrong joint-name / IMU convention does not error; it misbehaves silently** — a name
  that does not match reads 0 for that joint and its command is dropped; a wrong
  quaternion / angular-velocity frame diverges the policy. Cross-check against a
  reference implementation.

---

## Reference implementations & adding a new backend

- **Shared core**: `sim/simbase.py` is the single source of truth — `RobotSpec`
  (joint order / hold pose / gains) + `SimBackend(Node)` (command cache, MIT-PD,
  state+imu I/O and QoS, all here). **A new backend subclasses `SimBackend` and
  writes only the engine binding** (read q/dq, apply force, read IMU, drive the
  step loop); contract details are not recopied and cannot drift between backends
  (duplicating the contract details previously introduced a bug).
- **References**: `sim/mujoco_sim.py` (most complete, with passive viewer +
  wall-clock throttle) and `sim/webots/webots_sim.py` (smallest, ~100 lines) are
  both examples.
- **Lightweight backend** (pure Python + DDS, e.g. PyBullet): add
  `sim/<name>_sim.py` subclassing `SimBackend`.
  `./scripts/run_sim.sh --backend <name>` auto-discovers it.
- **Framework backend** (Gazebo/Webots etc., each needs its own ros2
  integration): add a directory `sim/<name>/` with an executable
  `sim/<name>/run.sh` as the entry point (it starts the launch / controller nodes
  internally; the only requirement is that it satisfies the topic contract above).
  Also auto-discovered by `--backend <name>`.
- **Real robot**: no backend to write — the onboard unit satisfies the same contract
  (it takes over after entering `08 SDK Mode` on the remote). Run
  `run_policy.sh` + `run_teleop.sh` with the real robot on the other side.

`./scripts/run_sim.sh --list-backends` lists the backends currently installed.
