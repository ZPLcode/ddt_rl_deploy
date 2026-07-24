# rl_inference dataflow reference

Summary of the sensor → policy → motor loop. `rl_inference.py` (shell) +
`policy_engine.py` (brain) are the Python port of the C++ `FSMState_RL`.

## Main line (every tick after spin)

```
joint_states in ─► callback cache ─► dual wall-clock timers ─► infer/decode ─► publish cmd ─► motors
    _joint_cb         RobotState        infer@50Hz             engine.infer     _publish
                                        publish@200Hz          engine.decode    JointControlCommand
```

## Modes (self-contained safety shell — one string variable, no state-machine class)

```
standup ──done──► rl ──posture over limit (roll>30° / pitch>45°)──► damping (locked)
   any mode ──joint_states stalled >0.2s──► damping (locked)
   exit any time (Ctrl-C / SIGTERM) ──► main finally: emit 20 damping cmds ──► quit
```

- **standup**: ports `FSMState_TransformUp` — two-stage linear interp: fold
  (duration = fold_timer × max joint deviation) → stand (stand_timer) + 100-tick
  settle; dedicated kp/kd/feedforward from the yaml `transform_up` section;
  wheels fixed at kp=0 / kd damping / no fold.
- **damping**: ports `FSMState_Passive` — all zeros, legs kd=5, wheels kd=0.
- **exit damping**: the downstream bridge holds the last command indefinitely (per
  official SDK docs); exiting under high kp leaves the robot rigidly seized, so
  SIGINT/SIGTERM both route through the finally block's damping burst.

## Functions (execution order)

| Function | One line |
|---|---|
| `_declare_params` | Declare all ROS params + defaults |
| `_load_yaml_section` / `_find_key_recursive` | Extract one policy section from controllers.yaml (dotted path → recursive whole-tree search) |
| `_load_params` + `y()` | Merge "YAML first, ROS default fallback" → `self._xxx` |
| `_init_onnx` | Load the .onnx into a session, read input names (inputs are fed by name) |
| `_obs_dim` + `_init_state` | Preallocate obs / history ring / caches per the dim table (dims must sum to model input, e.g. 57) |
| `_create_subscriptions` + 4 callbacks | Subscribe joint_states/imu/cmd_twist/cmd_pose; callbacks only fill caches |
| `_control_loop` / `step` | Gate whether to infer; publish every tick |
| `_update_observations` | Shift history first, then compute new obs; assemble section by section per the obs registry |
| `_build_np3o/ppo/asap` | Pack obs+history into the input layout the model expects |
| `_run_inference` | Select layout by policy_type → session.run → `_last_actions` |
| `_publish_joint_command` / `decode` | Decode into the MIT 5-tuple, publish command/joint_command |

## Pitfalls (learned in practice)

| Pitfall | Note |
|---|---|
| **Inference rate drift** | Frame-gating with `_iter % decimation` inside `_joint_cb` ties the rate to joint_states arrival. Drive inference from a wall-clock timer at `control_dt` instead → locks 50Hz. Requires a real-time state source (sim self-throttled to realtime, real robot inherently realtime); a non-realtime sim runs the wrong rate. |
| **Dead velocity commands** | Keyboard / rl_controller publish `Twist`. Subscribing to `TwistStamped` type-mismatches and DDS drops it silently — subscribe to `Twist`. |
| **Missed wheel parsing** | Recognizing only `wheel_names` misses configs (d1) that give `wheel_indices` → wheels get position-controlled as ordinary joints and pollute dof_pos. Handle the `wheel_indices → names` branch too. |
| **First-frame zero history** | Isaac Lab's first append after reset fills history with the current obs (circular_buffer.py:136-139). Deployment must prefill the first frame too, else the first ~0.2s feeds zero history (never seen in training). |
| **Low-pass reads stale value** | `_update_observations` shifts history first (stores old `_obs`), then computes new values; the ang_vel low-pass `0.03*old + 0.97*new` relies on `_obs` still being old at that point — order must not flip. |
| **Gravity needs transpose** | `R.T @ [0,0,-1]`: `quat_to_rotation_matrix` returns body→world; invert (= transpose) to get gravity in the body frame. |
| **base_height is integrated** | In the command, base_height integrates twist.linear.z (`height += vz*infer_dt`) then clips — not passed through directly. |
| **last_actions stores raw** | `_last_actions` = raw network output (unscaled); the next tick feeds it back into obs, and decode uses it too. |
| **Sparse inference, dense publish** | Infer every N frames, publish every frame; wheel P-mode `effort = (…)·kp − kd·live_wheel_vel` relies on per-frame publish to refresh damping. |
| **Joint order = policy order** | No reindexing; relies on "config joint order == training action order" + named commands. A wrong order silently misaligns. |

## Key contracts

- **MIT 5-tuple** `{position, velocity, effort, kp, kd}` → downstream `τ = effort + kp·(position−q) + kd·(velocity−dq)`
  - Legs: `position = action·scale + default`, kp/kd handled by downstream PD.
  - Wheel P: `effort = (action·scale+default)·kp − kd·live_wheel_vel`, rest 0 (downstream τ = effort).
  - Wheel P_V: `velocity = action·scale`, keep only kd (downstream τ = kd·(vel−dq)).
- **Three input layouts** (must match the training framework):
  - np3o: two inputs `(1,obs)` + `(1,hist,obs)`, history stays 2D.
  - ppo: one input, history flattened (obs-name major, frame minor) + current.
  - asap: one input, obs subset, history newest→oldest.
- **History ring layout**: `[t-H+1 … t-1, t]`, row 0 oldest, row -1 newest (matches Isaac Lab CircularBuffer).
