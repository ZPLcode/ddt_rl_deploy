#!/usr/bin/env python3
"""Transport-free RL policy engine (the "brain").

No ROS/DDS here: RobotState + Command in, JointCommand out. Obs assembly and
the MIT-PD decode mirror the vendor C++ FSMState_RL* controllers.
"""

import math
from dataclasses import dataclass, field

import numpy as np

try:
    import onnxruntime as ort
except ImportError:  # pragma: no cover - surfaced by the shell before use
    ort = None


# --------------------------------------------------------------------------- #
#  Math helpers
# --------------------------------------------------------------------------- #

def quat_to_rotation_matrix(w: float, x: float, y: float, z: float) -> np.ndarray:
    """Quaternion (w, x, y, z) → 3x3 rotation matrix (body → world)."""
    return np.array([
        [1 - 2*(y*y + z*z),  2*(x*y - w*z),      2*(x*z + w*y)],
        [2*(x*y + w*z),      1 - 2*(x*x + z*z),  2*(y*z - w*x)],
        [2*(x*z - w*y),      2*(y*z + w*x),      1 - 2*(x*x + y*y)],
    ], dtype=np.float32)


def euler_from_quat(w: float, x: float, y: float, z: float):
    """Quaternion (w, x, y, z) → (roll, pitch, yaw) radians."""
    roll  = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch = math.asin(max(-1.0, min(1.0, 2*(w*y - z*x))))
    yaw   = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return roll, pitch, yaw


# --------------------------------------------------------------------------- #
#  Plain data crossing the transport ↔ brain boundary
# --------------------------------------------------------------------------- #

@dataclass
class RobotState:
    """A single measured snapshot; the transport shell fills it each tick."""
    stamp: float                       # source msg time, s (debug only)
    gyro: np.ndarray                   # (3,) body angular velocity
    quat_wxyz: np.ndarray              # (4,) orientation w,x,y,z
    joint_pos: dict                    # joint name -> position (rad)
    joint_vel: dict                    # joint name -> velocity (rad/s)


@dataclass
class Command:
    """High-level user command (teleop)."""
    twist_linear: np.ndarray           # (3,) vx, vy, vz
    twist_angular: np.ndarray          # (3,) wx, wy, wz
    pose_rpy: np.ndarray               # (3,) head roll, pitch, yaw


@dataclass
class JointCommand:
    """Decoded per-joint MIT command; the shell serialises it onto DDS."""
    name: list = field(default_factory=list)
    position: list = field(default_factory=list)
    velocity: list = field(default_factory=list)
    effort: list = field(default_factory=list)
    kp: list = field(default_factory=list)
    kd: list = field(default_factory=list)


@dataclass
class PolicyConfig:
    """Everything the engine needs, resolved by the shell (no ROS types)."""
    onnx_path: str
    policy_type: str
    output_name: str
    num_actions: int
    history_len: int
    obs_names: list
    cmd_names: list
    cmd_scale: np.ndarray
    cmd_gain: np.ndarray
    cmd_min: np.ndarray
    cmd_max: np.ndarray
    ang_vel_scale: float
    dof_pos_scale: float
    dof_vel_scale: float
    wheel_joints: set
    default_angles: np.ndarray
    action_scales: np.ndarray
    joint_kp: np.ndarray
    joint_kd: np.ndarray
    control_type: str
    joint_names: list
    episode_length: float
    control_dt: float


# --------------------------------------------------------------------------- #
#  Observation registry
# --------------------------------------------------------------------------- #
#
# name -> (dim, compute method). dim is an int or callable(cfg)->int.
# dim and compute live together so they cannot drift; adding an obs = one entry
# + one _obs_<name> method. config picks which obs run and their order.

_OBS_SPECS = {
    'ang_vel':          (3,                                                   '_obs_ang_vel'),
    'gravity':          (3,                                                   '_obs_gravity'),
    'commands':         (lambda cfg: len(cfg.cmd_names),                      '_obs_commands'),
    'dof_pos':          (lambda cfg: cfg.num_actions,                         '_obs_dof_pos'),
    'dof_pos_nwp':      (lambda cfg: cfg.num_actions - len(cfg.wheel_joints), '_obs_dof_pos_nwp'),
    'dof_vel':          (lambda cfg: cfg.num_actions,                         '_obs_dof_vel'),
    'last_actions':     (lambda cfg: cfg.num_actions,                         '_obs_last_actions'),
    'phases':           (6,                                                   '_obs_phases'),
    'ref_motion_phase': (1,                                                   '_obs_ref_motion_phase'),
    'action_rescale':   (1,                                                   '_obs_action_rescale'),
}


# --------------------------------------------------------------------------- #
#  Policy engine
# --------------------------------------------------------------------------- #

class PolicyEngine:
    """Stateful (obs history, last action, integrated height), no transport,
    no internal clock: caller drives infer() at control_dt and decode() on
    each publish."""

    def __init__(self, cfg: PolicyConfig, logger=None):
        self.cfg = cfg
        self._log = logger
        self._session = None
        self._input_names = []
        if cfg.onnx_path:
            if ort is None:
                raise RuntimeError('onnxruntime not available')
            self._session = ort.InferenceSession(cfg.onnx_path)
            self._input_names = [i.name for i in self._session.get_inputs()]
            self._info(f'Loaded model: {cfg.onnx_path} | inputs={self._input_names}')
            self._check_model_dims()
        else:
            self._warn('onnx_path not set — inference disabled')
        self._init_memory()

    def _check_model_dims(self):
        """Fail at load time when config and onnx shapes disagree (e.g. 57-obs
        config with a 58-obs model) instead of a cryptic runtime error."""
        cfg = self.cfg
        obs_dim = sum(self._obs_dim(n) for n in cfg.obs_names)
        ins = self._session.get_inputs()

        def ints(shape):    
            return [d for d in shape if isinstance(d, int)]

        if cfg.policy_type == 'np3o':
            expected = [[1, obs_dim], [1, cfg.history_len, obs_dim]]
        elif cfg.policy_type == 'ppo':
            expected = [[1, (cfg.history_len + 1) * obs_dim]]
        else:              
            return
        for inp, exp in zip(ins, expected):
            got = ints(inp.shape)
            if got and got != exp:
                raise ValueError(
                    f'model/config mismatch: model input "{inp.name}" is {got} '
                    f'but the config implies {exp} '
                    f'(observations_name sums to {obs_dim} dims, '
                    f'history_len={cfg.history_len}, policy_type={cfg.policy_type}). '
                    f'Check that policy_name and the onnx file are a matching pair.')

    # ------------------------------------------------------------------ #

    def _info(self, m):
        if self._log is not None:
            self._log.info(m)

    def _warn(self, m):
        if self._log is not None:
            self._log.warn(m)

    @property
    def last_actions(self) -> np.ndarray:
        return self._last_actions

    # ------------------------------------------------------------------ #

    def _obs_dim(self, name: str) -> int:
        spec = _OBS_SPECS.get(name)
        if spec is None:
            raise ValueError(f'Unknown observation name: {name}')
        dim = spec[0]
        return dim(self.cfg) if callable(dim) else dim

    def _init_memory(self):
        cfg = self.cfg
        # current observations
        self._obs = {n: np.zeros(self._obs_dim(n), dtype=np.float32)
                     for n in cfg.obs_names}
        # history
        self._obs_hist = {n: np.zeros((cfg.history_len, self._obs_dim(n)), dtype=np.float32)
                          for n in cfg.obs_names}
        self._last_actions = np.zeros(cfg.num_actions, dtype=np.float32)

        self._current_time = 0.0
        self._current_height = 0.0            
        self._infer_dt = self.cfg.control_dt   
        self._first_frame = True              

    # ------------------------------------------------------------------ #
    #  Observation update
    # ------------------------------------------------------------------ #

    def _update_observations(self, state: RobotState, cmd: Command):
        cfg = self.cfg
        # shift history 
        if cfg.history_len == 1:
            for n in cfg.obs_names:
                self._obs_hist[n][0] = self._obs[n]
        else:
            for n in cfg.obs_names:
                self._obs_hist[n][:-1] = self._obs_hist[n][1:]
                self._obs_hist[n][-1]  = self._obs[n]

        for n in cfg.obs_names:
            self._obs[n] = getattr(self, _OBS_SPECS[n][1])(state, cmd)

        if self._first_frame:
            for n in cfg.obs_names:
                self._obs_hist[n][:] = self._obs[n]
            self._first_frame = False

    
    def _obs_ang_vel(self, state, cmd):
        raw = state.gyro * self.cfg.ang_vel_scale
        return 0.03 * self._obs['ang_vel'] + 0.97 * raw

    def _obs_gravity(self, state, cmd):
        w, x, y, z = state.quat_wxyz
        R = quat_to_rotation_matrix(w, x, y, z)
        return R.T @ np.array([0.0, 0.0, -1.0], dtype=np.float32)

    def _obs_commands(self, state, cmd):
        cfg = self.cfg
        cmds = np.empty(len(cfg.cmd_names), dtype=np.float32)
        for i, cname in enumerate(cfg.cmd_names):
            if cname == 'lin_vel_x':
                raw = cfg.cmd_gain[i] * cmd.twist_linear[0]
            elif cname == 'lin_vel_y':
                raw = cfg.cmd_gain[i] * cmd.twist_linear[1]
            elif cname == 'ang_vel_z':
                raw = cfg.cmd_gain[i] * cmd.twist_angular[2]
            elif cname == 'base_height':
                self._current_height = float(np.clip(
                    self._current_height + cmd.twist_linear[2] * self._infer_dt,
                    cfg.cmd_min[i], cfg.cmd_max[i]))
                raw = cfg.cmd_gain[i] * self._current_height
            elif cname == 'head_roll':
                raw = cfg.cmd_gain[i] * cmd.pose_rpy[0]
            elif cname == 'head_pitch':
                raw = cfg.cmd_gain[i] * cmd.pose_rpy[1]
            elif cname == 'head_yaw':
                raw = cfg.cmd_gain[i] * cmd.pose_rpy[2]
            else:
                self._warn(f'Unknown command: {cname}')
                raw = 0.0
            cmds[i] = float(np.clip(raw, cfg.cmd_min[i], cfg.cmd_max[i])) \
                      * cfg.cmd_scale[i]
        return cmds

    def _obs_dof_pos(self, state, cmd):
        cfg = self.cfg
        pos = np.zeros(cfg.num_actions, dtype=np.float32)
        for ai, jname in enumerate(cfg.joint_names[:cfg.num_actions]):
            if jname not in cfg.wheel_joints:
                pos[ai] = (state.joint_pos.get(jname, 0.0)
                           - float(cfg.default_angles[ai]))
        return pos * cfg.dof_pos_scale

    def _obs_dof_pos_nwp(self, state, cmd):
        cfg = self.cfg
        nwp = cfg.num_actions - len(cfg.wheel_joints)
        pos = np.zeros(nwp, dtype=np.float32)
        idx = 0
        for ai, jname in enumerate(cfg.joint_names[:cfg.num_actions]):
            if jname not in cfg.wheel_joints and idx < nwp:
                pos[idx] = (state.joint_pos.get(jname, 0.0)
                            - float(cfg.default_angles[ai]))
                idx += 1
        return pos * cfg.dof_pos_scale

    def _obs_dof_vel(self, state, cmd):
        cfg = self.cfg
        vel = np.zeros(cfg.num_actions, dtype=np.float32)
        for ai, jname in enumerate(cfg.joint_names[:cfg.num_actions]):
            vel[ai] = state.joint_vel.get(jname, 0.0)
        return vel * cfg.dof_vel_scale

    def _obs_last_actions(self, state, cmd):
        return self._last_actions.copy()

    def _obs_phases(self, state, cmd):
        t = self._current_time * math.pi / 2.0
        return np.array([
            math.sin(t),     math.cos(t),
            math.sin(t/2),   math.cos(t/2),
            math.sin(t/4),   math.cos(t/4),
        ], dtype=np.float32)

    def _obs_ref_motion_phase(self, state, cmd):
        phase_val = (self._current_time / self.cfg.episode_length
                     if self.cfg.episode_length > 1e-4 else 0.0)
        return np.array([phase_val], dtype=np.float32)

    def _obs_action_rescale(self, state, cmd):
        return np.array([0.25], dtype=np.float32)

    # ------------------------------------------------------------------ #
    #  Build inference inputs
    # ------------------------------------------------------------------ #

    def _build_np3o(self):
        """np3o: nn_input0=[1, obs_dim], nn_input1=[1, history_len, obs_dim]."""
        cfg = self.cfg
        obs = np.concatenate([self._obs[n] for n in cfg.obs_names])  # (obs_dim,)
        hist = np.stack([
            np.concatenate([self._obs_hist[n][i] for n in cfg.obs_names])
            for i in range(cfg.history_len)
        ])  # (history_len, obs_dim)
        return [obs[None], hist[None]]  # (1, obs_dim), (1, history_len, obs_dim)

    def _build_ppo(self):
        """Single input: [obs_hist (per-obs-name, oldest→newest), obs]."""
        cfg = self.cfg
        rows = []
        for n in cfg.obs_names:
            for i in range(cfg.history_len):
                rows.append(self._obs_hist[n][i])
        obs_hist = np.concatenate(rows)
        obs = np.concatenate([self._obs[n] for n in cfg.obs_names])
        combined = np.concatenate([obs_hist, obs])
        return [combined[None]]

    def _build_asap(self):
        """Single input with custom ordering (newest→oldest history)."""
        sorted_hist_names = [
            'last_actions', 'ang_vel', 'dof_pos', 'dof_vel', 'gravity', 'ref_motion_phase',
        ]
        rows = []
        for n in sorted_hist_names:
            if n not in self._obs_hist:
                continue
            for i in range(self.cfg.history_len - 1, -1, -1):   # newest → oldest
                rows.append(self._obs_hist[n][i])
        obs_hist = np.concatenate(rows) if rows else np.array([], dtype=np.float32)

        sorted_out_names = [
            'last_actions', 'ang_vel', 'dof_pos', 'dof_vel',
            'gravity', 'ref_motion_phase',
        ]
        pieces = []
        for n in sorted_out_names:
            if n == 'history':
                pieces.append(obs_hist)
            elif n in self._obs:
                pieces.append(self._obs[n])
        combined = np.concatenate(pieces)
        return [combined[None]]

    # ------------------------------------------------------------------ #
    #  Inference
    # ------------------------------------------------------------------ #

    def _run_inference(self):
        if self._session is None:
            return

        if self.cfg.policy_type == 'np3o':
            inputs = self._build_np3o()
        elif self.cfg.policy_type == 'ppo':
            inputs = self._build_ppo()
        elif self.cfg.policy_type == 'asap':
            inputs = self._build_asap()
        else:
            self._warn(f'Unknown policy_type: {self.cfg.policy_type}')
            return

        feed = {name: data.astype(np.float32)
                for name, data in zip(self._input_names, inputs)}
        outputs = self._session.run(None, feed)
        self._last_actions = np.array(outputs[0], dtype=np.float32).flatten()

    # ------------------------------------------------------------------ #
    #  Decode
    # ------------------------------------------------------------------ #

    def decode(self, state: RobotState) -> JointCommand:
        cfg = self.cfg
        out = JointCommand()
        for ai, jname in enumerate(cfg.joint_names):
            action  = float(self._last_actions[ai])
            scale   = float(cfg.action_scales[ai])
            default = float(cfg.default_angles[ai])
            kp      = float(cfg.joint_kp[ai])
            kd      = float(cfg.joint_kd[ai])

            out.name.append(jname)

            if jname in cfg.wheel_joints:
                if cfg.control_type == 'P':
                    out.position.append(0.0)
                    out.velocity.append(0.0)
                    out.effort.append((action * scale + default) * kp
                                      - kd * state.joint_vel.get(jname, 0.0))
                    out.kp.append(0.0)
                    out.kd.append(0.0)
                else:  # P_V
                    out.position.append(0.0)
                    out.velocity.append(action * scale)
                    out.effort.append(0.0)
                    out.kp.append(0.0)
                    out.kd.append(kd)
            else:
                out.position.append(action * scale + default)
                out.velocity.append(0.0)
                out.effort.append(0.0)
                out.kp.append(kp)
                out.kd.append(kd)
        return out

    # ------------------------------------------------------------------ #
    #  One inference (
    # ------------------------------------------------------------------ #

    def infer(self, state: RobotState, cmd: Command):
        """One inference step: assemble obs, run onnx, update _last_actions.
        Caller owns the cadence (wall-clock timer at control_dt). No stamp
        gate, so the state source must run at real time."""
        self._infer_dt = self.cfg.control_dt
        self._update_observations(state, cmd)
        self._run_inference()
        self._current_time += self.cfg.control_dt     # policy phase clock
