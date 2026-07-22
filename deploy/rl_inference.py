#!/usr/bin/env python3
# Standalone ROS 2 node: subscribes to joint_states / IMU / cmd_twist / cmd_pose,
# builds observations identical to FSMState_RL::update_observations(), and runs
# ONNX inference.  Supports policy_type: np3o / ppo / asap.

import math
import os
import sys
import yaml

from ament_index_python.packages import get_package_share_directory

import numpy as np
import rclpy
from ddt_msgs.msg import JointControlCommand
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu, JointState

try:
    import onnxruntime as ort
except ImportError:
    print("onnxruntime not found.  Install with: pip install onnxruntime")
    sys.exit(1)


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
#  Node
# --------------------------------------------------------------------------- #

class RLInferenceNode(Node):
    def __init__(self):
        super().__init__('rl_inference_node')
        self._declare_params()
        self._load_params()
        self._init_onnx()
        self._init_state()
        robot_ns = os.environ.get('ROBOT_NS', '').strip('/')
        self._tp = (lambda t: f'{robot_ns}/{t}') if robot_ns else (lambda t: t)
        self._create_subscriptions()
        self._joint_cmd_pub = self.create_publisher(
            JointControlCommand, self._tp('command/joint_command'), 10)
        # Own-clock control loop (the pattern every reference deploy uses:
        # LeggedLab RecurrentThread / roboparty SCHED_FIFO loops / rl_sar
        # LoopFunc / sdk_deploy 5 ms timerfd): a fixed 5 ms wall timer samples
        # the state cache; inference itself stays paced at control_dt by the
        # stamp gate in _control_loop, so the policy rate follows physics time.
        self._timer = self.create_timer(0.005, self._timer_cb)
        self.get_logger().info(
            f'RLInferenceNode ready | policy={self._policy_type}'
            f' num_actions={self._num_actions} history_len={self._history_len}'
            f' control_dt={self._control_dt:.4f}s ({1.0 / self._control_dt:.1f}Hz, time-gated)'
            f' loop=5ms-timer+state-cache'
            f' ns={robot_ns or "(none)"}'
        )

    # ----------------------------------------------------------------------- #
    #  Parameter declaration / loading
    # ----------------------------------------------------------------------- #

    def _declare_params(self):
        self.declare_parameter('config_file', '')   # path to params yaml
        self.declare_parameter('policy_name', '')   # key inside the yaml
        self.declare_parameter('onnx_path', '')
        self.declare_parameter('policy_type', 'np3o')       # np3o / ppo / asap
        self.declare_parameter('output_name', 'actions')
        self.declare_parameter('num_actions', 12)
        self.declare_parameter('history_len', 10)
        self.declare_parameter('observations_name',
            ['ang_vel', 'gravity', 'commands', 'dof_pos', 'dof_vel', 'last_actions'])
        self.declare_parameter('commands_name', ['lin_vel_x', 'lin_vel_y', 'ang_vel_z'])
        self.declare_parameter('commands_scale', [2.0, 2.0, 0.25])
        self.declare_parameter('commands_gain', [1.0, 1.0, 1.0])
        self.declare_parameter('min_commands', [-1.0, -1.0, -1.0])
        self.declare_parameter('max_commands', [1.0, 1.0, 1.0])
        self.declare_parameter('ang_vel_scale', 0.25)
        self.declare_parameter('dof_pos_scale', 1.0)
        self.declare_parameter('dof_vel_scale', 0.05)
        self.declare_parameter('wheel_joints', [])
        self.declare_parameter('default_joint_angles', [])
        self.declare_parameter('action_scales', [0.25])
        self.declare_parameter('joint_kp', [0.0])
        self.declare_parameter('joint_kd', [0.0])
        self.declare_parameter('control_type', 'P')    # P or P_V
        self.declare_parameter('episode_length', 0.0)

    def _load_yaml_section(self, config_file: str, policy_name: str):
        """Return (section_dict, full_yaml_dict)."""
        with open(config_file, 'r') as f:
            full = yaml.safe_load(f)
        if not policy_name:
            return (full if isinstance(full, dict) else {}), full
        # Try dot-separated path first: 'a.b.c'
        try:
            node = full
            for key in policy_name.split('.'):
                node = node[key]
            if isinstance(node, dict):
                return node, full
        except (KeyError, TypeError):
            pass
        # Fall back to recursive search for the key anywhere in the tree
        result = self._find_key_recursive(full, policy_name)
        if result is not None and isinstance(result, dict):
            return result, full
        raise KeyError(policy_name)

    def _find_key_recursive(self, data, key: str):
        if not isinstance(data, dict):
            return None
        if key in data:
            return data[key]
        for v in data.values():
            result = self._find_key_recursive(v, key)
            if result is not None:
                return result
        return None

    def _load_params(self):
        g = self.get_parameter
        config_file = g('config_file').value
        policy_name = g('policy_name').value
        cfg: dict = {}
        full_yaml: dict = {}
        if config_file:
            try:
                cfg, full_yaml = self._load_yaml_section(config_file, policy_name)
                self.get_logger().info(f'Loaded config: {config_file} [{policy_name}]')
            except Exception as e:
                self.get_logger().error(f'Failed to load config: {e}')

        def y(yaml_key, ros_param):
            """YAML value takes precedence over the ROS2 param default."""
            return cfg[yaml_key] if yaml_key in cfg else g(ros_param).value

        # policy_path: resolve relative paths against the package share directory,
        # matching the C++ ament_index_cpp::get_package_share_directory() behaviour.
        policy_path = str(cfg.get('policy_path', ''))
        if policy_path and not os.path.isabs(policy_path):
            # Prefer an onnx sitting next to the config file (keeps a standalone
            # deploy repo self-contained); fall back to the rl_controller package
            # share dir for the legacy in-tree layout.
            cfg_dir = os.path.dirname(os.path.abspath(config_file)) if config_file else ''
            local = os.path.join(cfg_dir, os.path.basename(policy_path)) if cfg_dir else ''
            if local and os.path.isfile(local):
                policy_path = local
            else:
                try:
                    policy_path = os.path.join(
                        get_package_share_directory('rl_controller'), policy_path)
                except Exception:
                    policy_path = local or policy_path
        self._onnx_path = policy_path or g('onnx_path').value

        self._policy_type = str(y('policy_type',  'policy_type'))
        self._output_name = str(y('output_name',  'output_name'))
        self._num_actions = int(y('num_actions',  'num_actions'))
        self._history_len = int(y('history_len',  'history_len'))
        self._obs_names   = list(y('observations_name', 'observations_name'))
        self._cmd_names   = list(y('commands_name',     'commands_name'))
        self._cmd_scale   = np.array(y('commands_scale', 'commands_scale'), dtype=np.float32)
        self._cmd_gain    = np.array(y('commands_gain',  'commands_gain'),  dtype=np.float32)
        self._cmd_min     = np.array(y('min_commands',   'min_commands'),   dtype=np.float32)
        self._cmd_max     = np.array(y('max_commands',   'max_commands'),   dtype=np.float32)
        self._ang_vel_scale = float(y('ang_vel_scale', 'ang_vel_scale'))
        self._dof_pos_scale = float(y('dof_pos_scale', 'dof_pos_scale'))
        self._dof_vel_scale = float(y('dof_vel_scale', 'dof_vel_scale'))
        joints_from_yaml = self._find_key_recursive(full_yaml, 'joints')
        self._joint_names = joints_from_yaml if isinstance(joints_from_yaml, list) else []
        if self._joint_names:
            self.get_logger().info(f'joints from YAML ({len(self._joint_names)})')
        wheel_names_from_yaml = self._find_key_recursive(full_yaml, 'wheel_names')
        wheel_idx_from_yaml = self._find_key_recursive(full_yaml, 'wheel_indices')
        wheel_joints_param = list(g('wheel_joints').value)
        if wheel_joints_param:
            self._wheel_joints = set(wheel_joints_param)
        elif isinstance(wheel_names_from_yaml, list):
            self._wheel_joints = set(wheel_names_from_yaml)
            self.get_logger().info(f'wheel_joints from YAML wheel_names: {self._wheel_joints}')
        elif isinstance(wheel_idx_from_yaml, list) and self._joint_names:
            # d1's controllers.yaml provides wheel_indices (the C++ controller
            # reads indices), NOT wheel_names.  Missing this left wheels position-
            # controlled at kp=60 with their angle leaking into dof_pos -> the
            # policy diverged and the legs twisted up.
            self._wheel_joints = {self._joint_names[i] for i in wheel_idx_from_yaml
                                  if 0 <= i < len(self._joint_names)}
            self.get_logger().info(
                f'wheel_joints from YAML wheel_indices: {sorted(self._wheel_joints)}')
        else:
            self._wheel_joints = set()
            if str(y('control_type', 'control_type')) == 'P_V':
                self.get_logger().warning(
                    'control_type is P_V but no wheel joints resolved — wheels '
                    'will be position-controlled, almost certainly wrong')

        def_angles = list(y('default_joint_angles', 'default_joint_angles'))
        self._default_angles = (
            np.array(def_angles, dtype=np.float32)
            if def_angles
            else np.zeros(self._num_actions, dtype=np.float32)
        )

        scales = list(y('action_scales', 'action_scales'))
        if len(scales) == 1:
            scales = scales * self._num_actions
        self._action_scales = np.array(scales, dtype=np.float32)

        kp = list(y('joint_kp', 'joint_kp'))
        if len(kp) == 1:
            kp = kp * self._num_actions
        self._joint_kp = np.array(kp, dtype=np.float64)

        kd = list(y('joint_kd', 'joint_kd'))
        if len(kd) == 1:
            kd = kd * self._num_actions
        self._joint_kd = np.array(kd, dtype=np.float64)

        self._control_type = str(y('control_type', 'control_type'))

        self._decimation = int(cfg.get('decimation', 4))
        # Inference is time-gated at the training control period, NOT frame-gated
        # on the joint_states count.  Frame-gating tied the policy rate to the
        # joint_states arrival rate (250 Hz / decimation 4 = 62.5 Hz), drifting off
        # the 50 Hz the policy was trained at (train decimation 4 x sim.dt 0.005 =
        # 0.02 s).  control_dt keeps the rate at the trained value regardless.
        self._control_dt = float(cfg.get('control_dt', 0.02))

        self._output_torque_scale = float(cfg.get('output_torque_scale', 1.0))
        self._episode_length = float(y('episode_length', 'episode_length'))

    # ----------------------------------------------------------------------- #
    #  ONNX session
    # ----------------------------------------------------------------------- #

    def _init_onnx(self):
        if self._onnx_path:
            self._session = ort.InferenceSession(self._onnx_path)
            self._input_names = [i.name for i in self._session.get_inputs()]
            self.get_logger().info(
                f'Loaded model: {self._onnx_path} | inputs={self._input_names}')
        else:
            self._session = None
            self.get_logger().warn('onnx_path not set — inference disabled')

    # ----------------------------------------------------------------------- #
    #  State initialisation
    # ----------------------------------------------------------------------- #

    def _obs_dim(self, name: str) -> int:
        if name in ('ang_vel', 'gravity'):
            return 3
        if name == 'commands':
            return len(self._cmd_names)
        if name in ('dof_pos', 'dof_vel', 'last_actions'):
            return self._num_actions
        if name == 'dof_pos_nwp':
            return self._num_actions - len(self._wheel_joints)
        if name == 'phases':
            return 6
        if name in ('ref_motion_phase', 'action_rescale'):
            return 1
        raise ValueError(f'Unknown observation name: {name}')

    def _init_state(self):
        # current observations
        self._obs = {n: np.zeros(self._obs_dim(n), dtype=np.float32)
                     for n in self._obs_names}
        # history: shape (history_len, obs_dim), row 0 = oldest, row -1 = newest
        self._obs_hist = {n: np.zeros((self._history_len, self._obs_dim(n)), dtype=np.float32)
                         for n in self._obs_names}
        self._last_actions = np.zeros(self._num_actions, dtype=np.float32)

        # sensor caches
        self._gyro      = np.zeros(3, dtype=np.float32)
        self._quat_wxyz = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
        self._joint_pos: dict[str, float] = {}
        self._joint_vel: dict[str, float] = {}
        self._twist_linear  = np.zeros(3, dtype=np.float32)
        self._twist_angular = np.zeros(3, dtype=np.float32)
        self._pose_rpy      = np.zeros(3, dtype=np.float32)

        self._iter = 0
        self._current_time = 0.0
        self._current_height = 0.0  # integrated base_height command
        self._infer_dt: float = 0.02          # actual elapsed time between inferences
        self._start_stamp: float | None = None
        self._last_infer_stamp: float | None = None   # stamp of last inference (None = not yet)
        self._state_stamp: float | None = None        # stamp of latest cached joint_states

    # ----------------------------------------------------------------------- #
    #  Subscriptions
    # ----------------------------------------------------------------------- #

    def _create_subscriptions(self):
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Imu,          self._tp('imu_sensor_broadcaster/imu'), self._imu_cb,   qos)
        self.create_subscription(JointState,   self._tp('joint_states'),               self._joint_cb, qos)
        self.create_subscription(Twist,        self._tp('command/cmd_twist'),          self._twist_cb, qos)
        self.create_subscription(PoseStamped,  self._tp('command/cmd_pose'),           self._pose_cb,  qos)

    def _imu_cb(self, msg: Imu):
        o = msg.orientation
        self._quat_wxyz = np.array([o.w, o.x, o.y, o.z], dtype=np.float32)
        v = msg.angular_velocity
        self._gyro = np.array([v.x, v.y, v.z], dtype=np.float32)

    def _joint_cb(self, msg: JointState):
        # Cache-only: control runs on its own timer (_timer_cb), never inside
        # this callback.  Running control here would tie the control cadence to
        # joint_states arrival timing, inheriting any burstiness of the
        # transport (e.g. a render-coupled simulator).
        for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
            self._joint_pos[name] = float(pos)
            self._joint_vel[name] = float(vel)
        self._state_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _timer_cb(self):
        if self._state_stamp is None:
            return  # nothing to control until the first joint_states arrives
        self._control_loop(self._state_stamp)

    def _twist_cb(self, msg: Twist):
        # keyboard_controller and the C++ rl_controller both publish
        # geometry_msgs/Twist (not TwistStamped); DDS matches by type, so a
        # mismatch silently drops every velocity command.
        self._twist_linear  = np.array([msg.linear.x,  msg.linear.y,  msg.linear.z],  dtype=np.float32)
        self._twist_angular = np.array([msg.angular.x, msg.angular.y, msg.angular.z], dtype=np.float32)

    def _pose_cb(self, msg: PoseStamped):
        q = msg.pose.orientation
        r, p, y = euler_from_quat(q.w, q.x, q.y, q.z)
        self._pose_rpy = np.array([r, p, y], dtype=np.float32)

    # ----------------------------------------------------------------------- #
    #  Observation update  (mirrors FSMState_RL::update_observations)
    # ----------------------------------------------------------------------- #

    def _update_observations(self):
        # 1. Shift history — save current (old) obs into history ring
        if self._history_len == 1:
            for n in self._obs_names:
                self._obs_hist[n][0] = self._obs[n]
        else:
            for n in self._obs_names:
                self._obs_hist[n][:-1] = self._obs_hist[n][1:]   # shift towards row 0
                self._obs_hist[n][-1]  = self._obs[n]             # newest at tail

        # 2. Compute new observations from sensor data
        #    NOTE: self._obs[n] still holds the PREVIOUS value here,
        #    which is intentional for the ang_vel low-pass filter below.
        for n in self._obs_names:
            if n == 'ang_vel':
                # 0.03/0.97 low-pass filter matching C++ implementation
                raw = self._gyro * self._ang_vel_scale
                self._obs['ang_vel'] = 0.03 * self._obs['ang_vel'] + 0.97 * raw

            elif n == 'gravity':
                w, x, y, z = self._quat_wxyz
                R = quat_to_rotation_matrix(w, x, y, z)
                # C++ quaternionToRotationMatrix() transposes the matrix (R_w2b),
                # then multiplies by [0,0,-1], giving gravity in body frame.
                # Python quat_to_rotation_matrix returns R_b2w, so we must transpose.
                self._obs['gravity'] = R.T @ np.array([0.0, 0.0, -1.0], dtype=np.float32)

            elif n == 'commands':
                cmds = np.empty(len(self._cmd_names), dtype=np.float32)
                for i, cname in enumerate(self._cmd_names):
                    if cname == 'lin_vel_x':
                        raw = self._cmd_gain[i] * self._twist_linear[0]
                    elif cname == 'lin_vel_y':
                        raw = self._cmd_gain[i] * self._twist_linear[1]
                    elif cname == 'ang_vel_z':
                        raw = self._cmd_gain[i] * self._twist_angular[2]
                    elif cname == 'base_height':
                        # integrate z velocity command over time (mirrors C++ integrate_height)
                        self._current_height = float(np.clip(
                            self._current_height + self._twist_linear[2] * self._infer_dt,
                            self._cmd_min[i], self._cmd_max[i]))
                        raw = self._cmd_gain[i] * self._current_height
                    elif cname == 'head_roll':
                        raw = self._cmd_gain[i] * self._pose_rpy[0]
                    elif cname == 'head_pitch':
                        raw = self._cmd_gain[i] * self._pose_rpy[1]
                    elif cname == 'head_yaw':
                        raw = self._cmd_gain[i] * self._pose_rpy[2]
                    else:
                        self.get_logger().warn(f'Unknown command: {cname}', once=True)
                        raw = 0.0
                    cmds[i] = float(np.clip(raw, self._cmd_min[i], self._cmd_max[i])) \
                              * self._cmd_scale[i]
                self._obs['commands'] = cmds

            elif n == 'dof_pos':
                pos = np.zeros(self._num_actions, dtype=np.float32)
                for ai, jname in enumerate(self._joint_names[:self._num_actions]):
                    if jname not in self._wheel_joints:
                        pos[ai] = (self._joint_pos.get(jname, 0.0)
                                   - float(self._default_angles[ai]))
                self._obs['dof_pos'] = pos * self._dof_pos_scale

            elif n == 'dof_pos_nwp':
                nwp = self._num_actions - len(self._wheel_joints)
                pos = np.zeros(nwp, dtype=np.float32)
                idx = 0
                for ai, jname in enumerate(self._joint_names[:self._num_actions]):
                    if jname not in self._wheel_joints and idx < nwp:
                        pos[idx] = (self._joint_pos.get(jname, 0.0)
                                    - float(self._default_angles[ai]))
                        idx += 1
                self._obs['dof_pos_nwp'] = pos * self._dof_pos_scale

            elif n == 'dof_vel':
                vel = np.zeros(self._num_actions, dtype=np.float32)
                for ai, jname in enumerate(self._joint_names[:self._num_actions]):
                    vel[ai] = self._joint_vel.get(jname, 0.0)
                self._obs['dof_vel'] = vel * self._dof_vel_scale

            elif n == 'last_actions':
                self._obs['last_actions'] = self._last_actions.copy()

            elif n == 'phases':
                t = self._current_time * math.pi / 2.0
                self._obs['phases'] = np.array([
                    math.sin(t),     math.cos(t),
                    math.sin(t/2),   math.cos(t/2),
                    math.sin(t/4),   math.cos(t/4),
                ], dtype=np.float32)

            elif n == 'ref_motion_phase':
                phase_val = (self._current_time / self._episode_length
                             if self._episode_length > 1e-4 else 0.0)
                self._obs['ref_motion_phase'] = np.array([phase_val], dtype=np.float32)

            elif n == 'action_rescale':
                self._obs['action_rescale'] = np.array([0.25], dtype=np.float32)

    # ----------------------------------------------------------------------- #
    #  Build inference inputs  (mirrors update_forward variants)
    # ----------------------------------------------------------------------- #

    def _build_np3o(self):
        """np3o: nn_input0=[1, obs_dim], nn_input1=[1, history_len, obs_dim]."""
        obs = np.concatenate([self._obs[n] for n in self._obs_names])  # (obs_dim,)
        hist = np.stack([
            np.concatenate([self._obs_hist[n][i] for n in self._obs_names])
            for i in range(self._history_len)
        ])  # (history_len, obs_dim)
        return [obs[None], hist[None]]  # (1, obs_dim), (1, history_len, obs_dim)

    def _build_ppo(self):
        """Single input: [obs_hist (per-obs-name, oldest→newest), obs].
        Matches FSMState_RLPPO::update_forward."""
        rows = []
        for n in self._obs_names:
            for i in range(self._history_len):
                rows.append(self._obs_hist[n][i])
        obs_hist = np.concatenate(rows)
        obs = np.concatenate([self._obs[n] for n in self._obs_names])
        combined = np.concatenate([obs_hist, obs])
        return [combined[None]]

    def _build_asap(self):
        """Single input with custom ordering (newest→oldest history).
        Matches FSMState_RLASAP::update_forward."""
        sorted_hist_names = [
            'last_actions', 'ang_vel', 'dof_pos', 'dof_vel', 'gravity', 'ref_motion_phase',
        ]
        rows = []
        for n in sorted_hist_names:
            if n not in self._obs_hist:
                continue
            for i in range(self._history_len - 1, -1, -1):   # newest → oldest
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

    # ----------------------------------------------------------------------- #
    #  Inference
    # ----------------------------------------------------------------------- #

    def _run_inference(self):
        if self._session is None:
            return

        if self._policy_type == 'np3o':
            inputs = self._build_np3o()
        elif self._policy_type == 'ppo':
            inputs = self._build_ppo()
        elif self._policy_type == 'asap':
            inputs = self._build_asap()
        else:
            self.get_logger().error(f'Unknown policy_type: {self._policy_type}', once=True)
            return

        feed = {name: data.astype(np.float32)
                for name, data in zip(self._input_names, inputs)}
        outputs = self._session.run(None, feed)
        self._last_actions = np.array(outputs[0], dtype=np.float32).flatten()

    # ----------------------------------------------------------------------- #
    #  Publish joint command  (mirrors FSMState_RL::run joint command section)
    # ----------------------------------------------------------------------- #

    def _publish_joint_command(self):
        msg = JointControlCommand()
        msg.header.stamp = self.get_clock().now().to_msg()

        for ai, jname in enumerate(self._joint_names):
            action  = float(self._last_actions[ai])
            scale   = float(self._action_scales[ai])
            default = float(self._default_angles[ai])
            kp      = float(self._joint_kp[ai])
            kd      = float(self._joint_kd[ai])

            msg.name.append(jname)

            if jname in self._wheel_joints:
                if self._control_type == 'P':
                    msg.position.append(0.0)
                    msg.velocity.append(0.0)
                    msg.effort.append((action * scale + default) * kp
                                      - kd * self._joint_vel.get(jname, 0.0))
                    msg.kp.append(0.0)
                    msg.kd.append(0.0)
                else:  # P_V
                    msg.position.append(0.0)
                    msg.velocity.append(action * scale)
                    msg.effort.append(0.0)
                    msg.kp.append(0.0)
                    msg.kd.append(kd)
            else:
                msg.position.append(action * scale + default)
                msg.velocity.append(0.0)
                msg.effort.append(0.0)
                msg.kp.append(kp)
                msg.kd.append(kd)

        self._joint_cmd_pub.publish(msg)

    # ----------------------------------------------------------------------- #
    #  Control loop
    # ----------------------------------------------------------------------- #

    def _control_loop(self, stamp: float):
        # initialise on first message
        if self._start_stamp is None:
            self._start_stamp = stamp

        self._current_time = stamp - self._start_stamp

        # Time-gated inference: run the policy once per training control period
        # (self._control_dt), measured on the message stamp (physics time), so the
        # rate stays at the trained 50 Hz no matter what rate joint_states arrives
        # at.  The first message always infers so there is a valid action to send.
        if self._last_infer_stamp is None or \
                (stamp - self._last_infer_stamp) >= self._control_dt - 1e-6:
            if self._last_infer_stamp is not None:
                self._infer_dt = stamp - self._last_infer_stamp
            self._last_infer_stamp = stamp
            self._update_observations()
            self._run_inference()

            # debug log at ~2 Hz (every 0.5 s)
            if self._iter % 200 == 0:
                actions = self._last_actions * self._action_scales + self._default_angles
                self.get_logger().debug(f'actions = {np.round(actions, 4).tolist()}')

        self._publish_joint_command()
        self._iter += 1


# --------------------------------------------------------------------------- #

def main():
    rclpy.init()
    node = RLInferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
