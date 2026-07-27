#!/usr/bin/env python3
"""Transport shell: ROS 2 topics in, joint_command out.

Snapshots joint_states / imu / cmd_twist / cmd_pose into RobotState/Command
and hands them to PolicyEngine (policy_engine.py), which does obs assembly +
onnx + decode. Velocity may instead come straight from a raw sensor_msgs/Joy
topic (joy_command), mapped by joy_mapping.py like ddt_ros2_control's
teleop_command_node. This node knows only ROS 2; the math lives in the modules.
"""

import math
import os
import signal
import time
import yaml

from ament_index_python.packages import get_package_share_directory

import numpy as np
import rclpy
from ddt_msgs.msg import JointControlCommand
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu, JointState, Joy

from joy_mapping import JoyMapping
from policy_engine import (
    Command,
    JointCommand,
    PolicyConfig,
    PolicyEngine,
    RobotState,
    euler_from_quat,
)


# --------------------------------------------------------------------------- #
#  Node
# --------------------------------------------------------------------------- #

class RLInferenceNode(Node):
    """ROS 2 node: sensor topics in, joint_command out; rl / damping mode on two wall-clock timers."""

    def __init__(self):
        super().__init__('rl_inference_node')
        self._declare_params()
        self._load_params()
        self._engine = PolicyEngine(self._build_config(), logger=self.get_logger())
        self._init_caches()
        robot_ns = os.environ.get('ROBOT_NS', '').strip('/')
        self._tp = (lambda t: f'{robot_ns}/{t}') if robot_ns else (lambda t: t)
        self._create_subscriptions()
        self._joint_cmd_pub = self.create_publisher(
            JointControlCommand, self._tp('command/joint_command'), 10)
        # two wall-clock timers: infer at control_dt(50Hz), publish at 200 Hz
        self._infer_timer = self.create_timer(self._control_dt, self._infer_cb)
        self._publish_timer = self.create_timer(0.005, self._publish_cb)
        # one-shot startup check: warn if no state source instead of idling
        self._nostate_timer = self.create_timer(3.0, self._nostate_check)
        self.get_logger().info(
            f'RLInferenceNode ready | policy={self._policy_type}'
            f' num_actions={self._num_actions} history_len={self._history_len}'
            f' control_dt={self._control_dt:.4f}s ({1.0 / self._control_dt:.1f}Hz infer)'
            f' loop=wall-clock infer@{1.0 / self._control_dt:.0f}Hz+publish@200Hz'
            f' ns={robot_ns or "(none)"}'
        )

    # ----------------------------------------------------------------------- #
    #  Parameter declaration / loading
    # ----------------------------------------------------------------------- #

    def _declare_params(self):
        self.declare_parameter('config_file', '')   
        self.declare_parameter('policy_name', '')  
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
        self.declare_parameter('joy_command', True)   # velocity source = raw sensor_msgs/Joy
        self.declare_parameter('joy_topic', 'joy')    # axis map/scales from a yaml `joy:` block
        # debug only: disables posture guard + staleness watchdog.
        self.declare_parameter('debug_disable_guards', False)

    def _load_yaml_section(self, config_file: str, policy_name: str):
        """Return (section_dict, full_yaml_dict)."""
        with open(config_file, 'r') as f:
            full = yaml.safe_load(f)
        if not policy_name:
            return (full if isinstance(full, dict) else {}), full
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
                # fail fast with the valid names; do not idle on ROS defaults
                self.get_logger().error(f'Failed to load config: {e}')
                try:
                    with open(config_file) as f:
                        names = self._find_key_recursive(
                            yaml.safe_load(f), 'rl_policy_names') or []
                except Exception:
                    names = []
                if names:
                    self.get_logger().error('available policies: ' + ', '.join(names))
                raise SystemExit(1)

        def y(yaml_key, ros_param):
            """YAML value takes precedence over the ROS2 param default."""
            return cfg[yaml_key] if yaml_key in cfg else g(ros_param).value

        # policy_path: prefer an onnx next to the config file (self-contained);
        # fall back to the rl_controller share dir for the legacy in-tree layout
        policy_path = str(cfg.get('policy_path', ''))
        if policy_path and not os.path.isabs(policy_path):
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
        self._cmd_min     = np.array(y('min_commands',   'min_commands'),   dtype=np.float32)
        self._cmd_max     = np.array(y('max_commands',   'max_commands'),   dtype=np.float32)
        # commands_gain is an optional per-command multiplier (default 1.0 each).
        _gain = np.array(y('commands_gain', 'commands_gain'), dtype=np.float32)
        self._cmd_gain = np.ones(len(self._cmd_names), dtype=np.float32)
        self._cmd_gain[:len(_gain)] = _gain
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

        self._control_dt = float(cfg.get('control_dt', 0.02))
        self._episode_length = float(y('episode_length', 'episode_length'))

        self._joy_command = bool(g('joy_command').value)
        self._joy_topic = str(g('joy_topic').value)
        self._joymap = JoyMapping.from_dict(self._find_key_recursive(full_yaml, 'joy'))

        self._no_guards = bool(g('debug_disable_guards').value)
        if self._no_guards:
            self.get_logger().warning(
                '!! debug_disable_guards=true — posture guard + staleness '
                'watchdog OFF; visualization only, never on real hardware !!')

    def _build_config(self) -> PolicyConfig:
        """Pack the loaded ROS params into the transport-free engine config."""
        return PolicyConfig(
            onnx_path=self._onnx_path,
            policy_type=self._policy_type,
            output_name=self._output_name,
            num_actions=self._num_actions,
            history_len=self._history_len,
            obs_names=self._obs_names,
            cmd_names=self._cmd_names,
            cmd_scale=self._cmd_scale,
            cmd_gain=self._cmd_gain,
            cmd_min=self._cmd_min,
            cmd_max=self._cmd_max,
            ang_vel_scale=self._ang_vel_scale,
            dof_pos_scale=self._dof_pos_scale,
            dof_vel_scale=self._dof_vel_scale,
            wheel_joints=self._wheel_joints,
            default_angles=self._default_angles,
            action_scales=self._action_scales,
            joint_kp=self._joint_kp,
            joint_kd=self._joint_kd,
            control_type=self._control_type,
            joint_names=self._joint_names,
            episode_length=self._episode_length,
            control_dt=self._control_dt,
        )

    # ----------------------------------------------------------------------- #
    #  Sensor caches (filled by callbacks, snapshotted by the timer)
    # ----------------------------------------------------------------------- #

    def _init_caches(self):
        self._gyro      = np.zeros(3, dtype=np.float32)
        self._quat_wxyz = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
        self._joint_pos: dict[str, float] = {}
        self._joint_vel: dict[str, float] = {}
        self._twist_linear  = np.zeros(3, dtype=np.float32)
        self._twist_angular = np.zeros(3, dtype=np.float32)
        self._pose_rpy      = np.zeros(3, dtype=np.float32)
        self._state_stamp: float | None = None   # stamp of latest cached joint_states
        self._state_walltime: float | None = None  # wall-clock receipt time (watchdog)
        self._inferred_once = False               # gate publishing until first inference
        self._iter = 0
        # mode: rl -> damping (fault/exit). plain string, no FSM classes.
        self._mode = 'rl'

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
        if self._joy_command:
            self.create_subscription(Joy, self._tp(self._joy_topic), self._joy_cb, qos)


    def _imu_cb(self, msg: Imu):
        o = msg.orientation
        self._quat_wxyz = np.array([o.w, o.x, o.y, o.z], dtype=np.float32)
        v = msg.angular_velocity
        self._gyro = np.array([v.x, v.y, v.z], dtype=np.float32)

    def _joint_cb(self, msg: JointState):
        for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
            self._joint_pos[name] = float(pos)
            self._joint_vel[name] = float(vel)
        self._state_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._state_walltime = time.monotonic()   # feeds the staleness watchdog

    def _twist_cb(self, msg: Twist):
        self._twist_linear  = np.array([msg.linear.x,  msg.linear.y,  msg.linear.z],  dtype=np.float32)
        self._twist_angular = np.array([msg.angular.x, msg.angular.y, msg.angular.z], dtype=np.float32)

    def _pose_cb(self, msg: PoseStamped):
        q = msg.pose.orientation
        r, p, y = euler_from_quat(q.w, q.x, q.y, q.z)
        self._pose_rpy = np.array([r, p, y], dtype=np.float32)

    def _joy_cb(self, msg: Joy):
        tl, ta, pr = self._joymap.to_command(msg.axes)
        self._twist_linear  = np.array(tl, dtype=np.float32)
        self._twist_angular = np.array(ta, dtype=np.float32)
        self._pose_rpy      = np.array(pr, dtype=np.float32)

    # ----------------------------------------------------------------------- #
    #  Control tick: snapshot caches → engine → publish
    # ----------------------------------------------------------------------- #

    def _snapshot(self):
        """Snapshot the caches. Single-threaded executor -> no lock needed."""
        state = RobotState(
            stamp=self._state_stamp,
            gyro=self._gyro,
            quat_wxyz=self._quat_wxyz,
            joint_pos=self._joint_pos,
            joint_vel=self._joint_vel,
        )
        cmd = Command(
            twist_linear=self._twist_linear,
            twist_angular=self._twist_angular,
            pose_rpy=self._pose_rpy,
        )
        return state, cmd

    def _nostate_check(self):
        self._nostate_timer.cancel()      # one-shot
        if self._state_stamp is None:
            self.get_logger().warning(
                'no joint_states after 3s — verify the sim/robot is online; start '
                './scripts/run_sim.sh (sim2sim), or check '
                'ROS_LOCALHOST_ONLY / ROS_DOMAIN_ID / ROBOT_NS')

    def _infer_cb(self):
        if self._mode != 'rl' or self._state_stamp is None:
            return
        # posture guard (Lite3 thresholds: roll 30deg pitch 45deg) -> damping
        w, x, y, z = self._quat_wxyz
        roll, pitch, _ = euler_from_quat(w, x, y, z)
        if not self._no_guards and \
                (abs(roll) > math.radians(30.0) or abs(pitch) > math.radians(45.0)):
            self._mode = 'damping'
            self.get_logger().error(
                f'posture unsafe (roll={math.degrees(roll):.1f} '
                f'pitch={math.degrees(pitch):.1f} deg) -> damping')
            return
        state, cmd = self._snapshot()
        self._engine.infer(state, cmd)
        self._inferred_once = True

        if self._iter % 100 == 0:
            actions = (self._engine.last_actions * self._action_scales
                       + self._default_angles)
            self.get_logger().debug(f'actions = {np.round(actions, 4).tolist()}')
        self._iter += 1

    def _publish_cb(self):
        # 200 Hz: re-decode with live velocity so wheel damping stays current
        if self._state_stamp is None:
            return
        # staleness watchdog
        if not self._no_guards and self._mode != 'damping' and \
                self._state_walltime is not None and \
                (time.monotonic() - self._state_walltime) > 0.2:
            self._mode = 'damping'
            self.get_logger().error(
                'joint_states stalled >0.2s -> damping (restart this node to recover)')
        if self._mode == 'rl':
            if not self._inferred_once:
                return
            state, _ = self._snapshot()
            self._publish(self._engine.decode(state))
        else:  # damping
            self._publish(self._damping_command())

    # ----------------------------------------------------------------------- #
    #  Damping command (mirror FSMState_Passive)
    # ----------------------------------------------------------------------- #

    def _damping_command(self):
        # legs kd=5, wheels kd=0
        jc = JointCommand()
        for jname in self._joint_names:
            jc.name.append(jname)
            jc.position.append(0.0)
            jc.velocity.append(0.0)
            jc.effort.append(0.0)
            jc.kp.append(0.0)
            jc.kd.append(0.0 if jname in self._wheel_joints else 5.0)
        return jc

    def send_damping_burst(self, n=20, period=0.02):
        # on shutdown
        for _ in range(n):
            self._publish(self._damping_command())
            time.sleep(period)

    def _publish(self, jc):
        msg = JointControlCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(jc.name)
        msg.position = [float(v) for v in jc.position]
        msg.velocity = [float(v) for v in jc.velocity]
        msg.effort = [float(v) for v in jc.effort]
        msg.kp = [float(v) for v in jc.kp]
        msg.kd = [float(v) for v in jc.kd]
        self._joint_cmd_pub.publish(msg)


# --------------------------------------------------------------------------- #

def main():
    rclpy.init()
    node = RLInferenceNode()
    def _sig(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.send_damping_burst()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
