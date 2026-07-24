#!/usr/bin/env python3
"""Shared core for sim2sim backends.

Backends subclass SimBackend and implement only their engine bindings (read
q/dq, apply torque, read IMU, drive the step loop). Joint order, stand pose,
hold gains, the MIT-PD law, command matching and the DDS contract live here
in one copy.

Per-robot data comes from config/<robot>/controllers.yaml via spec(robot) —
adding a robot needs no code change (see README).
"""

import os

import numpy as np
import yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState, Imu
from ddt_msgs.msg import JointControlCommand

CONFIG_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config')


def _find(node, key):
    """Depth-first search for the first value under ``key`` in a nested dict."""
    if isinstance(node, dict):
        if key in node:
            return node[key]
        for v in node.values():
            found = _find(v, key)
            if found is not None:
                return found
    return None


class RobotSpec:
    """Per-robot constants, loaded from config. joint_names order =
    controllers.yaml joints = MJCF actuator order. Wheels get velocity-only
    hold (kp=0), legs a stiff pose hold at stand_jpos."""

    def __init__(self, joint_names, hold_pose, wheel_indices=None,
                 leg_kp=60.0, leg_kd=2.0, wheel_kp=0.0, wheel_kd=1.0):
        assert len(joint_names) == len(hold_pose), (
            f'joints ({len(joint_names)}) != hold pose ({len(hold_pose)})')
        self.joint_names = list(joint_names)
        if wheel_indices is None:
            # fallback heuristic (d1-style names); configs should set wheel_indices
            self.is_wheel = np.array(['foot' in j for j in joint_names])
        else:
            self.is_wheel = np.zeros(len(joint_names), dtype=bool)
            self.is_wheel[list(wheel_indices)] = True
        self.hold_pose = np.asarray(hold_pose, dtype=float)
        self.hold_kp = np.where(self.is_wheel, wheel_kp, leg_kp)
        self.hold_kd = np.where(self.is_wheel, wheel_kd, leg_kd)

    @classmethod
    def from_config(cls, robot, config_dir=CONFIG_DIR, **gains):
        """Read joints / transform_up.stand_jpos / wheel_indices from
        <config_dir>/<robot>/controllers.yaml."""
        path = os.path.join(config_dir, robot, 'controllers.yaml')
        if not os.path.isfile(path):
            raise FileNotFoundError(
                f'no config for robot {robot!r}: {path} not found. To add it: '
                f'`cp -r config/_template config/{robot}` and fill it in '
                f'(see README)')
        cfg = yaml.safe_load(open(path))
        joints = _find(cfg, 'joints')
        hold_pose = _find(cfg, 'stand_jpos')
        if not joints or hold_pose is None:
            raise ValueError(
                f'{path}: need a `joints` list and a `transform_up.stand_jpos`')
        return cls(joints, hold_pose,
                   wheel_indices=_find(cfg, 'wheel_indices'), **gains)


def spec(robot):
    """RobotSpec for ``robot``, loaded from its config."""
    return RobotSpec.from_config(robot)


class SimBackend(Node):
    """Base node: command cache, hold/track MIT-PD, joint_states/imu pub +
    command sub with the topic-contract QoS. Subclasses bind their engine:
    build handles from spec.joint_names, apply compute_torque(q, dq) each
    step, publish via publish_state()."""

    def __init__(self, node_name, spec, imu_frame):
        super().__init__(node_name)
        self.spec = spec
        self.joint_names = spec.joint_names
        self._imu_frame = imu_frame
        self._name_to_idx = {j: i for i, j in enumerate(self.joint_names)}

        n = len(self.joint_names)
        self.kp = np.zeros(n)
        self.kd = np.zeros(n)
        self.pos_cmd = np.zeros(n)
        self.vel_cmd = np.zeros(n)
        self.tau_ff = np.zeros(n)
        self.have_cmd = False

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self._js_pub = self.create_publisher(JointState, 'joint_states', qos)
        self._imu_pub = self.create_publisher(
            Imu, 'imu_sensor_broadcaster/imu', qos)
        self.create_subscription(
            JointControlCommand, 'command/joint_command', self._cmd_cb, 10)

    # -- command cache: matched by joint name, zero-order held until replaced --
    def _cmd_cb(self, msg):
        self.have_cmd = True
        for k, name in enumerate(msg.name):
            i = self._name_to_idx.get(name)
            if i is None:
                continue
            if k < len(msg.kp):
                self.kp[i] = msg.kp[k]
            if k < len(msg.kd):
                self.kd[i] = msg.kd[k]
            if k < len(msg.position):
                self.pos_cmd[i] = msg.position[k]
            if k < len(msg.velocity):
                self.vel_cmd[i] = msg.velocity[k]
            if k < len(msg.effort):
                self.tau_ff[i] = msg.effort[k]

    # -- MIT PD: hold the resting pose until the first command, then track it --
    def compute_torque(self, q, dq):
        q = np.asarray(q)
        dq = np.asarray(dq)
        if not self.have_cmd:
            return self.spec.hold_kp * (self.spec.hold_pose - q) \
                - self.spec.hold_kd * dq
        return (self.kp * (self.pos_cmd - q)
                + self.kd * (self.vel_cmd - dq) + self.tau_ff)

    # -- publish the state contract; quat is (w,x,y,z) ------------------------
    def publish_state(self, q, dq, quat_wxyz, gyro, acc, effort=None):
        now = self.get_clock().now().to_msg()

        js = JointState()
        js.header.stamp = now
        js.name = self.joint_names
        js.position = [float(v) for v in q]
        js.velocity = [float(v) for v in dq]
        if effort is not None:
            js.effort = [float(v) for v in effort]
        self._js_pub.publish(js)

        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = self._imu_frame
        w, x, y, z = quat_wxyz
        imu.orientation.w = float(w)
        imu.orientation.x = float(x)
        imu.orientation.y = float(y)
        imu.orientation.z = float(z)
        imu.angular_velocity.x = float(gyro[0])
        imu.angular_velocity.y = float(gyro[1])
        imu.angular_velocity.z = float(gyro[2])
        imu.linear_acceleration.x = float(acc[0])
        imu.linear_acceleration.y = float(acc[1])
        imu.linear_acceleration.z = float(acc[2])
        self._imu_pub.publish(imu)
