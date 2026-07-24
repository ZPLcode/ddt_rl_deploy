#!/usr/bin/env python3
"""Webots <extern> controller — sim2sim backend, no webots_ros2/ros2_control.

Connects to a running Webots and speaks the sim/BACKEND.md contract, same
topics as mujoco_sim.py. Shared logic is in sim/simbase.py; this file is the
Webots binding only. Torque via Motor.setTorque(); Webots runs in real-time
mode so physics advances at wall-clock.
"""

import os
import sys

import numpy as np
import rclpy

from controller import Robot   # Webots controller API

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from simbase import SimBackend, spec   # noqa: E402


class WebotsBackend(SimBackend):
    """Webots binding for SimBackend: torque-mode motors, position sensors, IMU."""

    def __init__(self, spec):
        super().__init__('webots_sim', spec, imu_frame='imu_link')
        self.robot = Robot()
        self.dt_ms = int(self.robot.getBasicTimeStep())
        self.dt = self.dt_ms * 1e-3
        n = len(self.joint_names)

        # motors (torque mode) + position sensors, in shared joint order
        self.motors, self.psensors = [], []
        for j in self.joint_names:
            m = self.robot.getDevice(j)
            m.setTorque(0.0)                    # enter torque-control mode
            self.motors.append(m)
            s = self.robot.getDevice(j + '_sensor')
            s.enable(self.dt_ms)
            self.psensors.append(s)
        self.imu_quat = self.robot.getDevice('imu_quat'); self.imu_quat.enable(self.dt_ms)
        self.imu_gyro = self.robot.getDevice('imu_gyro'); self.imu_gyro.enable(self.dt_ms)
        self.imu_acc = self.robot.getDevice('imu_acc');   self.imu_acc.enable(self.dt_ms)

        # no joint-velocity sensor -> differentiate position; seed at hold pose
        self.q = spec.hold_pose.copy()
        self.q_prev = spec.hold_pose.copy()
        self.dq = np.zeros(n)

        self.get_logger().info(
            f'webots_sim ready | dt={self.dt:.4f}s ({1 / self.dt:.0f}Hz)'
            f' | {n} joints')

    def _read_state(self):
        self.q = np.array([s.getValue() for s in self.psensors])
        self.dq = (self.q - self.q_prev) / self.dt
        self.q_prev = self.q.copy()

    def _apply_pd(self):
        tau = self.compute_torque(self.q, self.dq)
        for m, t in zip(self.motors, tau):
            m.setTorque(float(t))

    def _publish(self):
        qx, qy, qz, qw = self.imu_quat.getQuaternion()     # Webots returns x,y,z,w
        self.publish_state(self.q, self.dq, (qw, qx, qy, qz),
                           self.imu_gyro.getValues(), self.imu_acc.getValues())

    def run(self):
        while self.robot.step(self.dt_ms) != -1:
            rclpy.spin_once(self, timeout_sec=0.0)   # drain incoming commands
            self._read_state()
            self._publish()
            self._apply_pd()


def main():
    robot = 'd1'
    if '--robot' in sys.argv:
        robot = sys.argv[sys.argv.index('--robot') + 1]

    rclpy.init(args=None)
    node = WebotsBackend(spec(robot))
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
