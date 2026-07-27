#!/usr/bin/env python3
"""Joystick axis mapping — the pure-math half of the raw-Joy velocity source,
kept out of the rl_inference transport shell (same split as policy_engine.py).

rl_inference subscribes sensor_msgs/Joy and calls JoyMapping.to_command(axes)
to get the (twist_linear, twist_angular, pose_rpy) it would otherwise receive on
cmd_twist / cmd_pose.  No ROS, no state — just the axis arithmetic, so it is
trivially testable and swappable.  Mirrors ddt_ros2_control teleop_command_node.

Axis map (defaults; override with a yaml `joy:` block):
    axes[2]  vx        -axes * 3.0 * speed     forward/back   (always live)
    axes[3]  wz         axes * 6.0 * speed     yaw            (always live)
    axes[0]  vy | roll  strafe (gate==0) | body roll  (gate==-1)
    axes[1]  vz | pitch height-rate (gate==0) | body pitch (gate==-1)
    axes[7]  speed      3-pos gear -> [0.33, 0.66, 1.0]
    axes[5]  gate       0 -> strafe/height, -1 -> body pose  (<0 -> always strafe)

    axis index <0 disables that channel; scale is signed (carries direction).
"""

from dataclasses import dataclass


@dataclass
class JoyMapping:
    ax_vx: int = 2
    ax_vy: int = 0
    ax_vz: int = 1
    ax_wz: int = 3
    ax_speed: int = 7        # 3-pos speed selector; <0 -> no gearing
    ax_gate: int = 5         # 0 -> strafe/height, -1 -> body pose; <0 -> always strafe
    sc_vx: float = -3.0      # m/s   (-axes * max_twist_linear)
    sc_vy: float = 1.0       # m/s
    sc_vz: float = -1.0      # base-height rate
    sc_wz: float = 6.0       # rad/s (axes * max_twist_angular)
    sc_roll: float = -0.2    # rad
    sc_pitch: float = -0.4   # rad
    speed_levels: tuple = (0.33, 0.66, 1.0)

    @classmethod
    def from_dict(cls, d):
        """Build from a yaml `joy:` block; missing keys -> default, unknown ignored."""
        if not isinstance(d, dict):
            return cls()
        keys = set(cls.__dataclass_fields__)
        return cls(**{k: v for k, v in d.items() if k in keys})

    def to_command(self, axes):
        """axes (sequence of float) -> (twist_linear[3], twist_angular[3], pose_rpy[3])."""
        n = len(axes)
        def a(i):
            return float(axes[i]) if 0 <= i < n else 0.0
        ratio = 1.0
        if self.ax_speed >= 0 and self.speed_levels:
            idx = max(0, min(len(self.speed_levels) - 1,
                             int(round(1.0 - a(self.ax_speed)))))
            ratio = self.speed_levels[idx]
        vx = a(self.ax_vx) * self.sc_vx * ratio
        wz = a(self.ax_wz) * self.sc_wz * ratio
        gate = int(round(a(self.ax_gate))) if self.ax_gate >= 0 else 0
        vy = vz = roll = pitch = 0.0
        if gate == 0:
            vy = a(self.ax_vy) * self.sc_vy * ratio
            vz = a(self.ax_vz) * self.sc_vz * ratio
        elif gate == -1:
            roll = a(self.ax_vy) * self.sc_roll
            pitch = a(self.ax_vz) * self.sc_pitch
        return [vx, vy, vz], [0.0, 0.0, wz], [roll, pitch, 0.0]
