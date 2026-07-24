#!/usr/bin/env python3
"""
MuJoCo sim2sim backend — no ros2_control.

Speaks the sim/BACKEND.md contract so rl_inference.py runs unmodified:
    sub  command/joint_command       (ddt_msgs/JointControlCommand)
    pub  joint_states                (sensor_msgs/JointState)
    pub  imu_sensor_broadcaster/imu  (sensor_msgs/Imu)

Shared logic (hold pose/gains, PD, DDS) is in sim/simbase.py; this file is
the MuJoCo binding + wall-clock step loop. Physics is paced by sleep and
never blocked by rendering (passive viewer, sync every few steps). PD is
recomputed every physics step from the latest cached command.
"""

import argparse
import os
import sys
import threading
import time

import numpy as np
import mujoco
import mujoco.viewer
import rclpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from simbase import SimBackend, spec   # noqa: E402

# Scene is derived from --robot so `--robot <r>` alone loads its MJCF; override
# with --scene.  Resolved relative to this file (repo is self-contained).
def _scene_for(robot):
    return os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        '..', 'models', f'{robot}_description', 'mujoco', 'scene.xml')

INIT_BASE_HEIGHT = 0.35   # free-joint spawn height; allow it to settle onto the wheels


class MujocoSimLite(SimBackend):
    """MuJoCo binding for SimBackend: build handles, step physics, publish state."""

    def __init__(self, scene_path, spec):
        super().__init__('mujoco_sim_lite', spec, imu_frame='trunk_imu')
        self.model = mujoco.MjModel.from_xml_path(scene_path)
        self.data = mujoco.MjData(self.model)
        self.dt = self.model.opt.timestep

        # Actuated joints in actuator order; must equal the shared joint order
        # (= command/state vector order) so ctrl[:] aligns with compute_torque.
        model_joints, qpos_adr, qvel_adr = [], [], []
        for i in range(self.model.nu):
            jid = self.model.actuator_trnid[i][0]
            model_joints.append(
                mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, jid))
            qpos_adr.append(self.model.jnt_qposadr[jid])
            qvel_adr.append(self.model.jnt_dofadr[jid])
        assert model_joints == spec.joint_names, (
            f'MJCF actuator order {model_joints} != spec order '
            f'{spec.joint_names}; fix the scene or the RobotSpec')
        self.qpos_adr = np.array(qpos_adr)
        self.qvel_adr = np.array(qvel_adr)

        # Spawn in the resting pose slightly above ground and settle.
        self.data.qpos[2] = INIT_BASE_HEIGHT
        self.data.qpos[self.qpos_adr] = spec.hold_pose
        mujoco.mj_forward(self.model, self.data)

        # Named IMU sensor views (framequat is w,x,y,z; gyro/accel body-frame).
        self.sens_quat = self.data.sensor('trunk_quat')
        self.sens_gyro = self.data.sensor('trunk_gyro')
        self.sens_acc = self.data.sensor('trunk_accel')

        self.get_logger().info(
            f'mujoco_sim_lite ready | physics dt={self.dt:.4f}s'
            f' ({1.0 / self.dt:.0f}Hz) | {self.model.nu} joints'
            f' | scene={scene_path}')

    # ------------------------------------------------------------------ #

    def _apply_pd(self):
        q = self.data.qpos[self.qpos_adr]
        v = self.data.qvel[self.qvel_adr]
        self.data.ctrl[:] = self.compute_torque(q, v)

    def _publish(self):
        q = self.data.qpos[self.qpos_adr]
        v = self.data.qvel[self.qvel_adr]
        self.publish_state(q, v, self.sens_quat.data,   # framequat = w,x,y,z
                           self.sens_gyro.data, self.sens_acc.data,
                           effort=self.data.ctrl)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--robot', default='d1',
                    help='needs config/<robot>/ + models/<robot>_description/')
    ap.add_argument('--scene', default=None,
                    help='MJCF scene (default: models/<robot>_description/mujoco/scene.xml)')
    ap.add_argument('--no-viewer', action='store_true',
                    help='run headless (no GUI window)')
    ap.add_argument('--pub-every', type=int, default=2,
                    help='publish every N physics steps (2 -> 250Hz at dt=0.002)')
    ap.add_argument('--log-period', type=float, default=5.0,
                    help='seconds between debug pose log lines')
    args, ros_args = ap.parse_known_args()

    rclpy.init(args=ros_args)
    node = MujocoSimLite(args.scene or _scene_for(args.robot), spec(args.robot))

    # Callbacks (command cache) run on a separate spin thread; the main
    # thread is the wall-clock-paced physics loop.
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    viewer = None
    if not args.no_viewer:
        viewer = mujoco.viewer.launch_passive(node.model, node.data)

    sync_every = max(1, int(round(1.0 / 60.0 / node.dt)))   # ~60Hz visual
    log_every = max(1, int(round(args.log_period / node.dt)))
    step = 0
    next_t = time.perf_counter()
    try:
        while rclpy.ok():
            if viewer is not None and not viewer.is_running():
                break

            node._apply_pd()
            mujoco.mj_step(node.model, node.data)
            step += 1

            if step % args.pub_every == 0:
                node._publish()
            if viewer is not None and step % sync_every == 0:
                viewer.sync()
            if step % log_every == 0:
                w, x, y, z = node.sens_quat.data
                roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
                pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
                q1 = node.data.qpos[node.qpos_adr[1]]
                node.get_logger().info(
                    f'z={node.data.qpos[2]:.3f}'
                    f' roll={np.degrees(roll):+.1f}deg'
                    f' pitch={np.degrees(pitch):+.1f}deg'
                    f' cmd={"Y" if node.have_cmd else "N"}'
                    f' FL_thigh cmd={node.pos_cmd[1]:+.2f} q={q1:+.2f}'
                    f' kp={node.kp[1]:.0f}')

            # Wall-clock pacing with drift correction (reference: pybullet
            # simulator's sleep(dt - cost)).  If the loop falls badly behind, resync
            # instead of bursting to catch up.
            next_t += node.dt
            delay = next_t - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            elif delay < -0.1:
                next_t = time.perf_counter()
    except KeyboardInterrupt:
        pass
    finally:
        if viewer is not None:
            viewer.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
