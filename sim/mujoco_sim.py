#!/usr/bin/env python3
"""
Lightweight MuJoCo simulator for DDT deployment testing — no ros2_control.

Speaks exactly the topics the deploy stack uses, so rl_inference.py runs
against it unmodified:
    subscribes  command/joint_command       (ddt_msgs/JointControlCommand)
    publishes   joint_states                (sensor_msgs/JointState)
    publishes   imu_sensor_broadcaster/imu  (sensor_msgs/Imu)

Loop architecture follows the DeepRobotics reference simulators
(Lite3_rl_deploy/mujoco_simulation.py, sdk_deploy/mujoco_simulation_ros2.py):
  * physics + publish are paced by the wall clock (sleep-based) and are never
    blocked by rendering: the viewer is mujoco.viewer.launch_passive (GUI runs
    in its own thread) and the loop only pushes state via viewer.sync() every
    few steps — it never waits on vsync.  This keeps joint_states regular,
    unlike the ros2_control mujoco bridge whose single loop couples physics,
    publishing and a vsync-blocking 60 fps render.
  * the MIT PD law  tau = kp*(p*-p) + kd*(v*-v) + tau_ff  is recomputed every
    physics step from the latest cached command (zero-order hold), so command
    arrival cadence never affects torque smoothness.
"""

import argparse
import threading
import time

import numpy as np
import mujoco
import mujoco.viewer
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState, Imu
from ddt_msgs.msg import JointControlCommand

DEFAULT_SCENE = ('/home/zhepeng/DDT/DDT_lab/ddt_ros2_control/urdfs/'
                 'd1_description/mujoco/scene_cargo_out.xml')

# Initial squat pose (rl_flat default_joint_angles).  Order is the MJCF
# actuator order, which equals the controllers.yaml joint order:
# FL, FR, RL, RR x (hip, thigh, calf, foot).
INIT_JOINT_POS = [0.1, 0.8, -1.5, 0.0,
                  -0.1, 0.8, -1.5, 0.0,
                  0.1, 1.0, -1.5, 0.0,
                  -0.1, 1.0, -1.5, 0.0]
INIT_BASE_HEIGHT = 0.35


class MujocoSimLite(Node):

    def __init__(self, scene_path: str):
        super().__init__('mujoco_sim_lite')
        self.model = mujoco.MjModel.from_xml_path(scene_path)
        self.data = mujoco.MjData(self.model)
        self.dt = self.model.opt.timestep

        # Actuated joints in actuator order (= command/state vector order).
        self.joint_names = []
        qpos_adr, qvel_adr = [], []
        for i in range(self.model.nu):
            jid = self.model.actuator_trnid[i][0]
            self.joint_names.append(
                mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, jid))
            qpos_adr.append(self.model.jnt_qposadr[jid])
            qvel_adr.append(self.model.jnt_dofadr[jid])
        self.qpos_adr = np.array(qpos_adr)
        self.qvel_adr = np.array(qvel_adr)
        self.name_to_idx = {n: i for i, n in enumerate(self.joint_names)}

        n = self.model.nu
        self.kp = np.zeros(n)
        self.kd = np.zeros(n)
        self.pos_cmd = np.zeros(n)
        self.vel_cmd = np.zeros(n)
        self.tau_ff = np.zeros(n)

        # Until the first command arrives, PD-hold the initial squat so the
        # robot is standing when a policy connects (the ros2_control sim gave
        # the same starting condition).  Legs get a stiff hold, wheels are
        # velocity-damped only.
        self.have_cmd = False
        is_wheel = np.array(['foot' in nme for nme in self.joint_names])
        self.hold_kp = np.where(is_wheel, 0.0, 60.0)
        self.hold_kd = np.where(is_wheel, 1.0, 2.0)
        self.hold_pos = np.array(INIT_JOINT_POS)

        # Start slightly above ground in the squat pose and let it settle.
        self.data.qpos[2] = INIT_BASE_HEIGHT
        self.data.qpos[self.qpos_adr] = INIT_JOINT_POS
        mujoco.mj_forward(self.model, self.data)

        # Named IMU sensor views (framequat is w,x,y,z; gyro/accel body-frame).
        self.sens_quat = self.data.sensor('trunk_quat')
        self.sens_gyro = self.data.sensor('trunk_gyro')
        self.sens_acc = self.data.sensor('trunk_accel')

        # Same QoS the deploy stack uses for state topics.
        qos_be = QoSProfile(depth=1,
                            reliability=ReliabilityPolicy.BEST_EFFORT,
                            durability=DurabilityPolicy.VOLATILE)
        self.js_pub = self.create_publisher(JointState, 'joint_states', qos_be)
        self.imu_pub = self.create_publisher(
            Imu, 'imu_sensor_broadcaster/imu', qos_be)
        self.create_subscription(
            JointControlCommand, 'command/joint_command', self._cmd_cb, 10)

        self.get_logger().info(
            f'mujoco_sim_lite ready | physics dt={self.dt:.4f}s'
            f' ({1.0 / self.dt:.0f}Hz) | {n} joints | scene={scene_path}')

    # ------------------------------------------------------------------ #

    def _cmd_cb(self, msg: JointControlCommand):
        # Cache-only, matched by joint name; consumed by _apply_pd each step.
        self.have_cmd = True
        for k, name in enumerate(msg.name):
            i = self.name_to_idx.get(name)
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

    def _apply_pd(self):
        q = self.data.qpos[self.qpos_adr]
        v = self.data.qvel[self.qvel_adr]
        if not self.have_cmd:
            self.data.ctrl[:] = self.hold_kp * (self.hold_pos - q) - self.hold_kd * v
            return
        self.data.ctrl[:] = (self.kp * (self.pos_cmd - q)
                             + self.kd * (self.vel_cmd - v) + self.tau_ff)

    def _publish(self):
        now = self.get_clock().now().to_msg()

        js = JointState()
        js.header.stamp = now
        js.name = self.joint_names
        js.position = self.data.qpos[self.qpos_adr].tolist()
        js.velocity = self.data.qvel[self.qvel_adr].tolist()
        js.effort = self.data.ctrl.tolist()
        self.js_pub.publish(js)

        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = 'trunk_imu'
        w, x, y, z = self.sens_quat.data
        imu.orientation.w = float(w)
        imu.orientation.x = float(x)
        imu.orientation.y = float(y)
        imu.orientation.z = float(z)
        gx, gy, gz = self.sens_gyro.data
        imu.angular_velocity.x = float(gx)
        imu.angular_velocity.y = float(gy)
        imu.angular_velocity.z = float(gz)
        ax, ay, az = self.sens_acc.data
        imu.linear_acceleration.x = float(ax)
        imu.linear_acceleration.y = float(ay)
        imu.linear_acceleration.z = float(az)
        self.imu_pub.publish(imu)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--scene', default=DEFAULT_SCENE)
    ap.add_argument('--no-viewer', action='store_true',
                    help='run headless (no GUI window)')
    ap.add_argument('--pub-every', type=int, default=2,
                    help='publish every N physics steps (2 -> 250Hz at dt=0.002)')
    ap.add_argument('--log-period', type=float, default=5.0,
                    help='seconds between debug pose log lines')
    args, ros_args = ap.parse_known_args()

    rclpy.init(args=ros_args)
    node = MujocoSimLite(args.scene)

    # Callbacks (command cache) run on a separate spin thread; the main
    # thread is the wall-clock-paced physics loop, as in the references.
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
            # simulator's sleep(dt - cost)).  If we fall badly behind, resync
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
