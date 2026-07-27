# ddt_rl_deploy

[English](README.md) | 中文

DDT 机器人强化学习策略部署。自包含仓库:sim2sim与 sim2real共用一套代码——上层 App 仅通过 ROS 2 topic 通信,差异仅在于对端(本地仿真或机载单元)。


## 机型支持

支持 **d1** —— 策略 `rl_flat` / `rl_flat_lab` / `rl_rough_lab`,sim2sim 与 sim2real 均可。

测试平台:Ubuntu 22.04 · ROS 2 Humble · Python 3.10。

## Quickstart

```bash
# 前置:系统已安装 ROS 2 Humble
./scripts/setup.sh                          # pip 依赖 + colcon build(默认:自包含 mujoco)

# sim2sim(三个终端):
./scripts/run_sim.sh                        # Mujoco(默认);--backend gazebo|webots、--robot、--terrain(webots)
./scripts/run_policy.sh rl_flat_lab d1      # 策略(可选项见"机型支持")
./scripts/run_teleop.sh                     # 遥控:ws 前后 · ad 转向 · qe 平移 · rf 升降 · 空格停 · x 退出

```

## 结构

```
deploy/     Python 应用:policy_engine(大脑)+ rl_inference(ROS 壳)+ joy_mapping + teleop
src/robot/  自己的 ROS 包:ddt_msgs、topic_command_controller(透传)、<robot>_description(URDF/xacro + MJCF)
src/sim/    sim2sim 后端(真机构建时整个跳过):mujoco_bridge(默认,自包含)
            + 可选 gazebo_bridge / webots_bridge
config/     <robot>/controllers.yaml + *.onnx —— 每机型策略,rl_inference 读
scripts/    shell 启动器:setup · run_sim · run_policy · run_teleop
vendor/     lodepng(MuJoCo 构建的源码依赖)
```

## 新增机型

放入两份文件,再用 `--robot <robot>` 运行。**若**策略用的观测都是引擎已有的
(见 `deploy/policy_engine.py` 的 `_OBS_SPECS`),则无需改代码;用到新观测则需在
那里加一条 + 一个对应的 `_obs_*` 函数。

- **`src/robot/<robot>_description/`** —— 一个 ament 包(照 `src/robot/d1_description` 建):
  `xacro/robot.xacro` + `xacro/ros2control.xacro`(声明每关节 position/velocity/
  effort/kp/kd 命令接口 + `trunk_imu` 传感器),以及 `mujoco/scene.xml` + `robot.xml`
  (IMU 传感器 `trunk_quat`/`trunk_gyro`/`trunk_accel`)。
- **`config/<robot>/`** —— `controllers.yaml`(rl_inference 读的策略参数:`joints`、
  每策略的观测/scale/增益)+ `*.onnx`。

```bash
./scripts/run_sim.sh --robot <robot>
./scripts/run_policy.sh <policy> <robot>
```

## 依赖

系统 **ROS 2 Humble** + pip(`requirements.txt`,由 `setup.sh` 统一处理)。默认(MuJoCo)后端自包含,无需任何外部工作空间。

**可选仿真后端** —— 默认关闭;先装好仿真器,再编译进来:

| 后端 | 编译 | 额外安装 |
|---|---|---|
| MuJoCo | `setup.sh`(默认) | 无(pip `mujoco`) |
| Gazebo | `setup.sh --with-gazebo` | gazebo classic + `ros-humble-gazebo-ros2-control` |
| Webots | `setup.sh --with-webots` | Webots R2025a + `ros-humble-webots-ros2` |

然后:`run_sim.sh --backend gazebo|webots`(webots 还可加 `--terrain empty_world|stairs|uneven`)。

## 故障排查

- **跨主机无法发现真机**:`env.sh` 设置了 `ROS_LOCALHOST_ONLY=1`(仅回环)。机载单元位于另一台主机时,注释掉该行(改用独立的 `ROS_DOMAIN_ID`),否则发现过程会静默失败。

## 已知限制

- 无脚本起身:节点上电直接进入 RL 模式,机器人须从可站立姿态开始(仿真初始位姿 / 人工扶持);机身翻覆时需人工扶正。
- Webots 后端 d1 站立有 sim2sim 差异,**策略验证以 Mujoco 为准**。
- 训练→部署配置仍需手工抄写。
