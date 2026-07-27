# ddt_rl_deploy

[English](README.md) | 中文

DDT 机器人强化学习策略部署。自包含仓库:sim2sim与 sim2real共用一套代码——上层 App 仅通过 ROS 2 topic 通信,差异仅在于对端(本地仿真或机载单元)。


## 机型支持

支持 **d1** —— 策略 `rl_flat` / `rl_flat_lab` / `rl_rough_lab`,sim2sim 与 sim2real 均可。

测试平台:Ubuntu 22.04 · ROS 2 Humble · Python 3.10。

## Quickstart

```bash
# 前置:系统已安装 ROS 2 Humble
./scripts/setup.sh                          # pip 依赖 + colcon build

# sim2sim(三个终端):
./scripts/run_sim.sh                        # Mujoco 仿真(--backend webots 切换后端,--robot 切换机型)
./scripts/run_policy.sh rl_flat_lab d1      # 策略(可选项见"机型支持")
./scripts/run_teleop.sh                     # 遥控:ws 前后 · ad 转向 · qe 平移 · rf 升降 · 空格停 · x 退出

```

## 结构

```
deploy/   policy_engine.py(纯函数大脑)+ rl_inference.py(ROS 壳)
src/      colcon 工作区:ddt_msgs;MuJoCo ros2_control 桥(mujoco_sim_ros2 +
          mujoco_ros2_control + mujoco_bridge);topic_command_controller(透传,
          把 joint_command 喂给仿真);<robot>_description(URDF/xacro + MJCF)
config/   <robot>/controllers.yaml + *.onnx —— 每机型策略,rl_inference 读
scripts/  setup · run_sim · run_policy · run_teleop
vendor/   lodepng(MuJoCo 构建的源码依赖)
```

## 新增机型

放入两份文件,再用 `--robot <robot>` 运行。**若**策略用的观测都是引擎已有的
(见 `deploy/policy_engine.py` 的 `_OBS_SPECS`),则无需改代码;用到新观测则需在
那里加一条 + 一个对应的 `_obs_*` 函数。

- **`src/<robot>_description/`** —— 一个 ament 包(照 `src/d1_description` 建):
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

系统 **ROS 2 Humble** + pip(`requirements.txt`,由 `setup.sh` 统一处理)。无需任何外部工作空间。

## 故障排查

- **跨主机无法发现真机**:`env.sh` 设置了 `ROS_LOCALHOST_ONLY=1`(仅回环)。机载单元位于另一台主机时,注释掉该行(改用独立的 `ROS_DOMAIN_ID`),否则发现过程会静默失败。

## 已知限制

- 无脚本起身:节点上电直接进入 RL 模式,机器人须从可站立姿态开始(仿真初始位姿 / 人工扶持);机身翻覆时需人工扶正。
- Webots 后端 d1 站立有 sim2sim 差异,**策略验证以 Mujoco 为准**。
- 训练→部署配置仍需手工抄写。
