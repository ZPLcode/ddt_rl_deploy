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
                                            # 加 gazebo/webots:--with-gazebo | --with-webots | --with-all(见"依赖")

# sim2sim(三个终端):
./scripts/run_sim.sh                        # Mujoco(默认);--backend gazebo|webots、--robot、--terrain(webots)
./scripts/run_policy.sh rl_flat_lab d1      # 策略(可选项见"机型支持")
./scripts/run_teleop.sh                     # 遥控:ws 前后 · ad 转向 · qe 平移 · rf 升降 · 空格停 · x 退出

```

## 结构

```
src/description/  机器人模型,每机型一个 ament 包:mesh + xacro + MuJoCo XML
src/config/       每机型一个文件夹(非包):<robot>/deploy.yaml + ONNX 策略
src/control/      ddt_msgs + topic 控制器 + Python 策略/遥控应用
src/sim/          MuJoCo(自包含)+ 可选 Gazebo / Webots bridge
scripts/          setup · run_sim · run_policy · run_teleop · new_robot
vendor/           MuJoCo 构建使用的 lodepng 源码依赖
```

## 新增机型

跑 `./scripts/new_robot.sh <robot>` 生成两处骨架,再往里放文件即可——**无需手写 CMakeLists**。
然后用 `--robot <robot>` 运行。**若**策略用的观测都是引擎已有的
(见 `src/control/inference/policy_engine.py` 的 `_OBS_SPECS`),则无需改代码;用到新观测则需在
那里加一条 + 一个对应的 `_obs_*` 函数。

- **`src/description/<robot>_description/`** —— 一个 ament 包(照 `src/description/d1_description` 建):
  `xacro/robot.xacro` + `xacro/ros2control.xacro`(声明每关节 position/velocity/
  effort/kp/kd 命令接口 + `trunk_imu` 传感器),以及 `mujoco/scene.xml` + `robot.xml`
  (IMU 传感器 `trunk_quat`/`trunk_gyro`/`trunk_accel`)。**必须是包**:xacro `$(find …)`、
  `package://` 网格、MuJoCo `model_package` 都靠它解析。
- **`src/config/<robot>/`** —— 一个普通文件夹(**非包**):唯一的 `deploy.yaml`
  由所有仿真器和 `rl_inference` 共用,旁边放它引用的 `*.onnx` 策略。按路径解析
  (`src/config/<robot>/deploy.yaml`),**无需编译**。

```bash
./scripts/new_robot.sh <robot>          # 生成 description 包 + config 文件夹
# ... 放模型文件 + deploy.yaml + onnx ...
./scripts/setup.sh
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
| 两者 | `setup.sh --with-all` | 上面两项都装 |

然后:`run_sim.sh --backend gazebo|webots`(webots 还可加 `--terrain empty_world|stairs|uneven`)。

## 故障排查

- **跨主机无法发现真机**:`env.sh` 设置了 `ROS_LOCALHOST_ONLY=1`(仅回环)。机载单元位于另一台主机时,注释掉该行(改用独立的 `ROS_DOMAIN_ID`),否则发现过程会静默失败。

## 已知限制

- 仿真会以站姿生成,并在第一条策略关节命令到达前主动保持该姿态。
  仍无翻身/起身动作:真机须人工扶到可站姿态,仿真或真机翻覆后也需人工复位。
- 训练→部署配置仍需手工抄写。
