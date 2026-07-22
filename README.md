# ddt_rl_deploy

DDT 机器人(d1 / d1h / tita)强化学习策略的**上层部署**。一个独立节点订状态、跑 ONNX 策略、发逐关节 MIT 命令给厂商机载运动单元(或仿真桥),完全绕开 ros2_control 框架。换机器人/换策略只改 `config/`,代码不动。

## 这是什么(架构定位)

```
本仓库(上层部署 App) ──command/joint_command──▶  接收方(不在本仓库)
  deploy/rl_inference.py                          仿真: topic_command_controller / gazebo_bridge
  订 joint_states + imu + cmd_twist/cmd_pose      真机: 机载闭源运动单元 → CAN → 电机
```

`rl_inference.py` 相当于官方 D1-ROS2-SDK **低层 `joint_command` 通道**上的策略大脑;等价于云深处 `Lite3_sdk_deploy` / LeggedLab `deploy.py`。同一份代码,仿真和真机靠相同 topic 名通吃。

## 目录

| 路径 | 内容 |
|---|---|
| `deploy/rl_inference.py` | 上层部署 App(核心)。观测拼装/低通/重力/轮子/ONNX/解码全在此 |
| `deploy/rl_inference_origin.py` | 稳定基线参照(勿改),对比用 |
| `topic_command_controller/` | 仿真桥:ros2_control 控制器,订 `command/joint_command` 写 loaned interface。含 mujoco/gazebo 两个 launch |
| `sim/mujoco_sim.py` | 轻量仿真(无 ros2_control,纯 DDS,passive-viewer 不阻塞物理) |
| `config/d1/` | `controllers.yaml`(关节/增益/观测/scale/wheel/策略)+ `*.onnx` 策略 |
| `scripts/` | 一键脚本 |

## 依赖(外部,不在本仓库)

需要一个已 build 的 `ddt_ros2_ws`(默认 `/home/zhepeng/DDT/ddt_ros2_ws`,用环境变量 `DDT_WS` 覆盖),它提供:
- **ddt_msgs**(来自官方 D1-ROS2-SDK-Demo)—— `JointControlCommand` 等消息类型,**硬依赖**
- **机器人描述**(`d1_description` 的 mujoco xml / xacro)、**gazebo_bridge**、**mujoco_ros2_control** —— 仿真用

系统包:`ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control ros-humble-angles`,`pip install --user onnxruntime`(装完 `pip uninstall -y numpy` 保系统 numpy)。

本仓库的 `topic_command_controller` 需软链进 ws 才能被 colcon build:
```bash
ln -sfn ~/DDT/ddt_rl_deploy/topic_command_controller ~/DDT/ddt_ros2_ws/src/topic_command_controller
cd ~/DDT/ddt_ros2_ws && colcon build --packages-select topic_command_controller
```

## 快速开始

```bash
# 终端1:仿真(推荐 gazebo,渲染分进程不干扰状态流)
./scripts/run_sim_gazebo.sh d1
# 终端2:策略
./scripts/run_policy.sh rl_flat_lab d1
```
策略名(d1):`rl_flat / rl_flat_lab / rl_rough_lab / rl_rough_cargo / rl_height_cargo / rl_height_cargo_out`。

其他仿真:`run_sim_mujoco.sh`(ros2_control 版,GUI 下有渲染卡顿)、`run_sim_lite.sh [--no-viewer]`(轻量无 ros2_control)。

## 关键设计(踩坑换来的)

- **自钟控制回路**:`_joint_cb` 只写缓存;5ms 定时器驱动控制,推理按消息 stamp 时间门控 50Hz(对齐训练 decimation4×dt0.005)。控制节奏不跟 joint_states 到达时序走 —— 这是所有参考部署(LeggedLab/roboparty/rl_sar/sdk_deploy)的公共模式。
- **sim == 真机**:靠 topic 名一致,代码不区分对端。
- **网络隔离**:局域网若有别的机器共用 `ROS_DOMAIN_ID=0` 会串扰(灌 cmd_twist/cmd_pose)。调试前 `export ROS_LOCALHOST_ONLY=1`;真机用独占 domain。
- **安全**:机载运动单元**保持最后一条指令、无超时**。高 kp 命令后退出会绷死机器人 —— 部署侧需看门狗+退出降阻尼。

## 已知缺口

- **FSM 安全外壳未做**:开机瘫软直接切策略会翻。真机前须补:先站起再交策略。两条路——(A) 自建 Idle/StandUp/Damping FSM;(B) 发官方 `command/user_command` 的 `fsm_mode: transform_up` 让机载 FSM 站起,再切逐关节。B 更省事,须真机验证交接时序。
- `config/d1/` 与 `ddt_ros2_control` 里 C++ rl_controller 的同名配置是**复制**关系,重训后需两边同步(或后续改为单一来源)。
