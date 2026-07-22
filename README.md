# ddt_rl_deploy

DDT 机器人(d1/d1h/tita)强化学习策略部署。**一个自包含仓库,同时支持 sim2sim(本地 Mujoco)和 sim2real(真机)**,零外部工作空间依赖。

架构照 DeepRobotics `sdk_deploy`:上层 App 只讲 ROS2 topic,sim 和真机靠相同 topic 通吃,换的只是"对面挂谁"——本地 Mujoco 节点 / 机载运动单元。

## 结构

```
ddt_rl_deploy/
├── src/ddt_msgs/            # 消息契约(JointControlCommand 等),仓库自带,唯一要 build 的包
├── deploy/rl_inference.py   # 上层部署 App(核心)。讲 topic → sim2sim 与 sim2real 通用
│        rl_inference_origin.py   # 稳定基线参照
├── sim/
│   ├── mujoco_sim.py        # sim2sim 后端(纯 DDS、passive viewer 不阻塞物理)
│   └── assets/d1/           # 机器人模型(mjcf + meshes),仓库自带
├── config/d1/               # controllers.yaml + *.onnx 策略
├── scripts/  run_sim.sh · run_policy.sh
└── env.sh                   # 自包含环境(系统 ROS2 + 自 build 的 ddt_msgs)
```

## 依赖

- 系统 **ROS 2 Humble**(`/opt/ros/humble`)
- pip:`mujoco`、`onnxruntime`(装完 `pip uninstall -y numpy` 保系统 numpy 1.21.5)、`pyyaml`
- **不需要任何外部工作空间**(ddt_msgs、机器人模型都在仓库里)

## 构建(一次)

```bash
cd ddt_rl_deploy
# 只 source 系统 ROS,build 仓库自带的 ddt_msgs
bash -c 'source /opt/ros/humble/setup.bash && colcon build'
```

## 运行

**sim2sim(本地 Mujoco):**
```bash
# 终端1:仿真
./scripts/run_sim.sh                    # 裸机 d1(scene.xml)+ GUI
# 终端2:策略
./scripts/run_policy.sh rl_flat_lab d1
```

**sim2real(真机):** 遥控器进 `08 SDK Mode` 后,只跑策略(对面是机载单元,rl_inference 代码不变):
```bash
./scripts/run_policy.sh rl_flat_lab d1
```

**键盘控制**(另开终端,同 env):
```bash
source env.sh && ros2 run keyboard_controller keyboard_controller_node   # 需真机侧提供该节点
```
w/s 前后 · a/d 转向 · ←/→ 平移。

## 模型与策略

本仓库只带**裸机 d1**(`scene.xml`)+ 对应的 57 维策略:`rl_flat` / `rl_flat_lab` / `rl_rough_lab`(均无 base_height 高度命令)。cargo(带货物)变体已移除。

## 关键设计

- **sim==真机**:rl_inference 讲 topic(`command/joint_command`/`joint_states`/`imu`),不区分对端。
- **自钟控制回路**:回调只写状态缓存,5ms 定时器驱动控制,推理按消息 stamp 时间门控 50Hz(对齐训练),不跟 topic 到达时序走。
- **网络隔离**:`env.sh` 默认 `ROS_LOCALHOST_ONLY=1`(防局域网串扰);跨网连真机时注释掉,改独占 `ROS_DOMAIN_ID`。
- **安全**:机载单元保持最后一条指令、无超时 → 高 kp 后退出会绷死机器人。看门狗 + 退出降阻尼是待补的 FSM 安全外壳。

## 已知缺口

- **FSM 安全外壳未做**:开机瘫软直接切策略会翻(sim 里 spawn 落地后要么先 `reset` 要么等 hold)。真机前须补"先站起再交策略":自建 Idle/StandUp/Damping,或发官方 `command/user_command` 的 `fsm_mode: transform_up` 让机载 FSM 站起。
- `config/d1` 与厂商 `rl_controller` 的同名配置曾是复制关系;本仓库现自持一份。
