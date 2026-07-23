# ddt_rl_deploy

DDT 机器人强化学习策略部署。**一个自包含仓库,同时支持 sim2sim(Mujoco / Webots)和 sim2real(真机)**,零外部工作空间依赖。

当前支持:**d1**(平地/粗糙地形策略)与 **d1cargo_out**(带货箱变高度爬楼策略)全链可跑;d1h / tita 模型就绪,补 config + onnx 即可(见"加新机型")。

架构照 DeepRobotics `sdk_deploy`:上层 App 只讲 ROS2 topic,sim 和真机靠相同 topic 通吃,换的只是"对面挂谁"——本地仿真节点 / 机载运动单元。

## Quickstart

```bash
# 0. 前置:系统装好 ROS 2 Humble(/opt/ros/humble)
./scripts/setup.sh                          # 一键:pip 依赖 + colcon build

# sim2sim 三个终端:
./scripts/run_sim.sh                        # 终端1:Mujoco 仿真(GUI;--backend webots 换后端,--robot 换机型)
./scripts/run_policy.sh rl_flat_lab d1      # 终端2:策略(--list 看可选)
./scripts/run_teleop.sh                     # 终端3:键盘遥控

# 变高度爬楼策略(带货箱):
#   ./scripts/run_sim.sh --robot d1cargo_out
#   ./scripts/run_policy.sh rl_height_cargo_out d1cargo_out    # r/f 键升降高度

# sim2real:遥控器进 `08 SDK Mode` 后,只跑终端2+3(对面是机载单元,代码不变)
```

遥控按键:`w/s` 前后 · `a/d` 转向 · `q/e` 平移 · `r/f` 升降 · `空格` 停 · `x` 退出。

## 结构

```
ddt_rl_deploy/
├── src/ddt_msgs/            # 消息契约(JointControlCommand 等),仓库自带,唯一要 build 的包
├── deploy/
│   ├── policy_engine.py     # 大脑:观测拼装/ONNX 推理/解码,纯函数、零 ROS,可单测可移植
│   ├── rl_inference.py      # 壳:ROS 收发 + 双定时器 + standup/rl/damping 安全模式
│   ├── rl_inference_origin.py   # 未重构基线参照
│   └── DATAFLOW.md          # 数据流速查(函数一句话 + 易错点)
├── models/                  # 机器人模型(URDF 为主,用户维护),仓库自带
│   └── <robot>_description/
│       ├── urdf/robot.urdf  # 主模型:pybullet/gazebo/真机都吃它(网格用相对路径)
│       ├── meshes/*.STL     # 共享网格
│       └── mujoco/*.xml     # MJCF 派生件(含手加的执行器+IMU传感器),仅 mujoco 后端用
├── sim/
│   ├── simbase.py           # 后端共享核心:RobotSpec(从 config 读)+ SimBackend(PD/收发)
│   ├── mujoco_sim.py        # mujoco 后端(纯 DDS、自节流实时、passive viewer 不阻塞物理)
│   ├── webots/              # webots 后端(extern controller,零 webots_ros2;--gui 走独显渲染)
│   └── BACKEND.md           # 后端契约规格书(写新仿真器照它填)
├── config/
│   ├── _template/           # 加新机型的骨架(全字段注释,照 <FILL> 填)
│   ├── d1/                  # controllers.yaml + *.onnx(平地/粗糙策略)
│   └── d1cargo_out/         # 带货箱变高度爬楼策略
├── docs/ADD_ROBOT.md        # 加新机型三步指南
├── scripts/  setup.sh · run_sim.sh(--backend/--robot)· run_policy.sh · run_teleop.sh
└── env.sh                   # 自包含环境(系统 ROS2 + 自 build 的 ddt_msgs)
```

## 依赖

- 系统 **ROS 2 Humble**(`/opt/ros/humble`)
- pip:见 `requirements.txt`(`./scripts/setup.sh` 一键处理,含 numpy ABI 兼容)
- **不需要任何外部工作空间**(ddt_msgs、机器人模型都在仓库里)

## 模型与策略

机器人模型在 `models/<robot>_description/`,**以 URDF 为主**(用户维护的那份):`urdf/robot.urdf` 用相对路径引网格,pybullet / gazebo / 真机都直接吃它。`mujoco/*.xml` 是 **MJCF 派生件**——在 URDF 运动学之上手加了 mujoco 专属的执行器 + IMU 传感器,**只给 mujoco 后端用**;换了模型要重新转一次 MJCF,不会自动同步。仓库自带 `d1 / d1h / tita` 三套。

> 训练侧(DDT_lab)若把 `DDT_MODEL_DIR` 指到这里的 `models/`,训练与部署就共用同一份模型源,`ddt_ros2_control` 不再是任何流程的必需件。

**加一个新机型**:不用改代码——丢一份模型 + 一份 config 即可。模板在 [config/_template/](config/_template/controllers.yaml)(全字段带注释),步骤见 [docs/ADD_ROBOT.md](docs/ADD_ROBOT.md)。`sim/simbase.py` 的 `spec(robot)` 从 `config/<robot>/controllers.yaml` 读关节序 + 站姿,sim 与 deploy 共用这一份真相源。

自带策略:
- **d1**(57 维):`rl_flat` / `rl_flat_lab` / `rl_rough_lab`
- **d1cargo_out**(58 维 = 57 + 高度命令):`rl_height_cargo_out` — 变高度爬楼,高度命令 0.19~0.50 m,由 teleop 的 `r/f` 键(twist.linear.z 积分)控制

策略与配置在**加载时做维度硬校验**,配错对(如 57 维配置配 58 维模型)直接报人话拒绝启动。

## 关键设计

- **sim==真机**:rl_inference 讲 topic(`command/joint_command`/`joint_states`/`imu`),不区分对端。
- **脑/壳分离**:`policy_engine.py` 纯函数(RobotState 进 → JointCommand 出,零 ROS),观测按注册表拼装(维度与计算同源);`rl_inference.py` 只管 ROS 收发与节奏。
- **墙钟双定时器**(DeepRobotics/LeggedLab 同款):推理定时器跑 `control_dt`(50Hz,对齐训练),发布定时器 200Hz(轮子阻尼项跟实时轮速)。前提是状态源实时——本仓库 sim 自节流到墙钟,真机天然满足。
- **安全外壳**(一个模式变量,零状态机类):
  - `standup`:镜像 `FSMState_TransformUp`,fold→stand 两段插值站起,参数从 yaml `transform_up` 段读;
  - `rl`:姿态守卫 roll>30°/pitch>45° → 切阻尼;
  - `damping`:腿 kd=5/轮 kd=0(镜像 `FSMState_Passive`);
  - **断流看门狗**:joint_states 断流 >0.2s → 切阻尼;
  - **退出保护**:Ctrl-C/SIGTERM 统一走阻尼连发再退出(机载单元会永久保持最后一条指令,高 kp 下直接退出会绷死机器人)。
- **网络隔离**:`env.sh` 默认 `ROS_LOCALHOST_ONLY=1`(防局域网串扰);跨网连真机时注释掉,改独占 `ROS_DOMAIN_ID`。

## 已知缺口

- 站起不含翻身(四脚朝天需人工扶正;参照的 C++ RollOver 同样未启用)。
- 训练→部署的 yaml 仍是手抄,自动导出 manifest + 更全的加载校验待做。
- **Webots 后端**:契约/PD/IMU 全链验证过,但 d1 站立仍有 roll 残余晃动(±35°,不倒)——IsaacLab↔ODE 的接触/执行器模型差异所致,Webots 关节无 armature 字段补不齐;**策略验证以 mujoco 为准**,Webots 当可视化/第二意见用。
- d1cargo_out 的 onnx 来自训练中的 checkpoint(`d1cargo_out_height_stairs`);训练收尾后重导出覆盖 `config/d1cargo_out/height_cargo_out.onnx` 即可,无代码改动。
