# rl_inference 数据流速查

一页看懂"传感器 → 策略 → 电机"这条闭环。`rl_inference.py`(壳)+ `policy_engine.py`(脑)是 C++ `FSMState_RL` 的 Python 镜像。`rl_inference_origin.py` 是未重构基线参照。

## 主线(spin 之后每一拍)

```
joint_states 到 ─► 回调缓存 ─► 双墙钟定时器 ─► 推理/解码 ─► 发命令 ─► 电机
     _joint_cb        RobotState    infer@50Hz     engine.infer   _publish
                                    publish@200Hz  engine.decode  JointControlCommand
```

## 模式(自包含安全外壳,一个字符串变量,零状态机类)

```
standup ──完成──► rl ──姿态超限(roll>30°/pitch>45°)──► damping(锁死)
   任何模式 ──joint_states 断流>0.2s──► damping(锁死)
   任何时刻退出(Ctrl-C/SIGTERM) ──► main finally: 阻尼连发20条 ──► 退出
```

- **standup**:镜像 `FSMState_TransformUp` 两段线性插值——fold(时长=fold_timer×最大关节偏差)→ stand(stand_timer)+100拍settle;专属 kp/kd/前馈来自 yaml `transform_up` 段;轮子恒 kp=0/kd 阻尼/不折叠
- **damping**:镜像 `FSMState_Passive`——全 0,腿 kd=5、轮 kd=0
- **退出降阻尼**:下游 bridge 永久保持最后一条指令(官方 SDK 文档),高 kp 下退出=绷死;所以 SIGINT/SIGTERM 统一走 finally 的阻尼连发
```

## 函数速查(执行序)

| 函数 | 一句话 |
|---|---|
| `_declare_params` | 声明所有 ROS 参数 + 默认值 |
| `_load_yaml_section` / `_find_key_recursive` | 从 controllers.yaml 抠出一个策略段(点分路径→全树递归搜) |
| `_load_params` + `y()` | 融合"YAML 优先、ROS 默认兜底" → `self._xxx` |
| `_init_onnx` | 加载 .onnx 成会话,读出输入名(喂数据靠名字) |
| `_obs_dim` + `_init_state` | 按维度表预分配观测/历史环/缓存(维度合计必须 = 模型输入,如 57) |
| `_create_subscriptions` + 4 回调 | 订 joint_states/imu/cmd_twist/cmd_pose;回调只填缓存 |
| `_control_loop` / `step` | 门控:该不该推理;每拍都发布 |
| `_update_observations` | 先移历史、再算新观测;按 obs 注册表逐段拼 |
| `_build_np3o/ppo/asap` | 把 obs+历史拼成模型要的输入布局 |
| `_run_inference` | 按 policy_type 选布局 → session.run → `_last_actions` |
| `_publish_joint_command` / `decode` | 解码成 MIT 五元组,发 command/joint_command |

## 易错点(踩过的坑)

| 坑 | 说明 |
|---|---|
| **推理频率漂** | origin 在 `_joint_cb` 里按帧门控 `_iter % decimation`,频率随 joint_states 到达率漂。重构版用**墙钟定时器**跑 `control_dt`(照 DeepRobotics/LeggedLab)→ 锁 50Hz。**前提:状态源实时**(sim 自节流到 realtime、真机天然实时);非实时 sim 会跑错频率 |
| **速度命令失灵** | origin 订 `TwistStamped`,而键盘/rl_controller 发 `Twist`,类型不匹配 DDS 静默丢。重构版改 `Twist` |
| **轮子解析漏** | origin 只认 `wheel_names`;d1 的 yaml 给的是 `wheel_indices` → 轮子被当普通关节位置控制、角度污染 dof_pos。重构版补 `wheel_indices→名字` 分支 |
| **首帧零历史** | Isaac Lab reset 后首 append 把历史铺满当前 obs(circular_buffer.py:136-139)。部署首帧也必须预填,否则起步 ~0.2s 喂零历史(训练没见过) |
| **低通读旧值** | `_update_observations` 先移历史(存旧 `_obs`)、再算新值;ang_vel 低通 `0.03*旧+0.97*新` 依赖此时 `_obs` 还是旧值 —— 顺序不能反 |
| **重力要转置** | `R.T @ [0,0,-1]`:Python `quat_to_rotation_matrix` 返回机身→世界,取逆(=转置)得机身系重力 |
| **base_height 是积分** | 命令里 base_height = 对 twist.linear.z 积分(`height += vz*infer_dt`)再 clip,不是直给 |
| **last_actions 存原始** | `_last_actions` = 网络原始输出(未缩放);下一拍喂回观测 + 解码都用它 |
| **推理疏、发布密** | 推理每 N 帧,发布每帧;轮子 P 模式 `effort=(...)·kp − kd·实时轮速` 靠每帧发布刷新阻尼 |
| **关节序 = 策略序** | 不做 reindex,靠"config 关节序 == 训练动作序" + 命令带名字。序写错会静默错位 |

## 关键契约

- **MIT 五元组** `{position, velocity, effort, kp, kd}` → 下游 `τ = effort + kp·(position−q) + kd·(velocity−dq)`
  - 腿:`position = action·scale + default`,kp/kd 交下游 PD
  - 轮 P:`effort = (action·scale+default)·kp − kd·实时轮速`,其余 0(下游 τ=effort)
  - 轮 P_V:`velocity = action·scale`,只留 kd(下游 τ=kd·(vel−dq))
- **三种输入布局**(必须匹配训练框架):
  - np3o:两输入 `(1,obs)` + `(1,hist,obs)`,历史保持 2D
  - ppo:一输入,历史(先obs名后帧)拼平 + 当前
  - asap:一输入,obs 子集,历史新→老
- **历史环布局**:`[t-H+1 … t-1, t]`,行 0 最老、行 -1 最新(与 Isaac Lab CircularBuffer 一致)

## origin vs 重构版

| | origin | 重构版(engine + shell) |
|---|---|---|
| 驱动 | `_joint_cb` 每帧调 `_control_loop` | 双墙钟定时器:`infer@control_dt` + `publish@200Hz` |
| 频率 | 帧计数 `_iter % decimation`(随到达率漂) | 墙钟 `control_dt` 锁 50Hz(需状态源实时) |
| 速度命令 | `TwistStamped`(错) | `Twist` |
| 轮子 | 只认 wheel_names | + wheel_indices |
| 首帧历史 | 零(与训练不符) | 预填当前 obs |
| 结构 | 数学与 rclpy 焊死 | `PolicyEngine`(纯,零 ROS)+ 薄壳 |
| obs 拼装 | 一个大 if/elif | `_OBS_SPECS` 注册表 + 派发 |
