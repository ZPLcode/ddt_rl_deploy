# 仿真/真机后端契约

部署 App(`deploy/rl_inference.py`)是**后端无关**的:它只讲下面这套 ROS 2 topic
契约。任何后端——某个仿真器,或真机的机载单元——只要讲同一套契约,就能接上,
**App 一个字不改**。这就是 sim==real 的实现方式。

这份文件是给"写一个新后端"的人看的规格书(相当于 Lite3 的 `RobotInterface.h`,
但落在进程边界上,而不是编译进 App 的接口类)。

## 后端必须做的三件事

| # | 做什么 | topic | 类型 |
|---|---|---|---|
| 1 | **发关节状态** | `joint_states` | `sensor_msgs/JointState` |
| 2 | **发 IMU** | `imu_sensor_broadcaster/imu` | `sensor_msgs/Imu` |
| 3 | **收关节命令并执行** | `command/joint_command` | `ddt_msgs/JointControlCommand` |

以及一条隐性要求:**按实时(1× 墙钟)推进**(见下)。

---

## 1. `joint_states`(后端发)

- `name[]`:关节名,**必须与 `config/<robot>/controllers.yaml` 的 `joints` 列表一致**
  (App 按名字匹配,顺序不强制,但名字要对得上)。
- `position[]` / `velocity[]`:每关节角/角速度。**必需**。
- `effort[]`:可选(App 不读)。
- `header.stamp`:建议填物理时间(App 目前用墙钟节奏,不依赖它,但填上有益调试)。
- **QoS**:`BEST_EFFORT` / `VOLATILE` / `KEEP_LAST(1)`(与 App 订阅端一致;
  RELIABLE 发布也兼容)。

## 2. `imu_sensor_broadcaster/imu`(后端发)

- `orientation`:机身姿态四元数。App 读 `w, x, y, z`(注意 ROS 消息字段是 x,y,z,w,
  按字段名填即可)。**必需**。
- `angular_velocity`:机身系角速度(陀螺)。**必需**。
- `linear_acceleration`:可选(App 不读,填上无妨)。
- `header.frame_id`:建议 `trunk_imu`。
- QoS:同 `joint_states`。

## 3. `command/joint_command`(后端收,并执行)

App 每关节发一个 MIT 五元组;后端按 PD + 前馈执行:

```
τ_joint = effort + kp · (position − q) + kd · (velocity − dq)
```

- 字段:`header, name[], kp[], kd[], position[], velocity[], effort[]`(按 `name` 对号入座)。
- **腿**:position 目标 + kp/kd → 位置 PD。
- **轮(P_V)**:velocity 目标 + kd(kp=0)→ 速度控制。
- **轮(P)**:effort 已由 App 算好,kp=kd=0 → 直接施加力矩。
- 后端**每个物理步都用最新缓存的命令重算**(零阶保持),命令到达节奏不影响力矩平滑。
- QoS:`RELIABLE` / `KEEP_LAST(10)`(与 App 发布端一致)。

---

## 约定(容易漏)

- **实时节流**:后端必须把物理推进节流到 ≈1× 墙钟(如 `sleep(dt − 本步耗时)`)。
  App 用墙钟定时器锁 50Hz 推理,**前提就是状态源是实时的**;跑成非实时(无头狂奔)
  会让策略频率错乱。真机天然满足;仿真必须自己节流。
- **命名空间**:若设了环境变量 `ROBOT_NS`,所有 topic 要加 `<ROBOT_NS>/` 前缀
  (App 侧也会加,两边要一致)。
- **初始位姿**:首条命令到达前,后端应 PD 保持一个稳定位姿(别让机器人在 App 连上
  之前就瘫掉)。App 的 standup 模式会从当前实测位姿把它抬起来。
- **关节名/IMU 约定错了不会报错,只会行为异常**——名字对不上则该关节读到 0、
  发的命令被丢;四元数/角速度坐标系错则策略发散。对表用参照实现最稳。

---

## 参照实现 & 加一个新后端

- **共享核心**:`sim/simbase.py` 是单一真相源——`RobotSpec`(关节序 / hold 姿势 / 增益)
  + `SimBackend(Node)`(命令缓存、MIT-PD、state+imu 收发与 QoS,全在这里)。
  **新后端继承 `SimBackend`,只写引擎绑定**(读 q/dq、施力、读 IMU、驱动 step 循环);
  契约细节不用重抄、也不会在后端之间漂移(以前抄两遍漂出过 bug)。
- **参照**:`sim/mujoco_sim.py`(最完整,含 passive viewer + 墙钟节流)与
  `sim/webots/webots_sim.py`(最薄,~100 行)都是范例。
- **轻量后端**(纯 Python + DDS,如 PyBullet):新建 `sim/<name>_sim.py`,继承 `SimBackend`。
  `./scripts/run_sim.sh --backend <name>` 会自动发现。
- **框架后端**(Gazebo/Webots 等,需各自 ros2 集成):新建目录 `sim/<name>/`,
  放一个可执行的 `sim/<name>/run.sh` 作为入口(内部启 launch / 控制器节点,
  只要最终讲上面的 topic 契约即可)。同样被 `--backend <name>` 自动发现。
- **真机**:不用写后端——机载单元讲同一套契约(进遥控器 `08 SDK Mode` 后接管)。
  只跑 `run_policy.sh` + `run_teleop.sh`,对面换成真机。

`./scripts/run_sim.sh --list-backends` 列出当前已装好的后端。
