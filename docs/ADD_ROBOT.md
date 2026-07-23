# 添加一个新机器人

得益于 config 驱动的设计,**加机器人不用改任何代码**——丢一个模型 + 一份配置就行。
本文是照着填的清单。示例机型名用 `<robot>`(如 `d1h`)。

## 你需要准备

- 机器人 **URDF**(+ meshes)
- 训练好的策略 **ONNX** + 它的**元数据**(obs 顺序、各 scale、num_obs/num_actions/history_len)——来自训练侧导出。这些是 `# [TRAIN]` 字段,抄错了策略会吃到垃圾输入。

## 三步

### 1. 模型 → `models/<robot>_description/`

| 文件 | 给谁用 | 说明 |
|---|---|---|
| `urdf/robot.urdf` (+ `meshes/`) | pybullet / 真机 / 转 MJCF | 主模型,网格用相对路径 `../meshes/` |
| `mujoco/robot.xml` + `mujoco/scene.xml` | **mujoco 后端** | 从 URDF 转 MJCF;**手加 IMU 传感器** `trunk_quat`/`trunk_gyro`/`trunk_accel` + 执行器。⚠️ **MJCF 执行器顺序必须 == config 的 `joints` 顺序**(mujoco 后端启动会断言,不一致会响亮报错) |
| `sim/webots/protos/<ROBOT>.proto` + `sim/webots/worlds/<robot>.wbt` | webots 后端(可选) | proto 用 `urdf2webots` 生成 + 手加 IMU;world 抄 [d1.wbt](../sim/webots/worlds/d1.wbt) 换机型名(`run.sh --robot <robot>` 按名字找 world)。⚠️ 两个转换坑,都参照 [D1.proto](../sim/webots/protos/D1.proto):**轮子碰撞圆柱轴**(URDF 轴 Z vs Webots 默认 Y,要补 `rotation 1 0 0 1.5708`);**关节镇定层**(每关节 `dampingConstant 0.1` + `staticFriction 0.2`,对齐 MJCF 默认块,少了会振荡) |

### 2. 配置 → `config/<robot>/`

```bash
cp -r config/_template config/<robot>
```

编辑 [config/_template/controllers.yaml](../config/_template/controllers.yaml),把每个 `# <FILL>` 换成你机器人的值:
- `joints` — 全部关节名,**按训练/MJCF 执行器顺序**(名字含 `foot` 的自动当轮子;否则用 `wheel_indices`)
- `transform_up.stand_jpos` — **真实站姿**([SIM] simbase 读它当 hold 姿势;standup 也奔它去)
- `wheel_indices` / `torque_limit` — 来自 URDF
- 每个策略一个 block(`rl_policy_names` 里列名字,下面配同名块),填 `policy_path` + `[TRAIN]` 字段

再把 `*.onnx` 放进 `config/<robot>/`。

### 3. 跑

```bash
# 仿真(另开一个终端起后端):
./scripts/run_sim.sh --backend mujoco --robot <robot>
# 部署 App:
./scripts/run_policy.sh <policy> <robot>          # <policy> 取自 rl_policy_names
```

`--robot` 会被 [run_sim.sh](../scripts/run_sim.sh) 透传给后端,无需改脚本。

## 最容易站不起来的三个点

1. **`joints` 顺序 ≠ MJCF 执行器顺序** → mujoco 断言报错(这是好事,别绕过,去对齐)。
2. **`stand_jpos` 写成对称** → 非对称机器人(前后腿不同)会俯仰栽倒。d1 就是前腿 thigh `0.8` / 后腿 `1.0`,别抄成一样。
3. **IMU 约定错**(四元数 `w,x,y,z` + 机身系陀螺)→ 静默发散。装水平时投影重力应 ≈ `[0,0,-1]`。对表用 mujoco 后端最稳。

## 校验清单

```bash
# spec 能从 config 加载:
python3 -c "import sys; sys.path.insert(0,'sim'); from simbase import spec; s=spec('<robot>'); print(len(s.joint_names), list(s.hold_pose))"
```
- [ ] `spec('<robot>')` 打印出对的关节数 + stand_jpos
- [ ] `run_sim.sh --backend mujoco --robot <robot>` 起来**无断言错**
- [ ] `num_obs`/`num_actions` 与 ONNX 一致(App 会 dim-check,不对直接拒跑)
- [ ] 挂策略后 `standup → rl`,pitch/roll 收敛不发散

## 它是怎么接线的(为什么零代码)

- [sim/simbase.py](../sim/simbase.py) 的 `spec(robot)` 从 `config/<robot>/controllers.yaml` 读 `joints` + `stand_jpos` → 任何继承 `SimBackend` 的后端自动拿到,不写死。
- 后端本身**丢文件即发现**:`sim/<name>_sim.py`(纯 Python)或 `sim/<name>/run.sh`(框架),见 [sim/BACKEND.md](../sim/BACKEND.md)。
- [deploy/rl_inference.py](../deploy/rl_inference.py) 读同一份 config 的策略块——**sim 和 deploy 共用一个真相源**,不会漂。
