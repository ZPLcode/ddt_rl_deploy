# ddt_rl_deploy

[English](README.md) | 中文

DDT 机器人强化学习策略部署。自包含仓库:sim2sim与 sim2real共用一套代码——上层 App 仅通过 ROS 2 topic 通信,差异仅在于对端(本地仿真或机载单元)。


## 机型支持

`d1` 是交付件,其余为实验性或仅有模型。

| 机型 | 策略 | sim2sim | sim2real | 状态 |
|---|---|:-:|:-:|---|
| **d1** | `rl_flat` / `rl_flat_lab` / `rl_rough_lab` | ✓ | ✓ | 已支持 |
| **d1h** | — | ○ | ○ | 仅模型——补 config + onnx 即用 |
| **tita** | — | ○ | ○ | 仅模型——补 config + onnx 即用 |

测试平台:Ubuntu 22.04 · ROS 2 Humble · Python 3.10。

## Quickstart

```bash
# 前置:系统已安装 ROS 2 Humble
./scripts/setup.sh                          # pip 依赖 + colcon build

# sim2sim(三个终端):
./scripts/run_sim.sh                        # Mujoco 仿真(--backend webots 切换后端,--robot 切换机型)
./scripts/run_policy.sh rl_flat_lab d1      # 策略(--list 查看可选项)
./scripts/run_teleop.sh                     # 遥控:ws 前后 · ad 转向 · qe 平移 · rf 升降 · 空格停 · x 退出

# sim2real:遥控器进入 08 SDK Mode 后仅运行后两条命令,对端换为机载单元,代码不变
```

## 结构

```
deploy/       policy_engine.py(纯函数大脑)+ rl_inference.py(ROS 壳)。数据流见 DATAFLOW.md
sim/          simbase.py(后端共享核心)+ mujoco_sim.py + webots/。topic 契约见 BACKEND.md
config/       controllers.yaml + *.onnx(按机型);_template/ 是新增机型的骨架
models/       机器人模型(URDF 为主 + mujoco MJCF 派生):d1 / d1h / tita / d1cargo_out
scripts/      setup · run_sim · run_policy · run_teleop
src/ddt_msgs/ 消息契约,唯一需要 build 的包
```

## 新增机型

无需改动代码:放入一份 `config/<robot>/`(controllers.yaml + onnx)与 `models/<robot>_description/`。
模板在 `config/_template/`,步骤见 [docs/ADD_ROBOT.md](docs/ADD_ROBOT.md)。

## 依赖

系统 **ROS 2 Humble** + pip(`requirements.txt`,由 `setup.sh` 统一处理)。无需任何外部工作空间。

## 故障排查

- **跨主机无法发现真机**:`env.sh` 设置了 `ROS_LOCALHOST_ONLY=1`(仅回环)。机载单元位于另一台主机时,注释掉该行(改用独立的 `ROS_DOMAIN_ID`),否则发现过程会静默失败。

## 已知限制

- 站起流程不含自翻身(机身翻覆时需人工扶正)。
- Webots 后端 d1 站立有 sim2sim 差异,**策略验证以 Mujoco 为准**。
- 训练→部署配置仍需手工抄写。
