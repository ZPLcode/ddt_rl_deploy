# ddt_rl_deploy

DDT 机器人强化学习策略部署。自包含仓库,sim2sim(Mujoco / Webots)与 sim2real(真机)共用一套代码——上层 App 只讲 ROS 2 topic,换的只是"对面挂谁"。

> [!NOTE]
> 上机前先在 Mujoco 验证。退出 / 断连一律降阻尼:机载单元会永久保持最后一条指令,高 kp 下直接退出会绷死机器人。

自带策略:**d1**(`rl_flat` / `rl_flat_lab` / `rl_rough_lab`)、**d1cargo_out**(`rl_height_cargo_out`,带货箱变高度爬楼)。d1h / tita 模型就绪,补 config + onnx 即用。

## Quickstart

```bash
# 前置:系统装好 ROS 2 Humble
./scripts/setup.sh                          # pip 依赖 + colcon build

# sim2sim(三个终端):
./scripts/run_sim.sh                        # Mujoco 仿真(--backend webots 换后端,--robot 换机型)
./scripts/run_policy.sh rl_flat_lab d1      # 策略(--list 看可选)
./scripts/run_teleop.sh                     # 遥控:wsad 移动 · qe 平移 · rf 升降 · 空格停 · x 退出

# sim2real:遥控器进 08 SDK Mode 后只跑后两条,对面换成机载单元,代码不变
```

## 结构

```
deploy/       policy_engine.py(纯函数大脑)+ rl_inference.py(ROS 壳)。数据流见 DATAFLOW.md
sim/          simbase.py(后端共享核心)+ mujoco_sim.py + webots/。topic 契约见 BACKEND.md
config/       controllers.yaml + *.onnx(按机型);_template/ 是加机型的骨架
models/       机器人模型(URDF 为主 + mujoco MJCF 派生):d1 / d1h / tita / d1cargo_out
scripts/      setup · run_sim · run_policy · run_teleop
src/ddt_msgs/ 消息契约,唯一要 build 的包
```

## 加一个新机型

不用改代码:丢一份 `config/<robot>/`(controllers.yaml + onnx)+ `models/<robot>_description/`。
模板在 `config/_template/`,步骤见 [docs/ADD_ROBOT.md](docs/ADD_ROBOT.md)。

## 依赖

系统 **ROS 2 Humble** + pip(`requirements.txt`,`setup.sh` 一键处理)。不需要任何外部工作空间。

## 已知限制

- 站起不含翻身(四脚朝天需人工扶正)。
- Webots 后端 d1 站立有 roll 残余晃动(sim2sim 接触差异),**策略验证以 Mujoco 为准**。
- 训练→部署配置仍手抄,自动导出 manifest 待做。
