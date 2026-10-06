# 示例

给首次上手臂、或要验证驱动改动的人看的可运行示例。每个示例都是普通脚本，装好包后
在仓库根目录运行（见 [../README.zh-CN.md](../README.zh-CN.md)）。

> **警告：**示例 02、03 会**驱动真实机械臂**。首次运行请用小速度，并把手放在急停旁边。

## 前置

机械臂通过 USB CDC **直连本机**——这条路径上没有 litearm-server。端口按
VID:PID `1d50:606f` 自动识别；要指定就用 `--port /dev/ttyACM0`。

```bash
pip install -e .
```

## 01 —— 读取观测（只读）

```bash
python examples/01_read_observation.py --count 5
```

打印观测特征与最新关节位置。

## 02 —— 下发正弦动作（会动）

```bash
python examples/02_send_action.py \
  --duration 10 --amplitude 0.05
```

通过 `robot.send_action()` 驱动小幅正弦轨迹。首次运行请把 `--amplitude` 调小。

> **不要**把绝对关节角当成轨迹传进去 —— 那会在第一步就把机械臂甩过整个行程。
> 示例 02 与任何策略回放都应从**启动时读到的位姿**偏移。

## 03 —— 录制 LeRobotDataset 回合

```bash
python examples/03_record_dataset.py \
  --repo-id litearm_demo --root data \
  --episodes 1 --episode-length 50 --task push
```

在移动机械臂（零重力、脚本或遥操作）的同时录制 `observation.state` 与
`action` 帧。数据集产出在 `data/litearm_demo/`，可直接供 LeRobot 训练脚本使用。

English: [README.md](README.md)
