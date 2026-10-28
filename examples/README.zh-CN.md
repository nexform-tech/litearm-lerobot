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

## 03 —— 录制状态回放诊断数据

该示例没有独立动作源，不能生成策略训练数据。除非显式确认诊断用途，否则脚本不会
打开机械臂。

```bash
python examples/03_record_dataset.py \
  --allow-state-replay-demo \
  --repo-id litearm_state_replay_demo --root diagnostic_data \
  --episodes 1 --episode-length 50 --task bench_check
```

该示例把每个实测位姿作为保持目标发回，只记录 `observation.state` 和
`diagnostic.state_replay_target`，并刻意不创建 `action` feature。**不要**把输出用于
策略训练。

English: [README.md](README.md)
