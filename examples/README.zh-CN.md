# 示例

每个示例都是普通脚本，装好包后在仓库根目录运行（见 [../README.zh-CN.md](../README.zh-CN.md)）。

> ⚠️ 示例 02、03 会**驱动真实机械臂**。首次运行请用小速度，并把手放在急停旁边。

## 前置

```bash
pip install -e .
```

## 01 —— 读取观测（只读）

```bash
python examples/01_read_observation.py \
  --endpoint tcp/192.168.31.237:7447 --count 5
```

打印观测特征与最新关节位置。

## 02 —— 下发正弦动作（会动）

```bash
python examples/02_send_action.py \
  --endpoint tcp/192.168.31.237:7447 \
  --duration 10 --amplitude 0.05 --speed 0.3
```

通过 `robot.send_action()` 驱动小幅正弦轨迹。首次运行请把 `--amplitude` 调小。

## 03 —— 录制 LeRobotDataset 回合

```bash
python examples/03_record_dataset.py \
  --endpoint tcp/192.168.31.237:7447 \
  --repo-id litearm_demo --root data \
  --episodes 1 --episode-length 50 --task push
```

在移动机械臂（零重力、脚本或遥操作）的同时录制 `observation.state` 与
`action` 帧。数据集产出在 `data/litearm_demo/`，可直接供 LeRobot 训练脚本使用。

English: [README.md](README.md)
