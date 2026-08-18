# litearm-lerobot

[LeRobot](https://github.com/huggingface/lerobot) `Robot` 适配器，用于 LiteArm 机械臂。
通过 LeRobot 标准的 `Robot` 接口使用 LiteArm：读取观测、下发动作、录制
`LeRobotDataset` 回合，用于策略训练。

## 特性

- 🦾 **完整的 LeRobot `Robot` 接口** —— observation/action 特征、connect、calibrate、get_observation、send_action、disconnect
- 🎯 **7 自由度关节控制** —— `observation.state` (7,) 与 `action` (7,)
- 🎥 **数据集录制** —— 直接产出 LeRobot 训练可用的 `LeRobotDataset` 回合
- 🔄 **`send_action` 非阻塞** —— 后台 commander 线程按设定频率持续驱动机械臂
- 🧩 **客户端无 numpy / 无硬件依赖** —— 基于 [litearm-python](../litearm-python)

> 📖 完整开发指南与 API 参考：[docs/DEVELOPER_GUIDE.md](docs/DEVELOPER_GUIDE.md)
> · English: [README.md](README.md)

## 环境要求

| 项目 | 要求 |
|---|---|
| Python | 3.10+ |
| LeRobot | `lerobot>=0.3.0,<0.5` |
| 基础 SDK | [litearm-python](https://pypi.org/project/litearm-python) `>=0.1.0` |

## 安装

```bash
pip install litearm-lerobot            # 发布安装
pip install -e .                       # 开发安装（在源码目录）
```

> 若使用 `lerobot` conda/venv 环境，请装进该环境，例如
> `conda run -n lerobot pip install -e .`。

## 快速开始

```python
from litearm_lerobot import LiteArmRobot, LiteArmRobotConfig

robot = LiteArmRobot(LiteArmRobotConfig(
    endpoint="tcp/192.168.31.237:7447",   # litearm-server 的地址
    arm_id="armA",
))
robot.connect()

obs = robot.get_observation()             # {"observation.state": [q0..q6]}
print(robot.observation_features)         # {"observation.state": (7,)}

robot.send_action({"action": [0.0] * 7})  # 回到零位
robot.disconnect()
```

`LiteArmRobot` 以 **`litearm`** 之名注册进 LeRobot 的配置注册表
（`RobotConfig.register_subclass("litearm")`），因此也可用 YAML 配置或
`RobotConfig` 实例化。

## 录制数据集

```python
from pathlib import Path
import numpy as np
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from litearm_lerobot import LiteArmRobot, LiteArmRobotConfig

robot = LiteArmRobot(LiteArmRobotConfig(endpoint="tcp/192.168.31.237:7447"))
robot.connect()

features = {
    "observation.state": {"dtype": "float32", "shape": (robot.config.num_joints,)},
    "action": {"dtype": "float32", "shape": (robot.config.num_joints,)},
}
dataset = LeRobotDataset.create(
    repo_id="litearm_demo", fps=30,
    root=Path("data") / "litearm_demo",          # 该目录必须不存在
    robot_type=robot.name, features=features, use_videos=False,
)
dataset.episode_buffer = dataset.create_episode_buffer()
obs = robot.get_observation()
dataset.add_frame({
    "observation.state": np.asarray(obs["observation.state"], dtype=np.float32),
    "action": np.asarray(obs["observation.state"], dtype=np.float32),
    "task": "push",
})
dataset.save_episode()
robot.disconnect()
```

## 示例

| 示例 | 说明 |
|---|---|
| [examples/01_read_observation.py](examples/01_read_observation.py) | 只读：连接并打印观测 |
| [examples/02_send_action.py](examples/02_send_action.py) | 正弦轨迹运动（会动——请握好急停） |
| [examples/03_record_dataset.py](examples/03_record_dataset.py) | 录制遥操作回合到 `LeRobotDataset` |

可运行命令见 [examples/README.zh-CN.md](examples/README.zh-CN.md)。
