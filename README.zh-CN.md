# litearm-lerobot

面向 **LiteArm** 机械臂的 [LeRobot](https://github.com/huggingface/lerobot)
`Robot` 适配器。通过 LeRobot 标准的 `Robot` 接口使用 LiteArm：读取观测、下发
动作、录制 `LeRobotDataset` 回合用于策略训练——LeRobot 生态工具无需改动。

```text
LeRobot Robot API ──→ LiteArmRobot ──→ litearm.Arm ──→ litearm-server ──→ arm/CAN
```

> 📖 完整开发指南与 API 参考：[docs/DEVELOPER_GUIDE.md](docs/DEVELOPER_GUIDE.md)
> · English: [README.md](README.md)

---

## 目录

- [概述与架构](#概述与架构)
- [环境要求](#环境要求)
- [安装](#安装)
- [快速开始](#快速开始)
- [配置](#配置)
- [观测 / 动作循环](#观测--动作循环)
- [录制数据集](#录制数据集)
- [LeRobot CLI 集成](#lerobot-cli-集成)
- [示例](#示例)
- [常见问题排查](#常见问题排查)
- [包结构](#包结构)
- [文档与许可证](#文档与许可证)

---

## 概述与架构

`litearm-lerobot` 基于 [litearm-python](https://pypi.org/project/litearm-python)
的远程 `Arm` 客户端实现 `lerobot.robots.robot.Robot` 抽象接口。任何能驱动
LeRobot 机器人的东西——策略循环、数据集管线、评测脚本——都能原样驱动 LiteArm。

| 关注点 | 实现 |
|---|---|
| 机器人类型名 | `"litearm"`（注册进 LeRobot 配置注册表） |
| 观测 | `{"observation.state": [q0..q6]}` —— 7 个绝对关节位置 |
| 动作 | `{"action": [q0..q6]}` —— 7 个关节位置目标 |
| 标定 | 空操作——机械臂上报绝对编码器位置，无需找零 |
| 运动 | `Arm.movej()`，经非阻塞后台 commander（或阻塞） |
| 后端 | 远程 `litearm.Arm` → **litearm-server** → 硬件/CAN |

LiteArm 从机械臂控制器直接上报**绝对**关节位置，因此 `calibrate()` 是空操作，
`is_calibrated` 恒为 `True`。

## 环境要求

| 项目 | 要求 |
|---|---|
| Python | 3.10+ |
| LeRobot | `lerobot>=0.3.0,<0.5` |
| 基础 SDK | [litearm-python](https://pypi.org/project/litearm-python) `>=0.1.0` |
| 运行时 | 可达的 **litearm-server**（Zenoh endpoint，例如 `tcp/192.168.31.237:7447`） |

> LeRobot 通常装在自己的 conda/venv 环境里。请把本包装进**同一个环境**，让
> `lerobot` 与 `litearm` 一起可导入（见[安装](#安装)）。

## 安装

```bash
pip install litearm-lerobot            # 发布安装
pip install -e .                       # 开发安装（在源码目录）
```

若使用专门的 LeRobot 环境，请装进该环境：

```bash
conda run -n lerobot pip install -e .
```

在任意机器上运行测试（无需硬件——`litearm.Arm` 已被 fake）：

```bash
python -m pytest tests/ -q
```

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

robot.send_action({"action": [0.0] * 7})  # 下发零位关节位姿
robot.disconnect()
```

`LiteArmRobot` 以 **`litearm`** 之名注册进 LeRobot 的配置注册表
（`RobotConfig.register_subclass("litearm")`），因此也可用 YAML 配置或
`RobotConfig` 实例化：

```python
from litearm_lerobot import LiteArmRobot
robot = LiteArmRobot(LiteArmRobotConfig.from_yaml("config.yaml"))
```

## 配置

`LiteArmRobotConfig(RobotConfig)` —— 全部字段：

| 字段 | 默认值 | 说明 |
|---|---|---|
| `endpoint` | `tcp/127.0.0.1:7447` | litearm-server 的 Zenoh endpoint |
| `arm_id` | `armA` | 服务器上注册的机械臂 id |
| `query_timeout` | `None` | 单次 RPC 超时（秒）；`None` 用 SDK 默认 |
| `num_joints` | `7` | 关节数（决定观测/动作特征形状） |
| `control_frequency_hz` | `30.0` | `send_action` 使用的 commander 循环频率 |
| `movej_speed` | `0.5` | 每次下发目标使用的 `movej` 速度（0..1） |
| `settle_s` | `0.2` | 每次 `movej` 到位后等待时间 |
| `use_commander` | `True` | 非阻塞后台 commander（见下） |
| `enable_on_connect` | `True` | `connect()` 时调用 `enable()`（上电并保持位姿） |
| `disable_on_disconnect` | `False` | `disconnect()` 时调用 `disable()` 而非 `hold()`（机械臂卸力） |

## 观测 / 动作循环

两个向量都是机械臂控制器上报的**七个绝对关节位置**。观测/动作特征为：

```text
observation_features = {"observation.state": (7,)}
action_features      = {"action": (7,)}
```

```python
while not done:
    obs = robot.get_observation()          # {"observation.state": [q0..q6]}
    action = policy(obs["observation.state"])
    robot.send_action({"action": action})
```

`send_action` 会校验输入（必须含 `"action"` 键且正好 `num_joints` 个值），然后
按以下两种模式之一工作：

- **`use_commander=True`（默认）—— 非阻塞。** 后台线程按 `control_frequency_hz`
  持续用 `movej` 驱动机械臂追向最新目标。`send_action` 只更新目标并立即返回，
  策略/遥操作循环可按自身频率运行。重复目标会被跳过。
- **`use_commander=False` —— 阻塞。** `send_action` 直接调用 `Arm.movej()`，
  阻塞到运动完成。

> `Arm.movej` 是**阻塞** RPC——机械臂到位后才返回。这正是 commander 线程存在
> 的原因；否则 LeRobot 循环每一步都会卡住。

## 录制数据集

`LeRobotDataset` 要求**完整 feature 描述**（不是 `observation_features` 返回的
简写）：

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

要点：

- `LeRobotDataset.create(...)` 会把数据集直接存在 `<root>`，且**拒绝覆盖**
  已存在的目录——请换新的 `repo_id` 或删目录重录。
- 每一帧必须含 `"task"` 键，且每个 feature 均为 numpy **float32** 数组。
- 完整循环为 `create_episode_buffer()` → `add_frame(...)` × N →
  `save_episode()`。

现成的录制脚本见 [examples/03_record_dataset.py](examples/03_record_dataset.py)。

## LeRobot CLI 集成

LeRobot 通过 `lerobot.robots.utils.make_robot_from_config` 构建机器人，这是一个
内置机器人名的 if/elif 链——`litearm` 默认不在其中。
`litearm_lerobot.utils.register()` 会 monkey-patch 该函数，补上 `litearm` 分支：

```python
from litearm_lerobot.utils import register
register()            # 幂等；程序启动时调用一次
```

之后任何 `type` 为 `litearm` 的配置都会解析为 `LiteArmRobot`。程序化使用
（快速开始路径）无需 patch。

## 示例

| 示例 | 说明 |
|---|---|
| [examples/01_read_observation.py](examples/01_read_observation.py) | 只读：连接并打印观测 |
| [examples/02_send_action.py](examples/02_send_action.py) | 正弦轨迹运动（**会动**——请握好急停） |
| [examples/03_record_dataset.py](examples/03_record_dataset.py) | 录制遥操作回合到 `LeRobotDataset` |

可运行命令：

```bash
python examples/01_read_observation.py --endpoint tcp/192.168.31.237:7447 --count 5
python examples/02_send_action.py --endpoint tcp/192.168.31.237:7447 \
  --duration 10 --amplitude 0.05 --speed 0.3
python examples/03_record_dataset.py --endpoint tcp/192.168.31.237:7447 \
  --repo-id litearm_demo --root data --episodes 1 --episode-length 50 --task push
```

> ⚠️ 示例 02、03 会**驱动真实机械臂**。首次运行请把 `--amplitude`/`--speed` 调小，
> 并把手放在急停旁边。每个示例的参数说明见
> [examples/README.md](examples/README.md)。

## 常见问题排查

| 现象 | 可能原因 / 解决 |
|---|---|
| `ModuleNotFoundError: No module named 'lerobot'` | 本包装进了与 LeRobot 不同的环境。执行 `conda run -n lerobot pip install -e .`。 |
| `ModuleNotFoundError: No module named 'litearm'` | 运行节点的 python 缺 `litearm-python`。执行 `pip install litearm-python`。 |
| `RuntimeError: LiteArmRobot is not connected` | 在 `get_observation()`/`send_action()` 前先调用 `robot.connect()`。 |
| `RuntimeError: No robot state received yet` | 服务器尚未广播状态。确认 litearm-server 在运行、`endpoint`/`arm_id` 正确，稍后重试。 |
| `ValueError: action must have 7 joints, got N` | 动作向量必须正好 `num_joints` 个值。 |
| `ValueError: action dict must contain an 'action' key` | `send_action` 期望 `{"action": [..]}`。 |
| 连接时 `enable()` 失败 | 服务器可能只读；连接会继续（仅记录警告日志）。 |
| `<root>` 数据集已存在 | `LeRobotDataset.create` 拒绝覆盖。换新的 `--repo-id` 或删除目录。 |

## 包结构

```text
litearm-lerobot/
├── pyproject.toml             包元数据 + pytest 配置
├── src/litearm_lerobot/
│   ├── __init__.py            公共 API（LiteArmRobot、LiteArmRobotConfig、register）
│   ├── config.py              LiteArmRobotConfig（LeRobot RobotConfig 子类）
│   ├── robot.py               LiteArmRobot + 后台 commander 线程
│   └── utils.py               register()——LeRobot 工厂 monkey-patch
├── examples/
│   ├── 01_read_observation.py
│   ├── 02_send_action.py
│   ├── 03_record_dataset.py
│   └── README.md / README.zh-CN.md
├── tests/                     mock 单元测试（无需硬件）
└── docs/
    ├── DEVELOPER_GUIDE.md
    └── DEVELOPER_GUIDE.zh-CN.md
```

## 文档与许可证

- 开发指南与 API 参考（接口细节、数据集录制、CLI 集成内部实现）：
  [docs/DEVELOPER_GUIDE.md](docs/DEVELOPER_GUIDE.md)
- English: [README.md](README.md)
- 许可证：见 `pyproject.toml`（`LicenseRef-Proprietary`）。
