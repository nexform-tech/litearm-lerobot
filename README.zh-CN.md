# litearm-lerobot

`litearm-lerobot` 是 **LiteArm** 机械臂的 [LeRobot](https://github.com/huggingface/lerobot)
`Robot` 适配器，供在机械臂上录制数据集或运行策略的工程师阅读。

通过 LeRobot 标准的 `Robot` 接口使用 LiteArm：读取观测、下发动作、录制
`LeRobotDataset` 回合用于策略训练——LeRobot 生态工具无需改动。

```text
LeRobot Robot API ──→ LiteArmRobot ──→ litearm.Arm ──→ USB CDC ──→ STM32 ──→ motors
```

> 完整开发指南与 API 参考：[docs/DEVELOPER_GUIDE.zh-CN.md](docs/DEVELOPER_GUIDE.zh-CN.md)
> · English: [README.md](README.md)

---

## 目录

- [概述与架构](#概述与架构)
- [环境要求](#环境要求)
- [安装](#安装)
- [快速开始](#快速开始)
- [配置](#配置)
- [运动与安全](#运动与安全)
- [观测 / 动作循环](#观测--动作循环)
- [录制数据集](#录制数据集)
- [LeRobot CLI 集成](#lerobot-cli-集成)
- [示例](#示例)
- [常见问题排查](#常见问题排查)
- [包结构](#包结构)
- [文档与许可证](#文档与许可证)

---

## 概述与架构

`litearm-lerobot` 基于 [litearm-python](https://pypi.org/project/litearm-python) 的
**USB CDC 直连**客户端（`litearm.Arm`）实现 `lerobot.robots.robot.Robot` 抽象接口。
这条路径上**没有 litearm-server，也没有 zenoh**。任何能驱动 LeRobot 机器人的东西
——策略循环、数据集管线、评测脚本——都能原样驱动 LiteArm。

| 关注点 | 实现 |
|---|---|
| 机器人类型名 | `"litearm"`（注册进 LeRobot 配置注册表） |
| 观测 | `{"observation.state": [q0..q6]}` —— 7 个绝对关节位置 |
| 动作 | `{"action": [q0..q6]}` —— 7 个关节位置目标 |
| 标定 | 空操作——机械臂上报绝对编码器位置，无需找零 |
| 运动 | 后台伺服环下发 `arm.joint_follow (0x08)` |
| 后端 | `litearm.Arm` → USB CDC（`1d50:606f`）→ STM32 → 电机 |

LiteArm 从机械臂控制器直接上报**绝对**关节位置，因此 `calibrate()` 是空操作，
`is_calibrated` 恒为 `True`。

## 环境要求

| 项目 | 要求 |
|---|---|
| Python | 3.10+ |
| LeRobot | `lerobot>=0.3.0,<0.5` |
| 基础 SDK | `litearm-python>=2.1.0`（不在 PyPI 上——从它的 git 仓库安装） |
| 固件 | `Litearm1.5.0` 或更新，且带 `CMD_JOINT_FOLLOW`（`0x08`） |
| 硬件 | 机械臂经 USB CDC（`1d50:606f`）连到本机 |

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

# port=None 表示按 VID:PID 1d50:606f 自动找 CDC 口。
with LiteArmRobot(LiteArmRobotConfig(port=None)) as robot:
    obs = robot.get_observation()             # {"observation.state": [q0..q6]}
    print(robot.observation_features)         # {"observation.state": (7,)}

    q = list(obs["observation.state"])        # 先读当前位姿
    q[0] += 0.05                              # 再在它上面偏移
    robot.send_action({"action": q})          # 非阻塞
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
| `port` | `None` | CDC 端口。`None` = 按 VID:PID `1d50:606f` 自动找。别写死 `/dev/ttyACM0`。 |
| `move_timeout` | `15.0` | 单次阻塞 `movej` 的超时（秒）。`movej` 到位即提前返回，这只是上限。 |
| `num_joints` | `7` | 关节数。连接后与 `arm.n` 对账，不符即抛。1J 台架板报 `1`。 |
| `servo_hz` | `250.0` | 伺服环节拍（Hz）。它直接进 `slew_target` 的 `dt`，应接近真实周期。⚠ 取值**偏高**不会过冲：掉帧时循环会重锚，于是参考每个真实秒推进得更少、臂比预期**更慢**。这个错法方向是安全的。 |
| `actuator` | `"joint_follow"` | 下发通道：`"joint_follow"`（`0x08`，默认）或 `"move_js"`（`0x03`）。 |
| `k_p` | `None` | 逐关节位置增益。`None` = 内置表。`actuator="move_js"` 时被忽略。 |
| `k_d` | `None` | 逐关节阻尼增益。`None` = 内置表。`actuator="move_js"` 时被忽略。 |
| `speed_limit` | `None` | 逐关节速度上限（rad/s）。`None` = 保守的通用档。 |
| `accel_limit` | `None` | 逐关节加速度上限（rad/s²）。`None` = 保守的通用档。 |
| `engage_sec` | `0.3` | 接管时先以低增益托住实测位姿的秒数。这一段跑在**上位机**的伺服循环里，不是固件。 |
| `limit_margin` | `0.01` | 软限位内缩量（rad）。**不能调大**——见[运动与安全](#运动与安全)。 |
| `enable_on_connect` | `True` | `connect()` 时调 `enable()`。`True`（默认）会起伺服环，失败即抛。`False` 给的是**只读会话**：电机不上电，`get_observation()` 照常读固件的被动状态流，而 `send_action()` 会抛——固件会拒绝在未使能的臂上执行伺服帧。 |
| `disable_on_disconnect` | `False` | 改调 `disable()`，而不是用零位移 `movej` 把臂交回。臂会卸力。 |

## 运动与安全

`send_action` 是**非阻塞**的：它先把目标钳进从固件读到的软限位，再交给后台伺服环；
伺服环按 `servo_hz` 节拍持续下发 `arm.joint_follow`（`0x08`）。伺服环、软限位墙与重力
前馈**三者都在固件里**，PC 侧只生成限速参考。

**不要**以为进程一死机械臂还会被托住。进程被杀，伺服流就断；固件的 0.1 s 命令看门狗
随即让机械臂 fail-soft——卸力并在自重下**缓慢下垂**。这是连续伺服通道的固有性质，
不是缺陷。请保持工作区清空，长时间任务边上要有人。

**不要**在退出时调 `disable()`。失能会让机械臂在自重下落，可能漂出软限位并锁存成
只能靠人把关节推回去的故障。`disconnect()` 用零位移 `movej` 把机械臂交回固件，臂保持
刚性。只有当机械臂已经落在支撑面上时，才把 `disable_on_disconnect` 设为 `True`。

**不要**把 `limit_margin` 调大。J4 的固件上端只有 `+0.017547 rad`（1°）。取 `0.02`
会把 J4 的上界压到 `-0.0025`，**静默**抹掉它整个正半轴——而且不报任何错。

**不要**写死 `/dev/ttyACM0`。本机两个 CDC 口的编号会在重启之间互换；让 `port=None`，
由 SDK 按 `VID:PID 1d50:606f` 去匹配。

**要**把会话放在 `with` 里跑，这样 `disconnect()` 一定会执行。

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

`send_action` 先校验输入（必须含 `"action"` 键且正好 `num_joints` 个值），然后做三件事：

- 把目标钳进从固件读到的软限位。被钳的轴会带轴名和原值记进日志。
- 把结果登记为伺服环的目标。这是 `send_action` 对机械臂做的**唯一**动作，且立即返回。
- 回传钳位后的向量，调用方据此记录"实际下发了什么"。

后台环按 `servo_hz` 唤醒，用 `slew_target` 推进限速参考，经**唯一**的下发点送到
`arm.joint_follow`。因为这个环是通往电机的唯一路径，它死了就不会被瞒住：`send_action()`
与 `get_observation()` 会带着环里的原始错误**显式抛**，而不是照旧返回旧数据。

正因为有这个环，LeRobot 策略循环才不会被卡住：没有任何一次调用会阻塞等运动完成。

## 录制数据集

`LeRobotDataset` 要求**完整 feature 描述**（不是 `observation_features` 返回的简写）：

```python
from pathlib import Path
import numpy as np
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from litearm_lerobot import LiteArmRobot, LiteArmRobotConfig

robot = LiteArmRobot(LiteArmRobotConfig())
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

可运行命令（只有自动识别选错设备时才需要传 `--port`）：

```bash
python examples/01_read_observation.py --count 5
python examples/02_send_action.py --duration 10 --amplitude 0.05
python examples/03_record_dataset.py \
  --repo-id litearm_demo --root data --episodes 1 --episode-length 50 --task push
```

示例 02、03 会**驱动真实机械臂**。首次运行请把 `--amplitude` 调小、清空工作区，并把
手放在急停旁边。示例 02 是**从启动时读到的位姿偏移**出去的；**不要**把它改成下发绝对
关节角——那会在第一步就把机械臂甩过整个行程。每个示例的参数说明见
[examples/README.zh-CN.md](examples/README.zh-CN.md)。

## 常见问题排查

| 现象 | 可能原因 / 解决 |
|---|---|
| `ModuleNotFoundError: No module named 'lerobot'` | 本包装进了与 LeRobot 不同的环境。执行 `conda run -n lerobot pip install -e .`。 |
| `ModuleNotFoundError: No module named 'litearm'` | 运行脚本的 python 缺 `litearm-python`。从它的 git 仓库安装。 |
| `litearm.TransportError: 未找到 STM32 CDC (VID:PID 1d50:606f), 请用 --port 指定` | 没有匹配的 CDC 设备。接上机械臂，或显式传 `port=`。 |
| `litearm.TransportError: 打开串口 /dev/ttyACMn 失败: ... Could not exclusively lock port` | 端口被别的进程占着。停掉它。**不要**让两个进程用同一个 CDC 口：Linux 上它们会分吃同一条字节流，两边一起坏。 |
| `RuntimeError: LiteArmRobot is not connected` | 在 `get_observation()`/`send_action()` 前先调用 `robot.connect()`。 |
| `RuntimeError: No robot state received yet` | 链路已通但状态帧还没到。稍后重试。 |
| `RuntimeError: 伺服环已停（joint_follow 下发失败）：...` | 伺服环已死，机械臂不再受控。消息里带着原始错误。重连并检查链路。 |
| `ValueError: action must have 7 joints, got N` | 动作向量必须正好 `num_joints` 个值。 |
| `ValueError: action dict must contain an 'action' key` | `send_action` 期望 `{"action": [..]}`。 |
| `ValueError: 关节数不符：固件报 1，配置是 7` | 固件上报的关节数与 `num_joints` 不一致。1J 台架板报 `1`，7J 整臂报 `7`。 |
| 连接时 `enable()` 失败 | 直连不可能是只读，因此这意味着机械臂没有应答。检查供电与 CDC 线缆。 |
| `<root>` 数据集已存在 | `LeRobotDataset.create` 拒绝覆盖。换新的 `--repo-id` 或删除目录。 |

## 包结构

```text
litearm-lerobot/
├── pyproject.toml             包元数据 + pytest 配置
├── src/litearm_lerobot/
│   ├── __init__.py            公共 API（LiteArmRobot、LiteArmRobotConfig、register）
│   ├── config.py              LiteArmRobotConfig（LeRobot RobotConfig 子类）
│   ├── robot.py               LiteArmRobot —— LeRobot Robot 的实现
│   ├── servo.py               ServoLoop（后台线程）与唯一的 _send()
│   ├── safety.py              软限位读取 + 钳位 + 限速参考
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
  [docs/DEVELOPER_GUIDE.zh-CN.md](docs/DEVELOPER_GUIDE.zh-CN.md)
- English developer guide: [docs/DEVELOPER_GUIDE.md](docs/DEVELOPER_GUIDE.md)
- English: [README.md](README.md)
- 许可证：见 `pyproject.toml`（`LicenseRef-Proprietary`）。
