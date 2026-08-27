# litearm-lerobot 开发指南与 API 参考

`litearm-lerobot` 通过标准的 LeRobot `Robot` 接口暴露 LiteArm 机械臂，使
LeRobot 生态（策略、数据集管线、评测脚本）无需改动即可驱动机械臂。

```text
LeRobot Robot API ──→ LiteArmRobot ──→ litearm.Arm ──→ litearm-server ──→ arm/CAN
```

---

## 1. 环境要求与安装

| 项目 | 要求 |
|---|---|
| Python | 3.10+ |
| LeRobot | `lerobot>=0.3.0,<0.5` |
| 基础 SDK | `litearm-python>=0.1.0` |

```bash
pip install litearm-lerobot          # 发布安装
pip install -e .                     # 开发安装
```

运行测试（无需硬件 —— `litearm.Arm` 已被 fake）：

```bash
python -m pytest tests/ -q
```

## 2. 配置

`LiteArmRobotConfig(RobotConfig)` 以 `litearm` 之名注册进 LeRobot 配置注册表。
全部字段：

| 字段 | 默认值 | 说明 |
|---|---|---|
| `endpoint` | `tcp/127.0.0.1:7447` | `litearm-server` 的 Zenoh 地址 |
| `arm_id` | `armA` | 服务端注册的机械臂 id |
| `query_timeout` | `None` | 单次调用超时（秒）；`None` 用 SDK 默认 |
| `num_joints` | `7` | 关节数（决定特征形状） |
| `control_frequency_hz` | `30.0` | `send_action` 使用的 commander 循环频率 |
| `movej_speed` | `0.5` | commander 使用的 `movej` 速度（0..1） |
| `settle_s` | `0.2` | 运动结束后等待时间 |
| `use_commander` | `True` | 使用非阻塞后台 commander |
| `enable_on_connect` | `True` | 连接时调用 `enable()` |
| `disable_on_disconnect` | `False` | 断开时调用 `disable()` 而非 `hold()` |

## 3. Robot 接口

实现 `lerobot.robots.robot.Robot` 的抽象成员：

| 成员 | litearm-lerobot |
|---|---|
| `name` | `"litearm"` |
| `observation_features` | `{"observation.state": (7,)}` |
| `action_features` | `{"action": (7,)}` |
| `is_connected` | 底层 `litearm.Arm` 是否打开 |
| `connect()` | 打开机械臂、预热状态、按配置 `enable()`、启动 commander |
| `is_calibrated` | 恒为 `True`（机械臂使用绝对编码器） |
| `calibrate()` | 空操作（无需标定） |
| `configure()` | 空操作 |
| `get_observation()` | `{"observation.state": [q0..q6]}` |
| `send_action(action)` | 非阻塞 commander 目标，或阻塞 `movej` |
| `disconnect()` | 停止 commander，`hold()` + `close()` |

### send_action —— 阻塞 vs commander

`litearm.Arm.movej` 是阻塞 RPC（运动完成才返回），直接调用会卡住策略循环。两种模式：

- `use_commander=True`（默认）：后台线程按 `control_frequency_hz` 持续对最新目标
  执行 `movej`；`send_action` 只更新目标并立即返回。重复目标会被跳过。
- `use_commander=False`：`send_action` 直接调用 `movej`，阻塞直到运动完成。

## 4. 数据集录制

`LeRobotDataset` 要求**完整 feature 描述**（不是 `robot.observation_features`
返回的简写）：

```python
features = {
    "observation.state": {"dtype": "float32", "shape": (7,)},
    "action":            {"dtype": "float32", "shape": (7,)},
}
```

`LeRobotDataset.create(repo_id, fps, root, ...)` 会把数据集直接存在
`<root>`，且**拒绝覆盖**已存在的目录——请换新的 `repo_id` 或删目录
重录。每一帧必须含 `"task"` 键且为 numpy float32 数组：

```python
dataset.add_frame({
    "observation.state": np.asarray(obs["observation.state"], dtype=np.float32),
    "action": np.asarray(action, dtype=np.float32),
    "task": "push",
})
```

完整循环为 `create_episode_buffer()` → `add_frame(...)` × N →
`save_episode()`。见 [examples/03_record_dataset.py](../examples/03_record_dataset.py)。

## 5. LeRobot CLI 集成（可选）

LeRobot 的 `make_robot_from_config` 是内置机器人名的 if/elif 链，`litearm`
不在其中，需加一个小 monkey-patch 才能进入。`litearm_lerobot.utils.register()`
可为它补上 `litearm` 分支：

```python
from litearm_lerobot.utils import register
register()            # 幂等；程序启动时调用一次
```

程序化使用（主要路径，见快速开始）无需 patch。

## 6. 测试

`tests/` 用 `FakeArm`（在 `conftest.py` 中）fake 掉 `litearm.Arm`，因此装了
`lerobot` + `litearm-python` 的任何机器都能跑全部测试。覆盖：配置类型、特征形状、
connect/observe、commander vs 阻塞 `send_action`、维度校验、断开生命周期、
`register()` 幂等。
