# litearm-lerobot 开发指南

如何在本仓上干活：LeRobot `Robot` 契约、配置面、伺服环，以及那些不能想当然的失败
方向。写给要扩展或调试这个驱动的人。

```text
LeRobot Robot API ──→ LiteArmRobot ──→ litearm.Arm ──→ USB CDC ──→ STM32 ──→ motors
```

这条路径上**没有 litearm-server，也没有 zenoh**。`litearm.Arm` 直接打开机械臂的
USB CDC 接口（`1d50:606f`）。

---

## 1. 环境要求与安装

| 项目 | 要求 |
|---|---|
| Python | 3.10+ |
| LeRobot | `lerobot>=0.3.0,<0.5` |
| 基础 SDK | `litearm-python>=2.1.0`（不在 PyPI 上——从它的 git 仓库安装） |
| 固件 | `Litearm1.5.0` 或更新，且带 `CMD_JOINT_FOLLOW`（`0x08`） |
| 硬件 | 机械臂经 USB CDC（`1d50:606f`）连到本机 |

```bash
pip install litearm-lerobot          # 发布安装
pip install -e .                     # 开发安装
```

运行测试（无需硬件 —— litearm SDK 已被 fake）：

```bash
python -m pytest tests/ -q
```

## 2. 配置

`LiteArmRobotConfig(RobotConfig)` 以 `litearm` 之名注册进 LeRobot 配置注册表。
全部字段：

| 字段 | 默认值 | 说明 |
|---|---|---|
| `port` | `None` | CDC 端口。`None` = 按 VID:PID `1d50:606f` 自动找。别写死 `/dev/ttyACM0`：端口编号会在重启之间互换。 |
| `move_timeout` | `15.0` | 单次阻塞 `movej` 的超时（秒）。`movej` 到位即提前返回，所以这只是上限。 |
| `num_joints` | `7` | 关节数，连接后与 `arm.n` 对账。不符会在任何运动之前就抛。1J 台架板报 `1`。 |
| `servo_hz` | `250.0` | 伺服环节拍（Hz）。它直接进 `slew_target` 的 `dt`，应接近真实周期。⚠ 取值**偏高**不会过冲：掉帧时循环会重锚，于是参考每个真实秒推进得更少、臂比预期**更慢**。这个错法方向是安全的。 |
| `actuator` | `"joint_follow"` | 固件通道：`"joint_follow"`（`0x08`）或 `"move_js"`（`0x03`）。见[执行器选型](#执行器选型)。 |
| `k_p` | `None` | 逐关节位置增益。`None` = 内置表。`actuator="move_js"` 时被忽略，并在 `connect()` 时告警。 |
| `k_d` | `None` | 逐关节阻尼增益。`None` = 内置表。忽略方式同上。 |
| `speed_limit` | `None` | 逐关节速度上限（rad/s）。`None` = 保守的通用档。 |
| `accel_limit` | `None` | 逐关节加速度上限（rad/s²）。`None` = 保守的通用档。 |
| `engage_sec` | `0.3` | 接管时先以低增益托住实测位姿的秒数。这一段跑在**上位机**的伺服循环里，不是固件。 |
| `limit_margin` | `0.01` | 软限位内缩量（rad）。**不能调大**——见[不要](#不要)。 |
| `enable_on_connect` | `True` | `connect()` 时调 `enable()`。`True`（默认）会起伺服环，失败即抛并关上串口。`False` 是只读会话：不起伺服环，`get_observation()` 照常，`send_action()` 会抛。 |
| `disable_on_disconnect` | `False` | 改调 `disable()`，而不是用零位移 `movej` 把臂交回。臂会卸力。 |

`validate()` 会拒掉未知的 `actuator`、非正的 `num_joints` / `servo_hz`、负的
`limit_margin`，以及任何长度不等于 `num_joints` 的逐轴列表。它在 `connect()` 最前面跑。

## 3. Robot 接口

实现 `lerobot.robots.robot.Robot` 的抽象成员：

| 成员 | litearm-lerobot |
|---|---|
| `name` | `"litearm"` |
| `observation_features` | `{"observation.state": (7,)}` |
| `action_features` | `{"action": (7,)}` |
| `is_connected` | 底层 `litearm.Arm` 是否打开 |
| `connect()` | 打开 CDC 链路、核对 `arm.n`、读软限位、按配置 `enable()`、启动伺服环 |
| `is_calibrated` | 恒为 `True`（机械臂使用绝对编码器） |
| `calibrate()` | 空操作（无需标定） |
| `configure()` | 空操作（增益与限位都在固件里） |
| `get_observation()` | `{"observation.state": [q0..q6]}` |
| `send_action(action)` | 钳位、登记伺服环目标、回传钳位后的向量 |
| `disconnect()` | 停止伺服环、把臂交回、关闭链路 |

### send_action 与伺服环

`send_action` 是**非阻塞**的：它先把目标钳进从固件读到的软限位，把结果登记为伺服环的
目标，然后立即返回。这次调用里没有任何一步在等运动完成。

后台 `ServoLoop` 线程按 `servo_hz` 唤醒，每个节拍做三件事：

1. 用 `slew_target` 把限速参考朝目标推进一步——它施加速度/加速度上限，并按制动距离
   `v²/(2a)` 提前减速。
2. 算出逐关节的速度参考。
3. 调**唯一**的下发点 `_send()`，转成 `arm.joint_follow(q, dq, k_p, k_d)`。

`_send()` 是全包唯一跟机械臂运动通道打交道的地方。换执行器只改这一个函数加一个配置值，
除此之外没有别的代码知道当前用的是哪条通道。

其余都在固件里：参考跟踪器、软限位墙力、重力前馈（`G(q_meas)`）。PC 侧从不下发力矩。

伺服环是通往电机的唯一路径，所以它死了就必须响：环里抛出的任何异常都会先触发一次
受控的 `hold_at_current()` 接管，然后把异常对象存进 `ServoLoop.error`，并由下一次
`get_observation()` / `send_action()` **带着原异常显式抛**。`ServoLoop.error` 是这个
不变量的具名载体——**不要**把"线程死了就返回旧数据"当成修法。

### 执行器选型

两条通道都已实现，默认是 `joint_follow`。这个取舍是真实存在的，而且离线判不了，所以
`actuator` 才是一等配置字段。

`joint_follow`（`0x08`）：

- 增益**随帧下发**（固件钳在 `kp ≤ 500`、`kd ≤ 5.0`），所以配的就是电机实际拿到的。
- 参考 slew 用固件的 `s_jf_vel_max` 表，但本仓默认的保守 `speed_limit` 才是真正的约束。
- 固件对这条通道**豁免**位置锁存与超速锁存，所以坏掉的实测值拿不到第二意见。入口钳位与
  软限位墙仍然把目标框住；硬件类判据（温度、电机错误、反馈陈旧）照旧生效，出口是**整臂
  急停**。
- 豁免在 `hold` 一置时就结束，所以它撑不过一次看门狗触发。

`move_js`（`0x03`）：

- 增益来自固件出厂表，不是你的配置：J1~J4 的 `mit_kp` 是 400，是调优值 200 的两倍，
  弹簧更硬。你给的 `k_p` / `k_d` 会被忽略，`connect()` 会告警说明。
- 固件还加了一项 τ 域的 `kd_extra`（J1~J4 为 6.0），`0x08` 拿不到它，所以这几轴的有效
  阻尼高于表上读到的值。
- `dq` 决定的是参考 slew 的**速率**，不只是前馈。`dq` 为 0 会把参考冻住：臂纹丝不动，
  而且不报任何错。`slew_target` 在运动时不会产出全零 `dq`，有测试盯着这一条。
- 位置锁存与超速锁存在这条通道上**生效**。

哪条通道更容易误判，取决于轨迹和负载，所以改默认值之前先做一次真机 A/B：同一段轨迹、
两种 `actuator` 各跑一遍，比 `joint_fault`、故障位和跟踪误差 rms。

### 不要

下面五条是读者最容易想当然的失败方向。完整的说明与原因见
[运动与安全](../README.zh-CN.md#运动与安全)。简版：

- **不要**以为进程一死机械臂还会被托住——0.1 s 后它 fail-soft，并在自重下缓慢下垂。
- **不要**在退出时调 `disable()`——臂会下落，并可能锁存成只能靠人清掉的故障。
- **不要**把 `limit_margin` 调大——取 `0.02` 会静默抹掉 J4 整个正半轴。
- **不要**写死 `/dev/ttyACM0`——两个 CDC 口的编号会在重启之间互换。
- **要**把会话放在 `with` 里跑，这样 `disconnect()` 一定会执行。

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

`tests/` 把 litearm SDK 整个 fake 掉（`conftest.py` 里的 `FakeArm`），因此装了
`lerobot` + `litearm-python` 的任何机器都能跑全部测试。覆盖：配置类型与校验、特征形状、
connect/observe、伺服环（收敛、`dq` 非零、节拍耗时、异常路径）、维度校验、断开生命
周期、`register()` 幂等。

`tests/test_sdk_contract.py` 不一样：它的**全部期望值都取自真 litearm SDK**，不从本仓
实现取。它真的构造一次 `litearm.Arm`、也真的去看真签名，所以它是**判据与实现同源**的
护栏——那种"全绿只因为它自己同意自己"的测试。你改动本仓对 SDK 的调用方式时，必须同时
改这个文件。
