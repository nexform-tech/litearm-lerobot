# litearm-lerobot 换用 litearm-python 直连后端

这是一份内部设计文档，写给要改或要评审 `litearm-lerobot` 底层驱动的人：它记录把
本仓从「litearm-server / zenoh 远程客户端」换到「litearm-python STM32 直连」的全部
决定、依据与验收判据。

日期：2026-09-29。基线：`main` @ `2a50592`。

## 1. 问题

### 1.1 现状代码对本机真 SDK 已经是坏的

本仓写的是 **server 语义**的旧 `litearm` API，而本机 `import litearm` 现在落在
`/home/llx/litearm-python/src/litearm/`（2.1.0，STM32 直连）。实测：

```console
$ /usr/bin/python3 -c "import litearm; litearm.Arm(endpoint='tcp/127.0.0.1:7447', arm_id='armA')"
TypeError: Arm.__init__() got an unexpected keyword argument 'endpoint'

$ /usr/bin/python3 -c "import litearm; print(hasattr(litearm.Arm, 'hold'))"
False
```

签名（原输出是一行，这里为排版折行）：

```text
(self, port: 'Optional[str]' = None, *, transport_factory: 'Optional[Any]' = None,
 min_firmware: 'tuple' = (1, 5, 0), q_tol: 'float' = 0.03, dq_tol: 'float' = 0.1,
 arrive_frames: 'int' = 3, move_timeout: 'float' = 15.0)
```

`robot.py` 里失效的调用**不止三处**：

| 调用 | 失效方式 |
|---|---|
| `litearm.Arm(endpoint=..., arm_id=..., query_timeout=...)` | `TypeError`（三个关键字都不存在） |
| `arm.get_state()["q"]` | `get_state()` 现在返回 `Msg` 信封，不是 dict |
| `arm.movej(q, speed=..., settle_s=...)` | `settle_s` 不存在 |
| `arm.hold()` | `AttributeError`（方法不存在） |
| `arm.close()` | 不变，这个还在 |

另外 `tests/conftest.py` 的 `FakeArm` 把前四项都按旧形状假造了出来，所以这四处
在测试里全部"通过"。

### 1.2 测试全绿是假的，而且假得有结构

`tests/` 目前 14 条全过，但 `tests/conftest.py` 的 `FakeArm` 是照着**旧 API** 造的：
`get_state()` 返回 dict、`movej` 接受 `settle_s`、有 `hold()`。判据与实现同源
⇒ 期望值由实现自己给出 ⇒ **判据自洽地错**。真机一跑就响。

本次必须补上一条**不与实现同源**的判据（见 §9.2）。

### 1.3 这不是功能升级，是驱动替换

`send_action` / `get_observation` 的对外形状（`observation.state` 与 `action`
各 7 个关节角）**不变**。变的是底下那条链路，以及"臂怎么动"这件事。

## 2. 目标与非目标

目标：

1. 本仓在**真机**上能用：`connect()` → `send_action()` → `disconnect()` 走通。
2. 保持 LeRobot `Robot` 接口的形状与语义不变。
3. 满足 `nexform-tech/repo-template` 的仓库规范。
4. 让"离线判不了"的决定**可翻转**，把真机当硬闸门。

非目标（本次不做）：

- 笛卡尔动作（`move_l` / `move_c` / `move_path`）—— 本仓观测/动作是关节角。
- 夹爪、遥操主从、VR 输入 —— 属别的仓。
- 改固件。`s_jf_vel_max` 那张表留在固件里不动（见 §7.2）。
- 改 `litearm-python`。

## 3. 架构

```text
LeRobot Robot API
   └── LiteArmRobot            src/litearm_lerobot/robot.py
         ├── ServoLoop         src/litearm_lerobot/servo.py
         │     └── _send()  ── arm.joint_follow(q, dq, K, B)   CMD_JOINT_FOLLOW 0x08
         ├── safety            src/litearm_lerobot/safety.py
         └── litearm.Arm(port=...)  ── USB CDC ── STM32 ── CAN ── 电机
```

与旧架构的差别：`litearm-server` 与 zenoh 整段消失。伺服环落在**固件**里
（`0x08` 通道），PC 只喂目标与增益，重力前馈与限位墙力由固件每拍自算
⇒ 每拍 **1 次**往返。

`ServoLoop` 是**全仓唯一让臂动起来的地方**。`_send()` 是**唯一的下发点** —— 换执行器
是改这一个函数（见 §5.4）。

## 4. 模块与职责

| 文件 | 职责 | 依赖 |
|---|---|---|
| `safety.py`（新增） | 纯函数、无线程无 I/O：`slew_target`、`clamp_to_limits`、`read_safe_limits`、`Limits` | `math` |
| `servo.py`（新增） | `ServoLoop`：后台线程，prime → engage → 伺服循环；`set_target()` / `stop()` / `error`；`hold_at_current()` | `safety`、`litearm` |
| `robot.py`（重写） | LeRobot 适配器：连接、观测、动作、收尾 | `servo`、`litearm` |
| `config.py`（重写） | `LiteArmRobotConfig` | `lerobot` |
| `utils.py` | `register()` 不变 | `lerobot` |

把 `slew_target` 与线程分开的理由：前者是纯数学，可以与 pylitearm 原版**逐拍对拍**、
不需要假臂也不需要线程；后者要线程、要假传输。混在一个文件里会让"算法对不对"被
"线程调度对不对"掩盖。

### 4.1 代码来源与漂移护栏

`slew_target` / `clamp_to_limits` / `read_safe_limits` **从 `litearm-teleop-isomorphic`
移植**（那份又是逐字移植自 `pylitearm/control/joint_follow.py` 与
`litearm-server` 的 `teleop_manager`）。

**不 import 那个仓** —— 它拖 PyQt5，而本仓是个库。

三个文件头都写明来源与「任一侧改动，两边必须对拍」。这是**已知的安全关键代码重复**，
代价是真实的：两边漂了没人知道。缓解办法是 §9.3 的逐拍对拍判据，以及把对拍对象选成
**上游 pylitearm 原版**（不是同构遥操仓），这样两边都对着同一个第三方参照。

## 5. 执行器选型

### 5.1 三条可用基元（逐项来自固件源码）

| | `0x08` `joint_follow` | `0x03` `move_js` | `0x05` `send_mit_all` |
|---|---|---|---|
| K/B | **随帧下发**，钳 `MIT_KP_MAX=500` / `MIT_KD_MAX=5.0`；**不含** `kd_extra` | **固件出厂**：`mit_kp` 400/400/300/300/50/50/50、`mit_kd` 5/5/4/5/2.5，**外加 τ 域 `kd_extra`（J1~J4 = 6.0，J5~J7 = 0）** | **随帧下发**，钳同上 |
| 重力前馈 | 固件算 `G(q_meas) + law_wall`，**不挂** `builtin_mode` | 固件算 `G(q_d)`，**挂** `builtin_mode`（`ff_mask` 出厂全开） | **不叠** ⇒ PC 必须送 `tau_ff`，否则臂会垂 |
| 走位速率 | 参考 slew 用 `s_jf_vel_max` = `[2.8,3.4,5.0,5.0,10.0,8.0,13.0]` | `v_lim = clamp(abs(target_dq), 0, jp->speed_limit×gov)`；`dq_s` 钳到 `jp->vel_max` | `v_lim` 与 `dq_s` 钳**都**取 `jp->vel_max` |
| 入口位置钳位 | 有（`clampf(q, q_min, q_max)`） | 有 | 有 |
| 位置越界锁存 | **豁免** | 生效 | 生效 |
| 超速锁存 | **豁免** | 生效 | 生效 |
| 温度 / 电机 err / 单轴反馈陈旧 | 照判 | 照判 | 照判 |
| 每拍往返 | 1 | 1 | 2（多一次 `get_gravity`） |
| `dq` 语义 | 只进 `dq_s` 前馈 | **决定参考 slew 速率** ⇒ `dq=0` ⇒ 参考冻结、臂纹丝不动，且不报错 | 只进 `dq_s` 前馈 |

⚠ **`kd_extra` 那一格是这张表里最容易漏的一项**：它加在 **τ 域**（`τ += kd_extra·(dq_d − dq_meas)`），
位置在 `builtin_mode` 块**内**，而 `builtin_mode` 的白名单只有 `MOVE_J` 与
`MOVE_JS && !s_js_user_ff` ⇒ **`0x03` 拿得到，`0x08`（走 `MOVE_MIT_ALL`）拿不到**。
所以 `0x03` 在 J1/J2 上的**有效阻尼是 `mit_kd + kd_extra` = 5 + 6 = 11**，
不是表上的 5 —— 而 `0x08` 的阻尼**恰好等于我们下发的那几个数**。

`move_js` 的 `dq` 那一行是**静默失败**的雷区，必须配判别力判据（§9.5）。

### 5.2 看门狗：两个通道完全一样

一个容易误判的点：`0x08` 与 `0x03` 走**同一个** `watchdog_check()`，超时都是 **0.1 s**
（`params/defaults.c:172`），触发后**同样** fail-soft 持位（降刚度、τ=0、**允许下垂**）。
只有 `MOVE_J` 自己 kick 看门狗（`control_loop.c:1939`），所以长距离 `movej` 不会中途掉。

⇒ 「换 `move_js` 更安全」**不能**拿看门狗论证。

### 5.3 「速度放开」可以在 PC 侧消掉

固件的 `q_ref` 是**两级 slew 串联**（`control_loop.c:2417`）：

```text
PC   slew_target    q_cmd 以 <= speed_limit 推进      <- 本仓控制这一级
固件 slew_linear    q_ref 以 <= v_lim 追踪 q_s        <- v_lim = s_jf_vel_max（0x08 会话）
```

PC 送下去的 `q_s` 本身已经是梯形限速的参考，**两级串联谁小谁算**。同构遥操仓的 PC 表
**恰好等于** `s_jf_vel_max`（它是照抄的）⇒ 两级同时顶格。

本仓把 PC 侧 `speed_limit` 默认收到通用档（§7.1）⇒ 固件那张 `s_jf_vel_max` 不再是
约束力，"速度放开"在效果上消失，且**不需要动固件**。

⇒ 真正拿不掉的只剩那两条锁存判据豁免。

### 5.4 决定

**默认执行器 = `0x08 joint_follow`，速度档收到通用档；`actuator` 是配置字段，
`_send()` 是唯一下发点。**

四条理由：

1. 被豁免的那对是**轨迹约束**类判据，而轨迹约束已有**四层**（PC `clamp_to_limits`
   / 固件入口 `clampf` / 固件 `law_wall` / 双层 slew）。层数没少，少的是对**实测值**
   的第二意见。
2. **硬件类判据一条没动**（温度 / 电机 err / 单轴反馈陈旧），且它们的出口是**整臂
   `EMERGENCY` + `enabled=false`**（`control_loop.c:1868`）—— 比单轴锁存更硬。
   兜底没有消失。
3. 输出侧钳位全在（位置 / 力矩 / kp / kd / 速度）⇒「乱飞」这类后果从这个通道**进不来**。
4. `0x08` 是同组织真机调优过的路径（`litearm-teleop-isomorphic/liteteleop/servo.py`）。
   在一个新仓重新启用 `servo.py` 明写「已弃的旧路线」的 `move_js`，等于让第二个仓承担
   同一份未验证风险。

**但这个决定有一条离线判不了的成分**，所以不把它做死：

- `move_js` 的出厂 `mit_kp` 是调优值的 **2 倍**（J1~J4 是 400 对 200），弹簧更硬 ——
  而固件注释（`safety_check.c:162` 上方）明说锁存误判的机理就是「kp 弹簧的瞬态」。
- 反过来，`move_js` 的命令速度 ≤ 2.0 而锁存阈值是 `vel_max × 1.5` = 3.0 ⇒ 有裕量。

**哪边更容易误判，离线判不了。** 因此 `actuator` 是一等配置字段，真机 A/B 后改一行
就能翻转默认值。

### 5.5 真机 A/B 判据

同一段轨迹（§11.2 的脚本），两种 `actuator` 各跑一遍，比较：

| 指标 | 取法 |
|---|---|
| `joint_fault` 是否触发 | 跑完后 `arm.get_state().value.joint_fault` |
| `flags` 里的故障位 | `state.flag_names` |
| 跟踪误差 rms | 逐拍 `q_cmd − q_meas` |
| 观感 | 人报 |

**这条判据要能判反**：若两边都没触发，说明这段轨迹的强度不足以区分，**结论是"未定"**
而不是"两边一样"—— 要加大轨迹幅度重跑。

## 6. 接口契约

### 6.1 配置字段映射（旧 → 新）

| 旧字段 | 处置 | 依据 |
|---|---|---|
| `endpoint: str` | **删除** | zenoh 已下线 |
| `arm_id: str` | **删除** | 直连无此概念 |
| `query_timeout: float \| None` | → `move_timeout: float = 15.0` | 对应 `Arm(move_timeout=)` |
| — | 新增 `port: str \| None = None` | `None` ⇒ `litearm.find_cdc_port()` 按 VID:PID `1d50:606f` 找 |
| `num_joints: int = 7` | 保留，改为**连接后与 `arm.n` 对账**，不符即抛 | 1J 台架板 `n == 1`，这条能挡住 |
| `control_frequency_hz: float = 30.0` | → `servo_hz: float = 250.0` | 与同构遥操仓同值（节拍必须是实际周期，`slew_target` 直接吃它） |
| `movej_speed: float = 0.5` | **删除** | 由 `speed_limit` 表取代 |
| `settle_s: float = 0.2` | **删除** | 新 `movej` 自身阻塞到位 |
| `use_commander: bool = True` | **删除** | 伺服线程是唯一路径 |
| `enable_on_connect: bool = True` | 保留，但**失败改为抛**；`False` = **只读会话**（不起伺服环，`get_observation()` 照常，`send_action()` 抛） | 直连下 enable 失败 = 臂不会动；旧代码吞异常是 server 只读场景的残留。⚠ `False` 时**不能**起伺服环：固件对未使能的臂直接拒 `joint_follow`（`control_loop.c:966` 的 `if (!g_arm.enabled) return 0x03;`） |
| `disable_on_disconnect: bool = False` | 保留 | 默认保持使能持位 |
| — | 新增 `actuator: "joint_follow" \| "move_js" = "joint_follow"` | §5.4 |
| — | 新增 `k_p` / `k_d`: `list[float] \| None = None` | `None` ⇒ 用 §8 的表 |
| — | 新增 `speed_limit` / `accel_limit`: `list[float] \| None = None` | `None` ⇒ 用 §7.1 的通用档 |
| — | 新增 `limit_margin: float = 0.01` | 软限位内缩；⚠ 见 §7.2 的 J4 约束，**不能随便调大** |
| — | 新增 `engage_sec: float = 0.3` | 接管托举时长 |

**`k_p` / `k_d` 只在 `actuator="joint_follow"` 时有效。** `move_js` 没有随帧增益通道，
传了会被**忽略**。这一条必须在文档里明说，并且**在 `connect()` 时若两者同时给出就
`log.warning`** —— 不能静默。

### 6.2 生命周期

```text
connect()
  1. arm = litearm.Arm(port=..., move_timeout=...).connect()   # 握手 + 校验固件版本
  2. arm.n 必须 == num_joints，否则抛
  3. limits = read_safe_limits(arm, limit_margin)              # 读不到即抛
  4. enable()  if enable_on_connect                            # 失败即抛，并先 close()
  5. servo = ServoLoop(...); servo.start()   # 仅当已使能；prime 兼作 0x08 能力探针
     （enable_on_connect=False ⇒ 不起伺服环，本次会话只读）

disconnect()   # 幂等
  1. servo.stop()                                              # join
  2. disable()  if disable_on_disconnect
     否则 hold_at_current(arm)                                # movej 到当前实测位姿
  3. arm.close()
```

`disconnect()` 的收尾**绝不 `disable()`**（除非用户显式要求）：失能会让臂在自重下
落下，可能漂出软限位，进而锁存成那个只能人工推回去的状态。

**必须在进程退出前显式 `disconnect()`**。推荐 `with` 形式：

```python
with LiteArmRobot(cfg) as robot:      # Robot.__enter__ / __exit__ 由 lerobot 提供
    ...
```

lerobot 的 `Robot.__del__` 是兜底网，但 GC 时刻不保证；`litearm.Arm.__del__` 会兜住
**链路**清理（委托给 `close()`），**兜不住姿态**。

### 6.3 方法语义

```python
get_observation() -> {"observation.state": [q0..q6]}   # float，来自 arm.get_state().value.q
send_action(a)    -> {"action": [q0..q6]}              # 返回【钳位后】的值
```

- `get_observation` 用 `arm.get_state(refresh=False)` 走缓存（实测 0.001 ms；
  `refresh=True` 是 10 ms，进了循环就等于把节拍钉死）。
- `send_action` 先 `clamp_to_limits`，**有轴被钳就 `log.warning` 报出轴号**，
  然后 `servo.set_target(q)`（非阻塞），返回钳位后的 `q`。
  LeRobot 的契约要求返回"实际发出的动作"，所以返回钳位后的值。
- 长度或有限性不符 ⇒ 抛 `ValueError`。（固件在协议边界也会拒 NaN/Inf，但这里早失败
  给出的信息更清楚。）

## 7. 安全设计

### 7.1 速度与加速度默认档（有意收窄）

```python
speed_limit = [2.0, 2.0, 1.75, 1.75, 2.0, 2.0, 2.0]   # = 固件 jp->speed_limit
accel_limit = [8.0, 8.0, 7.0, 7.0, 9.0, 9.0, 9.0]      # = 固件 jp->acc_max
```

取固件通用档，不取同构遥操仓的 `[2.8,3.4,5.0,5.0,10.0,8.0,13.0]`。

理由：本仓是**库**，不知道调用方是"人在场的数采"还是"无人值守的策略回放"。
默认值要取保守一侧（缺实测取保守下界）。要遥操级响应速度，显式传参即可 —— 那时
它就等于固件表，也就是同构遥操仓那份已验证配置。

**一条判据**（§9.4）：`speed_limit[i] <= s_jf_vel_max[i]` 必须成立。有人在默认值上
调到固件表之上，就等于把"速度放开"悄悄放回来了。

### 7.2 限位：五层

| 层 | 位置 | 作用 |
|---|---|---|
| 1 | 本仓 `clamp_to_limits` | 钳**目标**；限位源自固件 `params.all_joint_params()` 的 `q_min/q_max` 内缩 `limit_margin` |
| 2 | 本仓 `slew_target` | 限速 + 限加速（梯形曲线 + 制动距离） |
| 3 | 固件 `ctrl_accept_joint_follow` | 入口 `clampf(q, q_min, q_max)` |
| 4 | 固件 `law_wall` | 距限位 `margin` 处给排斥力矩，叠进 `tau_ff` |
| 5 | 固件 `slew_linear` | `q_ref` 逐拍斜率限制 |

⚠⚠ **`limit_margin` 不能随便调大 —— J4 的固件上端只有 `+0.017547 rad`（1°）**
（`joint_limit_macros.h:16`）。内缩 `margin` 后 J4 的上界是 `0.017547 − margin`：

```text
margin = 0.01（本仓取值）  -> J4 上界 +0.0075 rad，仍可用
margin = 0.02（错的值）    -> J4 上界 -0.0025 rad  ⇒ J4 整个正半轴消失
```

而且**这个错误不会报错**：`read_limits_ok` 只检查 `lo < hi`，两者仍成立 ⇒ 静默地
把 J4 的正半轴抹掉。这正是同构遥操仓文档里记的那次真机事故的根因
（"J4 特别容易过软件限位"），当时的内缩值曾被抬到 0.14，比实际所需大一个量级。

关于下界的方向也别搞反：固件的位置锁存条件是 `q_meas > q_max + 0.05`，而目标已被钳到
`q_max − margin`、实测最多再冲过 `overshoot ≈ vel_max × 2 × RTT ≈ 0.013 rad` ⇒
**不锁存的条件是 `margin ≥ overshoot − 0.05`**，即任何 `margin ≥ 0` 都够。取 0.01 是
照抄 litearm-server 的浮点/标定余量。

**判据**（§9.4）：`hi[i] > lo[i]` 之外，还要断言 **J4 的可用行程不被压没**
（`hi[3] - lo[3]` 不小于固件行程的 99%）。
限位**读不到就拒启动**，绝不退回哨兵值（`read_safe_limits` 抛 `LimitsError`）。

### 7.3 `0x08` 会话豁免的精确边界

`safety_check()` 返回 true ⇒ **整臂 `EMERGENCY` + `enabled=false`**，并且"进入锁存立发
失能帧 + 每 256 拍刷新"。被豁免的两条只是不再**参与**这个判定：

| `safety_check` 里的判据 | `0x08` 会话 | 触发后果 |
|---|---|---|
| 位置越界（`q` 出 `[q_min−0.05, q_max+0.05]`，连续拍去抖） | 豁免 | — |
| 超速（`abs(dq) > vel_max × 1.5` 连续 5 拍） | 豁免 | — |
| 单轴反馈陈旧 / 电机 err / 温度 | **照判** | 整臂 `EMERGENCY` + 失能 |

豁免本身有边界：`ctrl_joint_follow_active()` 带 `!hold`（`control_loop.c:684-685`）——
看门狗一触发或 `drop_hold` 一置，豁免**立即结束**，回到与通用路径**完全一致**的语义。

⚠ `hold` 有**两个来源**，性质**不同**（`control_loop.c:1942-1945`）：

| 来源 | 触发 | 持位强度 |
|---|---|---|
| `watchdog_tripped()` | PC 断开 / 超时 | fail-soft：**降刚度、τ=0、允许下垂** |
| `drop_hold` | 有轴掉线 / 自保护失能 | **刚性持位**：全刚度 + 重力前馈 |

两者都让豁免结束，但只有前者会下垂。

### 7.4 失败方向（必须写进用户文档）

| 情形 | 结果 |
|---|---|
| 进程被 `kill -9` | 伺服线程随进程消失 ⇒ 固件 0.1 s 看门狗 fail-soft ⇒ **臂缓慢下垂** |
| 链路断 | `_send` 抛 ⇒ `hold_at_current()` 受控接管；接管也失败 ⇒ 记 `servo.error`，此时 fail-soft 下垂 |
| 调用方忘记 `disconnect()` | 链路由 `Arm.__del__` 兜住；**姿态不保证** |
| 正常 `disconnect()` | `movej` 到当前实测位姿，交回固件 `MOVE_J` 持位 |

第一行是 `0x08`/`move_js` 通道的**固有性质**，不是缺陷 —— 两个通道都这样（§5.2）。

### 7.5 故障要有具名载体

伺服线程里抛出的任何异常（链路坏、固件没有 `0x08`）：

1. 线程内先 `hold_at_current()` 受控接管；
2. 把异常对象存进 `servo.error`；
3. 线程退出；
4. 此后 `get_observation()` / `send_action()` **显式抛**（带上原异常）。

不做"线程死了、调用方照旧拿到旧数据"。`servo.error` 是那个不变量的具名载体。

`prime` 那一帧兼作**能力探针**：固件不支持 `0x08` 时立刻
`UnsupportedByFirmwareError`，不等到第一次 `send_action` 才发现。

## 8. 参数真值

### 8.1 伺服增益

```python
k_p = [200.0, 200.0, 200.0, 200.0, 80.0, 80.0, 80.0]
k_d = [3.0, 5.0, 3.0, 3.0, 1.5, 1.5, 1.5]
ENGAGE_KP, ENGAGE_KD = 15.0, 0.8
```

**来源**：`litearm-teleop-isomorphic/liteteleop/servo.py` 的 `SETUP_K` / `SETUP_B`。

**但不是本仓实测，而且证据强度要如实标：**

| 轮次 | 表（J5~J7） | 结论 |
|---|---|---|
| P2 | K 133、B 2.4 | 真机验收：滞后 ↓55%、J2 过冲 ↓86%（**这两组数字属于 P2 那张表**） |
| P3（= 本仓采用的这版） | K 80、B 1.5 | 台账自己写着「**本轮不是干净对照实验**……绝对值不可跨轮直比」；依据是"更贴出厂值"；实测结论是用户在**两版之间手感无差别** |

⚠ **不要把 P2 的 ↓55%/↓86% 记到这版表上**。P3 那轮真正的结论是
「J5 的过冲与增益**无关**（K 40→133→80、B 0.8→2.4→1.5，各变 3 倍多，过冲始终在
0.053~0.063 的窄带里）⇒ **结构性**，最可能是腕部机械弹性，**建议不再追**」。

而且这份调参的**场景是主从遥操**（人手拖主臂、连续平滑目标），本仓还有策略回放
（目标可能跳变）。**场景不同 ⇒ 这批值在本仓只能算"起点"，不是"已验证"。**

`k_d` 里 J2 = 5.0 是**顶到固件上限** `MIT_KD_MAX`。写 6.0 会被固件**静默钳成 5.0**
—— 表上看着满足等比、实际值不符。故写真值 5.0。

### 8.2 台账登记

本组织唯一的调参台账是 **`/home/llx/litearm-stm32/tools/PARAM_STATE.md`**
（`§B`，其下有 `§B-补. 遥操侧（PC）常量` 一节专门收 PC 侧常量 —— 同构遥操仓的
P1/P2/P3 三条就登记在那里）。

**本仓不建第二本台账**（两本必然漂），本次要登记的两条，**由我来写进 spec 但由用户
落到那份台账里** —— 它在另一个仓，本仓的改动不去动它：

| 项 | 值 | 依据 | 状态 |
|---|---|---|---|
| PC 侧 `speed_limit` / `accel_limit` | `[2.0,2.0,1.75,1.75,2.0,2.0,2.0]` / `[8,8,7,7,9,9,9]` | 有意收窄；来源 = 固件 `jp->speed_limit` / `jp->acc_max` | 待真机验证 |
| PC 侧 `k_p` / `k_d` | 见 §8.1 | 来源 = 同构遥操仓 P3 那版 | **非本仓实测** |
| PC 侧 `limit_margin` | `0.01` | 照抄 litearm-server；**受 J4 上端 `+0.0175 rad` 约束**（§7.2） | 沿用，非本仓实测 |

## 9. 测试策略

### 9.1 FakeArm 按真 SDK 的形状重造

`FakeArm` 必须逐个复刻真对象的形状，否则判据又会同源：

- 构造：`Arm(port=..., move_timeout=...)`，**没有** `endpoint` / `arm_id`
- `connect()` 幂等，返回 `self`
- `get_state(refresh=False) -> Msg(value=RobotState)`，`RobotState` 有 `.q` / `.dq` /
  `.joints` / `.flags` / `.flag_names` / `.joint_fault` / `.faulted`
- `params.all_joint_params() -> list[JointParam]`，每项有 `.q_min` / `.q_max`
- `joint_follow(q, dq, kp, kd)` / `move_js(q, dq)` / `movej(q, speed)` / `enable()` /
  `disable()` / `close()`
- **没有** `hold()`

### 9.2 签名对账判据（本次事故的补丁）

一条测试直接对**真 SDK** 取签名，断言本仓实际用到的每个名字与关键字都在其中：

```python
import inspect, litearm

def test_arm_constructor_keywords_exist():
    params = inspect.signature(litearm.Arm.__init__).parameters
    for kw in ("port", "move_timeout"):
        assert kw in params

def test_no_unexpected_arm_kwargs():
    # 本仓源码里出现的 Arm(...) 关键字，逐个必须在真签名里
    ...
```

期望值取自**真 SDK**，不与本仓实现同源 ⇒ 这条判据**不会自洽地错**。它正是 1.1 那次
失效的直接补丁。

### 9.3 `slew_target` 与上游逐拍对拍

⚠⚠ **这条判据在 CI 上不跑。** 它硬编码
`/home/llx/pylitearm/src/pylitearm/control/joint_follow.py` 这个**本机绝对路径**，
而 CI 只 checkout 本仓 ⇒ `skipif` 命中、静默跳过。也就是说 §4.1 那条「三份副本
靠对拍防漂移」的缓解措施**只在有那台机器的人手里成立**，不是流水线闸门。
跳过时带 `reason`，所以是**已知缺口**而不是隐藏缺口；补它需要把 pylitearm 带进 CI
（本次不做）。

对着 **pylitearm 原版**（不是同构遥操仓）的 `joint_follow.py` 逐拍比对，覆盖：
到目标吸附、制动距离减速、加速度限幅、方向反转。

### 9.4 默认值判据

`speed_limit[i] <= s_jf_vel_max[i]`，逐轴断言。

**`s_jf_vel_max` 只能硬编码，不能靠解析。** 两条理由：① 它是 `static const float`
定义在 `control_loop.c:238`，**没有任何头文件声明它**（`litearm.h:83` 只在注释里提到
名字）；② 本仓的 CI 从不 checkout `litearm-stm32`（`.github/workflows/ci.yml` 只装
`litearm-python`）⇒ 解析根本跑不了。

所以测试里硬编码 `[2.8,3.4,5.0,5.0,10.0,8.0,13.0]` 并在注释里写上
「来源 = `litearm-stm32` `control_loop.c:238`，改固件那张表时必须同步改这里」。
期望值仍然**不是**从本仓实现取的（本仓默认是通用档），所以这条判据有判别力。

### 9.5 伺服环的判别力

喂一个阶跃目标，断言：

1. 参考序列**单调收敛**到目标；
2. 每一拍都调到了 `_send`（往返数 == 拍数，允许 1 拍误差）；
3. **`dq_cmd` 不全为 0** —— `dq=0` 在 `move_js` 上会让参考冻结、臂纹丝不动且不报错。
   这条是负控的反面：没有它，"循环跑了"与"臂真会动"分不开。
4. **判据的判据是耗时**：断言实测节拍不低于设定的一半（⚠ 实现在
   `tests/test_servo.py` 里取的是 **0.5×** 而不是本行原先写的 0.9× —— 0.9× 在
   CI 的抖动下会偶发红，而这一条要挡的是「线程根本没跑起来」那种量级的失效，
   0.5× 已足够判别。相邻的 `test_one_send_per_tick` 是更强的那条：它能挡住
   「每拍发两次」与「循环卡住」。线程若没真跑起来，
   前三条的断言可能仍然成立。

### 9.6 生命周期与收尾

- `disconnect()` 调的是 `movej`，**不是** `disable`。
- `disable_on_disconnect=True` 时才断言 `disable`。
- 两次 `disconnect()` 幂等。
- `connect()` 两次幂等。

### 9.7 失败路径

- 读不到限位 ⇒ `connect()` 抛。
- `arm.n != num_joints` ⇒ `connect()` 抛。
- 伺服线程异常后 `send_action()` / `get_observation()` 抛，且异常对象是原来那个。
- `actuator="move_js"` 时 `_send` 真的调了 `move_js` 且**不传** kp/kd。
- `actuator="move_js"` 且用户给了 `k_p` ⇒ `connect()` 时告警（不静默）。

## 10. repo-template 规范合规

已合规（核对过，无需改动）：`AGENTS.md`、`.releaserc.json`、
`.github/workflows/release.yml`、`.github/workflows/ci.yml`（无 `exit 1` 占位，
测试命令是真的）。

⚠ **`AGENTS.md` 那条是用 blob SHA 核的，不是 diff**（diff 会因为本地克隆陈旧而误报）：
本仓 `HEAD:AGENTS.md`、本地克隆的 `origin/main:AGENTS.md`、以及
`gh api repos/nexform-tech/repo-template/contents/AGENTS.md` 三者同为
`c45dafeaeff442ff0de60f68e75abf392288624e`。本地克隆的 remote 是
`yang-dong-yd/repo-template`（fork），与 `release.yml` 里引用的
`nexform-tech/repo-template` **同内容** —— 这一条只对 `AGENTS.md` 成立，其余文件没核。

本次要补：

| 项 | 处置 |
|---|---|
| `pyproject.toml` 的 `version` | `0.1.0` → **`0.0.0+semantic-release`**（`AGENTS.md` §3 + `INTEGRATION.md` gotcha 6 要求用占位符、唯一真源是 git tag；⚠ 但模板写的那串**过不了 PEP 440**，见下） |

⚠ **占位符的拼法在本仓必须是 `0.0.0+semantic-release`，不是模板里写的
`0.0.0-semantic-release`**：后者**过不了 PEP 440**，setuptools 直接拒（实测
`configuration error: project.version must be pep440`）⇒ CI 的
`pip install -e ".[dev]"` 装不上包。同组织 `litearm-studio/daemon/pyproject.toml`
就是 `0.0.0+semantic-release`。

| `pyproject.toml` 的依赖 | `litearm-python>=0.1.0` → `>=2.1.0`（旧值指向 server 语义那份） |
| `ci.yml` | 保持 `litearm-python @ git+https://github.com/nexform-tech/litearm-python`（它不在 PyPI 上）；确认与新版 `pyproject` 一致 |
| `README.md` / `README.zh-CN.md` | 重写：删 zenoh + server，加固件版本要求与 CDC 端口 |
| `docs/DEVELOPER_GUIDE.md` / `.zh-CN.md` | 重写：加执行器选型、速度档、看门狗下垂这条 **do not** |
| `examples/*` | 参数从 `--endpoint/--arm-id` 换成 `--port`；`02` 的轨迹要从**当前实测位姿**偏移，不是把全轴设成同一个绝对值 |
| 文档风格 §4 | 每份开头一句「这是什么、给谁看」；清掉现有 emoji（`📖`、`⚠️`）；加 **do not** 行 |
| 孤儿检查 | 逐个确认每个文件都被引用到 |

**内部 spec 用中文**：本组织其他仓的 `docs/superpowers/specs/*` 都是中文，而 AGENTS.md
的「English」规矩实际落在**交付文档**上（README / 指南 / 示例）。本仓的交付文档仍按
现状走英文 + `zh-CN` 双份。

## 11. 交付与验收

### 11.1 交付边界

- 分支：`feat/port-to-litearm-python-direct-cdc`（大改走分支）
- **本地提交，不 push**；PR 由仓库所有者开
- 破坏消费者的提示：**六个**配置字段消失或改名 —— `endpoint`、`arm_id`、
  `use_commander`、`movej_speed`、`settle_s` 消失，`query_timeout` 改名为
  `move_timeout`（见 §6.1）⇒ 现有配置全部失效
- 按 repo-template AGENTS.md §3，**不自行写 `BREAKING CHANGE:`**。本仓 tag 只有
  `v0.0.0`，首次 `feat:` 会发 **0.1.0**。若需 1.0.0，由用户明确要求

### 11.2 真机验收（离线判不了的部分，交给用户执行）

1. **只读**：`examples/01_read_observation.py` 连上，读数与 `pylitearm` 时代一致。
2. **小幅度运动**：`examples/02` 幅度 0.02 rad、`servo_hz=250`，确认平滑、无抖动、
   无 `joint_fault`。`disconnect()` 后臂**保持刚性持位**（不是下垂）。
3. **数采**：跑一段遥操录制，看 `joint_fault`、看数据是否连续。
4. **回放**：跑一段策略回放，同样看 `joint_fault`。
5. **A/B**：§5.5 的判据，`actuator` 两种各跑一遍。
6. **失败方向**：故意 `kill -9`，确认下垂是缓慢的、可控的（这一步要人在场、
   手边有急停）。

前五条全过才允许合并。第 6 条可以单独跑。

## 12. 已知偏离与非目标

| 项 | 说明 |
|---|---|
| 安全关键代码重复 | `slew_target` 在本仓与 `litearm-teleop-isomorphic` 各一份。缓解见 §4.1、§9.3 |
| 默认速度档偏离上游 | 同构遥操仓用固件表，本仓默认收到通用档（§7.1）。这是**有意**的，不是照抄漏了 |
| K/B 非本仓实测 | 直接搬同构遥操仓 P3 那版（§8.1）。**它不是被"↓55%/↓86%"验证过的那版**，而且那份调参的场景是主从遥操、不是策略回放 ⇒ 本仓只能当"起点" |
| `0x08` 两条锁存豁免 | 拿不掉（PC 侧无开关）。已按 §7.3 写清精确边界与残余风险 |
| `move_js` 通道 | 已实现但非默认；它的 `dq` 静默失败风险、出厂增益（含 `0x08` 拿不到的 `kd_extra`）均**未在本仓验证** |
| 台账不在本仓 | 组织唯一的调参台账在 `/home/llx/litearm-stm32/tools/PARAM_STATE.md`；本仓不建第二本，登记条目见 §8.2，由用户落到那份里 |

## 附录 A：固件源码依据索引

基线 `litearm-stm32` master。

| 结论 | 位置 |
|---|---|
| `s_jf_vel_max` 定义与"只对 `0x08` 生效" | `User/litearm/control/control_loop.c:238` |
| `v_lim` 按会话取（`jf_on ? s_jf_vel_max : jp->vel_max`） | `control_loop.c:2305` |
| `ctrl_accept_joint_follow` 入口 `clampf`（`[M5 fix]`） | `control_loop.c:962`（钳位在 `:973`） |
| `joint_follow` 的前馈不挂 `builtin_mode` | `control_loop.c:2108` |
| `builtin_mode` 白名单（仅 `MOVE_J`、`MOVE_JS` 且无用户 `tau_ff`） | `control_loop.c:2522` |
| `MOVE_JS` 分支（`kp_s=mit_kp`、`tau_u` 由 `s_js_user_ff` 决定） | `control_loop.c:2270`、`:2279` |
| `kd_extra` 加在 τ 域、且**在 `builtin_mode` 块内** ⇒ `0x03` 有、`0x08` 无 | `control_loop.c:2633`（块自 `:2577`） |
| `MIT_KP_MAX=500.0f` / `MIT_KD_MAX=5.0f` 的线格式硬上限 | `control_loop.c:44-45` |
| `slew_linear` 两级串联的那一级 | `control_loop.c:2417` |
| `ctrl_joint_follow_active` 的定义与带 `!hold` 的返回 | `control_loop.c:674`、`:684-685` |
| `hold` 的两个来源（fail-soft 下垂 vs 刚性持位） | `control_loop.c:1942-1945` |
| `safety_check` → 整臂 `EMERGENCY` + 失能 | `control_loop.c:1868` |
| 两条判据的豁免门 | `User/litearm/safety/safety_check.c:162` |
| 看门狗超时 0.1 s（**7J 那张表**；`:48` 是 1J 台架表，值相同） | `User/litearm/params/defaults.c:172` |
| 逐关节出厂值（`mit_kp`/`mit_kd`/`kd_extra`/`tau_max`/`speed_limit`/`vel_max`） | `params/defaults.c:65-124` |
| `MOVE_J` 自动 kick 看门狗（其余模式不踢，保留断连 fail-soft） | `control_loop.c:1938-1939` |
