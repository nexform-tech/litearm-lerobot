"""配置契约 —— 字段名与默认值都是对外契约，改名即破坏消费者。"""
from __future__ import annotations

import pytest

from litearm_lerobot import LiteArmRobotConfig
from litearm_lerobot.servo import (
    DEFAULT_ACCEL_LIMIT,
    DEFAULT_K_D,
    DEFAULT_K_P,
    DEFAULT_SPEED_LIMIT,
)


def test_registered_under_litearm():
    assert LiteArmRobotConfig().type == "litearm"


def test_old_server_fields_are_gone():
    """六个字段消失或改名 —— 见 spec §11.1，PR 里要明说破坏消费者。"""
    cfg = LiteArmRobotConfig()
    for gone in ("endpoint", "arm_id", "use_commander", "movej_speed", "settle_s",
                 "query_timeout"):
        assert not hasattr(cfg, gone), f"{gone} 是旧 server 语义的字段"


def test_new_defaults():
    cfg = LiteArmRobotConfig()
    assert cfg.port is None              # None = find_cdc_port() 自动找
    assert cfg.move_timeout == 15.0
    assert cfg.num_joints == 7
    assert cfg.servo_hz == 250.0
    assert cfg.actuator == "joint_follow"
    assert cfg.enable_on_connect is True
    assert cfg.disable_on_disconnect is False
    assert cfg.limit_margin == 0.01      # ⚠ 不是 0.02（会毁掉 J4 正半轴）
    assert cfg.engage_sec == 0.3


def test_servo_table_defaults_come_from_servo_module():
    cfg = LiteArmRobotConfig()
    assert cfg.k_p is None and cfg.k_d is None
    assert cfg.speed_limit is None and cfg.accel_limit is None
    assert DEFAULT_K_P == [200.0, 200.0, 200.0, 200.0, 80.0, 80.0, 80.0]
    assert DEFAULT_K_D == [3.0, 5.0, 3.0, 3.0, 1.5, 1.5, 1.5]
    assert DEFAULT_SPEED_LIMIT == [2.0, 2.0, 1.75, 1.75, 2.0, 2.0, 2.0]
    assert DEFAULT_ACCEL_LIMIT == [8.0, 8.0, 7.0, 7.0, 9.0, 9.0, 9.0]


def test_actuator_only_accepts_known_names():
    with pytest.raises(ValueError):
        LiteArmRobotConfig(actuator="nope").validate()
