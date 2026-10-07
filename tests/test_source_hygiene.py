"""守一条规则：包源码里不许出现写死的发行版本号。

AGENTS.md §3 把 git tag 定为版本的唯一真相，并禁止手改版本号 —— 那么源码里的
版本字面量按定义就是**第二份、且会漂移的副本**：semantic-release 永远不会重写它，
它保持正确的唯一方式是谁都没发现它是错的。

§4 说一条没人能检查的规则是噪音。所以这条规则由本文件来检查：谁再把
`__version__ = "1.2.3"` 加回来，在这里红，而不是发布出去。
"""
from __future__ import annotations

import re
from pathlib import Path

import litearm_lerobot

_PACKAGE_DIR = Path(litearm_lerobot.__file__).parent

#: 形如 `__version__ = "1.2.3"` / `VERSION = '2.0'` 的赋值。只认**版本形状**的
#: 字面量（`\d+\.\d+` 起头），免得把 `__version__ = _dist_version(...)` 这类
#: 派生写法也判成违规 —— 要禁的是写死的数字，不是这个属性名。
_HARDCODED_VERSION = re.compile(
    r"""^\s*(?:__version__|VERSION|version)\s*=\s*["']\d+\.\d+""",
    re.MULTILINE,
)


def test_no_hardcoded_version_literal_in_the_package():
    offenders = sorted(
        p.relative_to(_PACKAGE_DIR).as_posix()
        for p in _PACKAGE_DIR.rglob("*.py")
        if _HARDCODED_VERSION.search(p.read_text(encoding="utf-8"))
    )
    assert offenders == [], (
        f"包源码里出现了写死的版本号：{offenders}。"
        "版本只认 git tag（AGENTS.md §3）；运行时要用版本请走 importlib.metadata。"
    )
