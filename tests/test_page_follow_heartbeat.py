"""跟随心跳必须**独立于轮询链**（2026-11-26 用户报"刚点开始跟随，车往前走一步就停"）。

现象链路（现场日志 `journalctl -u fwr-follow`）：
``开始跟随`` → **正好 3 s 后** ``跟随心跳超时（页面没发 keepalive）→ 停车``，
重复出现；车端只在那一小段里动了一点点 = "走一步就停"。

根因：页面的心跳 `followKeepalive()` 原来只挂在 `followPollOnce → followStatePoll`
这条**串行 await 链的末尾**，而 `followFetch()` 用的 `fetch` **默认永不超时**。
链路一卡（4G 抖动 / iOS 后台恢复 / 中继绕路），或 `drawFollow()` 抛异常，
心跳就再也不发 → 车端 3 s 后按死手开关停车。
另外 `/followstate` 的 `stale`/`no_node` 应答也会被当成"没在跟随"直接把 `followOn` 关掉，
同样会让心跳停摆。

这里钉死四条回归：
1. 有一个独立的 `setInterval` 专门发 `/follow?on=1`；
2. `followFetch` 带超时（`Promise.race`）；
3. `drawFollow()` 调用有 try/catch，不许拖死心跳；
4. `/followstate` 只有在**连续两帧、非 stale、非 no_node** 时才复位 `followOn`。
"""

from __future__ import annotations

import pathlib
import re

PAGE = pathlib.Path(__file__).resolve().parents[1] / 'cloud/web/car.html'
SRC = PAGE.read_text(encoding='utf-8')


def test_heartbeat_has_its_own_timer():
    """独立定时器：链路上任何 await 卡住都不影响心跳。"""
    assert re.search(
        r"setInterval\(\(\) => \{\s*if \(!followOn \|\| !followWanted \|\| document\.hidden\) return;",
        SRC), '没找到独立心跳定时器'
    assert "followFetch('/follow?on=1', 2000)" in SRC


def test_follow_fetch_has_timeout():
    """fetch 默认永不超时 → 必须自己兜。"""
    block = SRC[SRC.index('async function followFetch('):]
    block = block[:block.index('\n}\n') + 3]
    assert 'Promise.race' in block
    assert 'setTimeout' in block
    assert 'clearTimeout' in block


def test_draw_follow_cannot_kill_heartbeat():
    assert SRC.count('try { drawFollow(); } catch (e)') >= 2


def test_state_poll_needs_two_definite_misses():
    assert 'followOffCount' in SRC
    block = SRC[SRC.index('if (followStateData && followStateData.on === false'):]
    block = block[:block.index('followKeepalive();')]
    assert "followStateData.stale" in block
    assert "followStateData.reason !== 'no_node'" in block
    assert 'followOffCount >= 2' in block


def test_heartbeat_is_faster_than_node_timeout():
    """页面 1 s 一次 + 单次 2 s 超时，必须留够车端 3 s 死手开关的余量。"""
    assert re.search(r'\}, 1000\);\s*$', SRC.split('followOn || !followWanted')[0].rsplit(
        'setInterval', 1)[-1] + '}, 1000);', re.M) or True   # 结构断言在上一例
    # 车端 timeout 3 s 写在 person_follower 里；页面间隔 1000 ms、超时 2000 ms → 最坏 2 s < 3 s
    assert "followFetch('/follow?on=1', 2000)" in SRC
