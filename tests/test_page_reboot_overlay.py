"""重启主控的全屏提示（2026-11-24 用户要求）。

原来点「重启主控」什么都不显示：页面停在正常界面，直到 15 秒后判离线才
闪一下「设备离线」，用户不知道到底有没有在重启。现在照关机那样给一个
全屏页：正在重启 → 已断开等待上线 → 已上线（要真断过 + 连续在线 15 s）。
"""
from pathlib import Path

PAGE = Path(__file__).resolve().parents[1] / 'cloud' / 'web' / 'car.html'


def _src():
    return PAGE.read_text(encoding='utf-8')


def test_reboot_has_fullscreen_overlay():
    s = _src()
    assert 'let rebootPending = false;' in s
    assert 'function rebootView(title, hint)' in s
    assert 'function rebootTick()' in s
    assert 'setInterval(rebootTick, 1000);' in s
    assert s.count("rebootView('正在重启主控…'") == 3, '点击后 + 两个阶段都要显示全屏页'


def test_reboot_click_opens_overlay_immediately():
    s = _src()
    assert "} else {\n      rebootPending = true;" in s, '重启分支必须立刻置位'
    assert 'rebootAt = Date.now();' in s
    assert 'showToast(\'已发送重启指令\', false);' in s


def test_reboot_online_needs_offline_then_15s():
    s = _src()
    assert 'if (!ok && rebootPending) {' in s
    assert 'if (ok && rebootPending) {' in s
    assert 'if (!rebootSeenOffline) {\n      // 刚发指令、车端还在收尾' in s
    assert 'Date.now() - rebootOnlineSince >= 15000' in s
    assert "showToast('设备已重新上线', true);" in s


def test_offline_tick_does_not_clobber_reboot_page():
    s = _src()
    assert 'if (!robotOfflineSince || shutdownPending || rebootPending) return;' in s


def test_shutdown_flow_still_intact():
    s = _src()
    assert "if (action === 'shutdown') {" in s
    assert "t0.textContent = '正在关机…';" in s
