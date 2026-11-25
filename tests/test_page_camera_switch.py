"""摄像头画面右上角「切换摄像头」按钮的回归测试（2026-11-25 用户要求）。

需求原话：「在 hp60c 的 rgb 显示画面的右上角添加一个切换的按键，
这个按键只有在摄像头画面打开之后才显示，然后点击之后切换摄像头到这个新的摄像头」。

* 位置：`#camStage` 的**右上角**（`right` + `top`），不是右下角（右下角是刷新键）；
* 可见性：默认 `display:none`，只有画面打开（`camStart()`）才加 `.show`，
  关画面（`camStop()`）立刻摘掉；
* 行为：点一下把 `<img>` 的地址换成 `?cam=wide`，再点回主摄；
* 断流自愈必须**带着当前选择**重连，否则切到广角后一断就悄悄跳回主摄。
"""

from __future__ import annotations

import pathlib
import re

PAGE = pathlib.Path(__file__).resolve().parents[1] / 'cloud/web/car.html'


def _css(selector: str) -> str:
    src = PAGE.read_text(encoding='utf-8')
    m = re.search(r'(?m)^[ \t]*' + re.escape(selector) + r'\s*\{([^}]*)\}', src)
    assert m, f'没找到 CSS 规则：{selector}'
    return m.group(1)


def test_swap_button_sits_in_top_right_corner():
    body = _css('#camSwap')
    assert re.search(r'right:\s*8px', body)
    assert re.search(r'top:\s*8px', body)
    assert not re.search(r'bottom:\s*\d+px', body)   # 右下角是刷新键，别抢


def test_swap_button_hidden_until_stream_starts():
    assert '#camSwap.show { display: flex; }' in PAGE.read_text(encoding='utf-8')
    hidden = _css('#camSwap')
    assert 'display: none' in hidden


def test_swap_button_lives_inside_camera_stage():
    src = PAGE.read_text(encoding='utf-8')
    stage = re.search(r'<div id="camStage">(.*?)</div>\s*</div>', src, re.S)
    assert stage, '没找到 #camStage'
    assert 'id="camSwap"' in stage.group(1)


def test_stream_url_carries_camera_choice():
    src = PAGE.read_text(encoding='utf-8')
    m = re.search(r'function camSrcURL\(\)\s*\{(.*?)\n\}', src, re.S)
    assert m, '没找到 camSrcURL()'
    body = m.group(1)
    assert "'cam='" in body
    assert "camWide ? 'wide' : 'main'" in body


def test_button_toggles_between_main_and_wide():
    src = PAGE.read_text(encoding='utf-8')
    m = re.search(r"\$\('camSwap'\)\.onclick\s*=\s*\(e\)\s*=>\s*\{(.*?)\n\};", src, re.S)
    assert m, '没找到 camSwap 的点击处理'
    body = m.group(1)
    assert 'camWide = !camWide' in body
    assert 'camReload()' in body          # 清 src 重连，不能只改 label


def test_reconnect_keeps_selected_camera():
    src = PAGE.read_text(encoding='utf-8')
    assert "i.src = camMedia.url + '?t='" not in src, '断流重连还在用旧地址（会跳回主摄）'
    # 两处自愈（onerror + 3 s 兜底轮询）都走 camReload()
    # 手动画面：onerror + 3 s 兜底 + 点切换各一次
    assert src.count('camReload();') >= 3


def test_nav_camera_pane_has_same_switch():
    """导航模式那块画面也有切换键（用户 2026-11-25：都加上）。"""
    src = PAGE.read_text(encoding='utf-8')
    pane = src[src.index('<div id="navCamPane"'):]
    pane = pane[:pane.index('</div>')]
    assert 'id="navCamSwap"' in pane

    body = _css('#navCamSwap')
    assert re.search(r'right:\s*8px', body)
    assert re.search(r'top:\s*8px', body)
    # 顶部两行浮层是 z-index 6 且 pointer-events:none，按钮必须更高才点得到
    assert re.search(r'z-index:\s*7', body)


def test_both_switches_share_one_choice():
    src = PAGE.read_text(encoding='utf-8')
    m = re.search(r'function camSwapLabel\(\)\s*\{(.*?)\n\}', src, re.S)
    assert m and "'navCamSwap'" in m.group(1)
    assert src.count("camWide = !camWide") >= 2      # 手动 + 导航各一个入口


def test_switch_replaces_img_node():
    """黑屏事故根因：WebKit 不认 removeAttribute('src')，旧 MJPEG 长连接不断，
    攒满同域 6 条连接后两路全黑。必须换新的 <img> 节点。"""
    src = PAGE.read_text(encoding='utf-8')
    assert 'function camMount' in src
    assert 'old.replaceWith(fresh)' in src
    # 去掉注释后，除 camMount 内部那一处，页面里不许再有"只清 src 断流"的写法
    code = re.sub(r'/\*.*?\*/', '', src, flags=re.S)
    code = re.sub(r'//[^\n]*', '', code)
    assert code.count("removeAttribute('src')") == 1


def test_all_three_panes_remount():
    src = PAGE.read_text(encoding='utf-8')
    assert "camMount('camImg'" in src
    assert "camMount('navCamImg'" in src
    assert "camMount('followImg'" in src


def test_nav_swap_uses_reload_helper():
    src = PAGE.read_text(encoding='utf-8')
    assert 'function navCamReload' in src
    swap = src.split("$('navCamSwap').onclick", 1)[1][:400]
    assert 'navCamReload()' in swap
