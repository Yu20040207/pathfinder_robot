"""手机页「异地看画面」（云端中继）的回归。

现场背景：`/media` 下发的地址是机器人**局域网 IP**，手机换 4G / 别的 WiFi
就不可达；控制本来走云 MQTT 所以没事，只有画面黑。现在页面会：
局域网直连 → 2.5 s 没图就自动切云端中继（同源 `/api/...`，短时令牌）。
"""
import pathlib

HTML = (pathlib.Path(__file__).resolve().parents[1] / 'cloud' / 'web' / 'car.html')


def text() -> str:
    return HTML.read_text(encoding='utf-8')


def test_page_keeps_remote_info_from_media():
    body = text()
    assert 'let camRemote = null;' in body
    assert 'let camViaRemote = false;' in body
    # 两个入口（手动 / 导航）都要记下来
    assert body.count('camRemote = camMedia.remote || null;') >= 1
    assert body.count('camRemote = media.remote || null;') >= 1


def test_page_uses_same_origin_url_for_remote():
    """中继地址必须是 `/api/...` 相对路径：绝对 http:// 会被 https 页面拦（混合内容）。"""
    body = text()
    start = body.index('function camSrcURL()')
    end = body.index('function camFallbackToRemote()')
    block = body[start:end]
    # 注释里会出现 "http://" 字样，只看代码行
    code = '\n'.join(ln for ln in block.splitlines()
                     if not ln.strip().startswith('//'))
    assert 'camRemote.url + ' in code
    assert "'http://' +" not in code and '"http://" +' not in code, \
        '中继分支不许拼绝对地址'
    assert "profile=low" in block, '中继必须走低码率档（云端出网按客户数算钱）'


def test_page_falls_back_when_direct_stream_shows_nothing():
    body = text()
    assert 'function camFallbackToRemote()' in body
    assert 'camRemote.online' in body, '车端中继不在线就别切（切了也是黑屏）'
    assert 'naturalWidth > 0' in body, '2.5 s 探针必须先确认真的没出图'
    assert '}, 2500);' in body


def test_page_refreshes_short_lived_token_before_switching():
    """观看令牌只有 3 分钟：切中继前必须重新要一张，否则 401 全黑。"""
    body = text()
    assert 'async function camRefreshMedia()' in body
    assert body.count('camRefreshMedia().then(') >= 2, '手动/导航两条路都要刷新令牌'


def test_page_error_paths_try_remote_before_giving_up():
    body = text()
    cam_err = body[body.index('function camImgErr()'):body.index('function camReload()')]
    assert 'camFallbackToRemote()' in cam_err
    nav_err = body[body.index('function navCamImgErr()'):body.index('function navCamImgErr()') + 400]
    assert 'camFallbackToRemote()' in nav_err


def test_page_handles_robot_without_lan_address():
    """车只挂 4G 时 `/media` 不再 404，而是给你空的 `url` + 可用的 `remote`。"""
    body = text()
    assert 'if (!camMedia.url) camViaRemote = true;' in body
    assert 'if (!media.url) camViaRemote = true;' in body
