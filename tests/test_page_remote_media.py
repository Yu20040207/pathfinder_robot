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
    assert "'&profile=' + camProfile()" in block, \
        '中继必须带画质档（高清 low / 弱网 eco），云端出网按客户数算钱'


def test_page_falls_back_when_direct_stream_shows_nothing():
    body = text()
    assert 'function camFallbackToRemote()' in body
    assert 'camRemote.online' in body, '车端中继不在线就别切（切了也是黑屏）'
    assert 'naturalWidth > 0' in body, '2.5 s 探针必须先确认真的没出图'
    assert '}, 1500);' in body, '异地开始出画面要快，别等 2.5 s'


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


def test_page_has_two_remote_quality_tiers():
    """2026-11-25 用户报「手机 4G 延迟大、偶尔卡住」：异地必须能切到弱网档。

    4G 下 2.3 Mbps 的 480p 会把手机下行塞满（实测限速 1.2 Mbps 时端到端延迟
    从 2 ms 涨到 10 s），所以页面要能发 `?profile=eco`（320 宽 10 fps ≈ 0.5 Mbps）。
    """
    body = text()
    assert 'function camProfile()' in body
    assert "return camEco ? 'eco' : 'low';" in body
    assert "&profile=' + camProfile() + '&t='" in body
    # 默认按接入类型自动选（iOS 没有 navigator.connection → 当弱网）
    assert 'navigator.connection' in body
    assert 'function camAutoEco()' in body
    # 手动切档的键
    assert 'id="camQuality"' in body
    assert "camEco = camProfile() === 'eco' ? false : true;" in body


def test_page_refreshes_token_when_switching_camera():
    """切摄像头时令牌可能已过期（180 s）→ 401 → 黑屏（云端日志实锤 3 次）。"""
    body = text()
    assert body.count('if (camViaRemote) camRefreshMedia().then(camReload);') == 1
    assert body.count('if (camViaRemote) camRefreshMedia().then(reload);') == 1


def test_follow_mode_works_over_relay():
    """2026-11-26 用户报「切到跟随模式画面是黑的」：跟随模式原来只认局域网地址。

    现在视频走 `followVideoURL()`（异地 → 同源 `/api/...`），面板接口走
    `followFetch()`（异地 → `/api/devices/<id>/vision/...`），并且**固定主摄**
    （人体识别只跑 HP60C 那一路）。
    """
    body = text()
    assert 'function followRemote()' in body
    assert 'function followVideoURL()' in body
    assert 'function followFetch(path)' in body
    assert "?cam=main&profile=' + camProfile()" in body
    assert "api('/api/devices/' + DEVICE + '/vision' + path)" in body
    # 所有面板接口都不许再直连局域网
    assert 'fetch(followBase() + ' in body        # 只允许 followFetch 内部这一处
    assert body.count('fetch(followBase() + ') == 1
    # 进跟随模式必须把中继信息一起存下来，否则 followRemote() 永远为假
    assert 'followMedia = m;' in body and 'camRemote = m.remote || camRemote;' in body
    # 视频挂载统一走 camMount（换新节点断旧长连接）
    assert "camMount('followImg', followVideoURL(), true)" in body


def test_follow_picture_sits_at_top_like_other_modes():
    """2026-11-26 用户要求「跟随模式画面放到最上面，跟其他模式的画面一样」。

    原来 4:3 画面在整块区域里垂直居中 → 上下各有黑边。现在图像内容顶对齐，
    画布矩形也必须同步从 y=0 起算（否则人体框整体下移半个黑边）。
    顶部提示条同时改成导航模式那种**全透明浮层**（文字压在画面上，靠描边）。
    """
    body = text()
    assert 'object-position: top center' in body
    assert 'return { x: (w - iw) / 2, y: 0, w: iw, h: ih };' in body
    # followTop 不许再有半透明底色（要和 #navTopWrap 一致）
    assert 'background: rgba(13,17,23,.62)' not in body
    head = body.split('#followTop {')[1].split('}')[0]
    assert 'background: transparent' in head
    assert 'text-shadow' in body.split('#followTop #followHint {')[1].split('}')[0]
