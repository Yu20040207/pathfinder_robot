"""重启主控后「等待导航栈」的提示（2026-11-24 10:02 现场截图）。

重启后导航栈要 40~90 s 才出地图/雷达数据，`auto_initial_pose` 这时回
「还没有地图或激光数据（已重新订阅，请 3 秒后再点一次）」。旧正则没覆盖这句，
被当成**定位失败**：8 秒一次、3 次（24 秒）就用完，直接弹「自动定位失败」。
另外那条长提示换行后压住了下面的绿色状态条。
"""
from pathlib import Path

PAGE = Path(__file__).resolve().parents[1] / 'cloud' / 'web' / 'car.html'


def _src():
    return PAGE.read_text(encoding='utf-8')


def test_startup_reason_is_not_a_localization_failure():
    s = _src()
    assert '还没有地图|激光数据|重新订阅' in s, '启动期原因必须归到 stackIssue'
    assert 'relocFails = 0;            // 启动期不算定位失败' in s
    assert '导航栈正在启动，等地图和雷达数据就绪后自动重新定位' in s


def test_top_hints_are_stacked_not_overlapping():
    s = _src()
    assert '#navTopWrap { position: absolute;' in s
    assert 'display: flex; flex-direction: column;' in s
    # 两个提示不再是各自绝对定位（否则长提示会压住绿条）
    assert '#navInfo { position: static;' in s
    assert '#navStatus { position: static;' in s
    # 结构上必须在同一个容器里，且顺序为 提示 → 状态
    i_wrap = s.index('<div id="navTopWrap">')
    i_info = s.index('<div id="navInfo">', i_wrap)
    i_stat = s.index('<div id="navStatus">', i_wrap)
    assert i_wrap < i_info < i_stat
