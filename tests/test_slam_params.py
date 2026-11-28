"""建图参数回归（2026-11-28「快建完时整张图突然转 ~30° 然后重影」事故）。

那次的机理链条：
  建图摇杆转向上限 0.60 rad/s（页面 `MAP_MAX_ANGULAR`）
  × 扫描处理最小间隔 `minimum_time_interval` 0.5 s = 相邻两帧差 17.2°；
  而粗搜索窗口 `coarse_search_angle_offset` 只有 ±0.349 rad（±20°），
  余量仅 1.17 倍 → 匹配容易锁到隔壁错峰 → 位姿一步跳十几到二十几度 → 后续扫描全插错位姿。

这组参数的保护点是「帧间角差 ≪ 粗搜索窗口」，以及「别把粗搜索窗口往小改」。
"""

from __future__ import annotations

import math
import re
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
SLAM_YAML = ROOT / 'ros2_ws/src/fwr_bringup/config/real_slam.yaml'
PAGE = ROOT / 'cloud/web/car.html'


def _params():
    return yaml.safe_load(SLAM_YAML.read_text(encoding='utf-8'))['slam_toolbox']['ros__parameters']


def _map_turn_cap():
    text = PAGE.read_text(encoding='utf-8')
    m = re.search(r'MAP_MAX_LINEAR\s*=\s*([0-9.]+)\s*,\s*MAP_MAX_ANGULAR\s*=\s*([0-9.]+)', text)
    assert m, '页面里找不到 MAP_MAX_LINEAR / MAP_MAX_ANGULAR'
    return float(m.group(1)), float(m.group(2))


def test_processed_scan_angular_step_is_small():
    p = _params()
    _, w_max = _map_turn_cap()
    step_deg = math.degrees(p['minimum_time_interval'] * w_max)
    assert p['minimum_time_interval'] <= 0.3, (
        'minimum_time_interval 又被调大了：0.3 是上限，'
        '再大就会让转弯时相邻帧差超过 10°（事故版本是 0.5 s / 17.2°）')
    assert step_deg <= 11.0, f'转弯帧间角差 {step_deg:.1f}° 太大（事故版本 17.2°）'


def test_coarse_search_window_covers_the_step_with_margin():
    p = _params()
    _, w_max = _map_turn_cap()
    step = p['minimum_time_interval'] * w_max
    offset = p['coarse_search_angle_offset']
    assert offset >= 1.7 * step, (
        f'粗搜索窗口 ±{offset:.3f} rad 对帧间角差 {step:.3f} rad 余量不足 1.7 倍，'
        '真值会贴在窗口边缘 → 跳错峰（事故版本余量 0.349/0.300 = 1.17）')
    # 反方向也钉死：不许为了"更稳"把窗口收到比一帧角差还小，那样直接匹配不到。
    assert offset > step * 1.2, '粗搜索窗口不能小于帧间角差，否则真值落在窗口外'


def test_loop_closure_is_strict_enough():
    p = _params()
    assert p['loop_match_minimum_response_coarse'] >= 0.45, (
        '假回环会把整张图旋转/平移（事故症状），粗匹配门槛不许再降回 0.35')
    assert p['loop_match_minimum_response_fine'] >= 0.60
    assert p['loop_match_minimum_response_fine'] > p['loop_match_minimum_response_coarse']


def test_graph_node_thresholds_stay_out_of_the_crash_zone():
    """minimum_travel_heading 低于 0.1 会让 mapper abort（上游 issue #642），不许动。"""
    p = _params()
    assert p['minimum_travel_heading'] >= 0.25
    assert p['minimum_travel_distance'] >= 0.25
    assert p['scan_buffer_size'] <= 10
