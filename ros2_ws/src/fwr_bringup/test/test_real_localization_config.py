from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def test_saved_map_localization_uses_real_frames_and_filtered_scan():
    config = yaml.safe_load((ROOT / 'config' / 'real_localization.yaml').read_text())
    amcl = config['amcl']['ros__parameters']
    assert amcl['use_sim_time'] is False
    assert (amcl['global_frame_id'], amcl['odom_frame_id'], amcl['base_frame_id']) == (
        'map', 'odom', 'base_footprint')
    assert amcl['scan_topic'] == '/scan_mapping'
    assert amcl['tf_broadcast'] is True
    # 实机走廊环境：粒子与观测数量要足够，且打开随机粒子注入以便从错误收敛恢复。
    assert amcl['min_particles'] >= 1000
    assert amcl['max_particles'] >= 4000
    assert amcl['max_beams'] >= 90
    assert amcl['laser_max_range'] <= 8.0
    assert amcl['recovery_alpha_slow'] > 0.0
    assert amcl['recovery_alpha_fast'] > 0.0
    assert config['map_server']['ros__parameters']['use_sim_time'] is False


def test_localization_launch_has_one_map_odom_owner_and_motion_disabled():
    mapping = (ROOT / 'launch' / 'real_mapping.launch.py').read_text()
    launch = (ROOT / 'launch' / 'real_localization.launch.py').read_text()
    assert "DeclareLaunchArgument('start_slam', default_value='true')" in mapping
    assert "condition=IfCondition(LaunchConfiguration('start_slam'))" in mapping
    # start_slam 现在由 localizer 参数决定：amcl 模式为 false，slam 模式为 true。
    assert "'start_slam': start_slam" in launch
    assert "'motion_enabled': False" in mapping
    assert 'map_server' in launch and 'amcl' in launch
    assert 'lifecycle_manager' in launch
    assert 'real_map_merged.yaml' in launch


def test_localization_supports_amcl_or_slam_toolbox_localization():
    """localizer:=slam 用 slam_toolbox localization 模式加载 posegraph，边定位边修图。"""
    src = (ROOT / 'launch' / 'real_localization.launch.py').read_text()
    assert "DeclareLaunchArgument('localizer', default_value='amcl')" in src
    assert "real_slam_localization.yaml" in src
    assert 'condition=amcl_selected' in src
    cfg = yaml.safe_load(
        (ROOT / 'config' / 'real_slam_localization.yaml').read_text())['slam_toolbox']['ros__parameters']
    assert cfg['mode'] == 'localization'
    assert cfg['map_file_name'].endswith('real_map_2026-10-28')


def test_navigation_and_localization_use_the_same_default_map():
    """两个 launch 的默认地图必须一致；否则导航栈会悄悄加载旧图。"""
    import re
    root = ROOT / 'launch'
    pattern = re.compile(r"DeclareLaunchArgument\('map',\s*default_value=str\(share / 'maps' / '([^']+)'\)\)")
    maps = []
    for name in ('real_navigation.launch.py', 'real_localization.launch.py'):
        found = pattern.search((root / name).read_text())
        assert found, f'{name} has no map default'
        maps.append(found.group(1))
    assert maps[0] == maps[1], f'map defaults differ: {maps}'
