from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[1]


def test_monitor_is_only_automatic_bridge_input():
    bridge = (ROOT / 'fwr_bringup' / 'base_bridge.py').read_text()
    assert "create_subscription(Twist, '/cmd_vel'," not in bridge
    assert "create_subscription(Twist, '/cmd_vel_auto_safe'," in bridge
    forward = yaml.safe_load((ROOT / 'config' / 'real_collision_monitor_forward.yaml').read_text())['collision_monitor_forward']['ros__parameters']
    reverse = yaml.safe_load((ROOT / 'config' / 'real_collision_monitor_reverse.yaml').read_text())['collision_monitor_reverse']['ros__parameters']
    for cfg in (forward, reverse):
        assert cfg['cmd_vel_in_topic'] == '/cmd_vel_auto_unchecked'
        assert cfg['base_frame_id'] == 'base_footprint'
        assert .5 <= cfg['source_timeout'] <= 1.0
        assert 'scan' in cfg['observation_sources']
        assert cfg['scan']['topic'] == '/scan'
        assert 17.0 <= cfg['stop_pub_timeout'] <= 20.0
    assert forward['cmd_vel_out_topic'] == '/cmd_vel_auto_forward_filtered'
    assert reverse['cmd_vel_out_topic'] == '/cmd_vel_auto_reverse_filtered'
    assert forward['polygons'] == ['front_stop', 'front_slow']
    assert reverse['polygons'] == ['rear_visible_stop']
    assert forward['front_stop']['action_type'] == 'stop'
    assert forward['front_slow']['action_type'] == 'slowdown'
    # At the 0.10 m/s trial cap, a 0.4 slowdown output was below the loaded
    # chassis' observed starting speed; keep a usable crawl before stop zone.
    assert .6 <= forward['front_slow']['slowdown_ratio'] <= .8
    assert reverse['rear_visible_stop']['action_type'] == 'stop'
    # Keep zero flowing through Nav2's 15 s progress check, then require a
    # bounded timeout so loss of that stream disarms the bridge.
    launch = (ROOT / 'launch' / 'real_navigation.launch.py').read_text()
    assert "'collision_monitor_forward', 'collision_monitor_reverse'" in launch
    assert "executable='directional_safety_selector'" in launch
    assert 'real_collision_monitor.yaml' not in launch


def test_collision_stop_zones_keep_in_place_rotation_usable():
    """Stop zones must hug the chassis edge so a pivot in a corner is not frozen."""
    forward = yaml.safe_load(
        (ROOT / 'config' / 'real_collision_monitor_forward.yaml').read_text()
    )['collision_monitor_forward']['ros__parameters']
    reverse = yaml.safe_load(
        (ROOT / 'config' / 'real_collision_monitor_reverse.yaml').read_text()
    )['collision_monitor_reverse']['ros__parameters']
    fx = [forward['front_stop']['points'][i] for i in (0, 2, 4, 6)]
    assert min(fx) >= 0.30, 'front stop zone must start at the chassis front edge'
    assert max(fx) - min(fx) <= 0.20, 'front stop zone deeper than 0.20 m freezes pivots'
    fy = [abs(forward['front_stop']['points'][i]) for i in (1, 3, 5, 7)]
    assert 0.22 <= max(fy) <= 0.26, (
        'front stop half-width must just clear the 0.215 m body, otherwise a '
        'passable narrow doorway triggers it')
    rx = [reverse['rear_visible_stop']['points'][i] for i in (0, 2, 4, 6)]
    assert max(rx) <= -0.30, 'rear stop zone must start at the chassis rear edge'
    assert max(rx) - min(rx) <= 0.20, 'rear stop zone deeper than 0.20 m freezes pivots'


def test_real_launch_defaults_closed_and_one_slam():
    src = (ROOT / 'launch' / 'real_navigation.launch.py').read_text()
    assert src.count('real_mapping.launch.py') == 1
    assert 'frontier_explorer' in src
    assert 'auto_command_gate' in src
    assert 'nav2_collision_monitor' in src
    assert "'autostart': 'false'" in src
    assert "'motion_enabled': False" in (ROOT / 'launch' / 'real_mapping.launch.py').read_text()


def test_real_planner_stays_in_known_free_space_and_rearms_after_link_loss():
    cfg = yaml.safe_load((ROOT / 'config' / 'real_nav2.yaml').read_text())
    assert cfg['planner_server']['ros__parameters']['GridBased']['allow_unknown'] is False
    bridge = (ROOT / 'fwr_bringup' / 'base_bridge.py').read_text()
    assert "'/base/arm_auto'" in bridge
    assert "'/base/auto_armed'" in bridge
    assert "'require_bridge_arm': True" in (ROOT / 'launch' / 'real_navigation.launch.py').read_text()


def test_mapping_scan_is_separate_from_raw_safety_scan():
    mapping = (ROOT / 'launch' / 'real_mapping.launch.py').read_text()
    navigation = (ROOT / 'launch' / 'real_navigation.launch.py').read_text()
    slam = yaml.safe_load((ROOT / 'config' / 'real_slam.yaml').read_text())
    nav = yaml.safe_load((ROOT / 'config' / 'real_nav2.yaml').read_text())
    assert mapping.count("executable='scan_mapping_filter'") == 1
    assert slam['slam_toolbox']['ros__parameters']['scan_topic'] == '/scan_mapping'
    for key in ('local_costmap', 'global_costmap'):
        assert nav[key][key]['ros__parameters']['obstacle_layer']['scan']['topic'] == '/scan_mapping'
    assert "'scan_topic': '/scan'" in navigation
    assert '/scan_mapping' in (ROOT / 'config' / 'real_mapping.rviz').read_text()
    for direction in ('forward', 'reverse'):
        cfg = yaml.safe_load((ROOT / 'config' / f'real_collision_monitor_{direction}.yaml').read_text())
        assert cfg[f'collision_monitor_{direction}']['ros__parameters']['scan']['topic'] == '/scan'
