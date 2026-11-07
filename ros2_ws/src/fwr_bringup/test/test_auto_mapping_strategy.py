"""Guard: 1A simulation improvements must be wired into the real auto-mapping side."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_viewpoint_core_exposes_clusters_and_heading():
    src = (ROOT / 'fwr_bringup' / 'viewpoint_core.py').read_text()
    assert 'def frontier_clusters(' in src
    assert 'turn_weight' in src and 'def pick_next_viewpoint' in src


def test_planner_uses_clusters_and_heading_without_touching_navigation():
    src = (ROOT / 'fwr_bringup' / 'viewpoint_planner.py').read_text()
    assert 'vc.frontier_clusters(' in src
    assert "get_parameter('turn_weight')" in src
    assert 'heading=self._robot_heading()' in src
    assert 'origin=cell' in src


def test_auto_mapping_config_has_turn_weight_only():
    text = (ROOT / 'config' / 'real_viewpoint_planner.yaml').read_text()
    assert 'turn_weight: 0.4' in text
    # Manual mapping / navigation configs stay untouched by this feature.
    nav = (ROOT / 'config' / 'real_nav2.yaml').read_text()
    assert 'turn_weight' not in nav


def test_room_geofence_is_wired_everywhere():
    planner = (ROOT / 'fwr_bringup' / 'viewpoint_planner.py').read_text()
    assert "declare_parameter('max_goal_distance_m'" in planner
    assert 'def _inside_geofence(' in planner
    assert 'geofence-reject' in planner
    assert 'any(self._inside_geofence' in planner
    real = (ROOT / 'launch' / 'real_auto_mapping.launch.py').read_text()
    mission = (ROOT / 'launch' / 'auto_mapping_mission.launch.py').read_text()
    assert "'max_goal_distance_m'" in real
    assert "'max_goal_distance_m'" in mission


def test_mission_hard_stops_on_base_fault():
    mission = (ROOT / 'fwr_bringup' / 'auto_mapping_mission.py').read_text()
    assert "'/base/fault'" in mission
    assert 'base_fault' in mission
    assert 'Int32' in mission


def test_adaptive_room_split_is_wired():
    planner = (ROOT / 'fwr_bringup' / 'viewpoint_planner.py').read_text()
    assert "declare_parameter('room_aware'" in planner
    assert 'segment_room(' in planner
    assert 'def _inside_room(' in planner
    assert 'self._update_room_mask(free, cell)' in planner
    real = (ROOT / 'launch' / 'real_auto_mapping.launch.py').read_text()
    mission = (ROOT / 'launch' / 'auto_mapping_mission.launch.py').read_text()
    assert "'room_aware'" in real and "'room_aware'" in mission
    assert "'door_clearance_m'" in real and "'door_clearance_m'" in mission


def test_room_warmup_minimum_is_wired():
    planner = (ROOT / 'fwr_bringup' / 'viewpoint_planner.py').read_text()
    assert "declare_parameter('room_min_cells'" in planner
    assert 'core not stable yet' in planner
    mission = (ROOT / 'launch' / 'auto_mapping_mission.launch.py').read_text()
    assert "'room_min_cells'" in mission


def test_chain_watchdog_is_optional_for_room_runs():
    real = (ROOT / 'launch' / 'real_auto_mapping.launch.py').read_text()
    mission = (ROOT / 'launch' / 'auto_mapping_mission.launch.py').read_text()
    assert "DeclareLaunchArgument('chain_watchdog'" in real
    assert "IfCondition(LaunchConfiguration('chain_watchdog'))" in real
    assert "'chain_watchdog'" in mission


def test_bootstrap_is_exempt_from_small_room_mask():
    planner = (ROOT / 'fwr_bringup' / 'viewpoint_planner.py').read_text()
    assert 'check_room=False' in planner
    assert 'allow_outside_room=True' in planner


def test_map_resize_invalidates_observable_cache():
    planner = (ROOT / 'fwr_bringup' / 'viewpoint_planner.py').read_text()
    assert 'self._observable = None' in planner
    assert 'self._visible_cache = None' in planner
