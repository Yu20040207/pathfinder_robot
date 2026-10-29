from pathlib import Path
import xml.etree.ElementTree as ET
import yaml

ROOT = Path(__file__).resolve().parents[1]


def test_auto_sim_bridge_and_single_map_owner():
    src = (ROOT / 'launch' / 'corridor_sim.launch.py').read_text()
    assert "DeclareLaunchArgument('auto_mapping', default_value='false')" in src
    assert 'corridor_auto_bridge.yaml' in src
    assert 'frontier_explorer' in src
    assert "DeclareLaunchArgument('auto_explorer', default_value='true')" in src
    assert "LaunchConfiguration('auto_explorer')" in src
    assert 'collision_monitor' in src
    assert 'sim_scan_relay' in src
    assert "'topic': '/scan_safety'" in src
    assert "'false'" in src  # default disarmed
    bridge = yaml.safe_load((ROOT / 'config' / 'corridor_auto_bridge.yaml').read_text())
    cmd = [x for x in bridge if x['gz_topic_name'] == '/cmd_vel']
    assert len(cmd) == 1
    assert cmd[0]['ros_topic_name'] == '/cmd_vel_sim_enforced'
    assert cmd[0]['direction'] == 'ROS_TO_GZ'
    scan = [x for x in bridge if x['gz_topic_name'] == '/scan']
    assert len(scan) == 1 and scan[0]['ros_topic_name'] == '/scan_raw'
    relay = (ROOT / 'fwr_bringup' / 'sim_scan_relay.py').read_text()
    assert "'/scan_safety'" in relay


def test_auto_sim_nav2_speed_matches_gate_limit():
    cfg = yaml.safe_load((ROOT / 'config' / 'corridor_auto_nav2.yaml').read_text())
    # The planned detour must begin before the forward stop polygon reaches an obstacle.
    for name in ('local_costmap', 'global_costmap'):
        params = cfg[name][name]['ros__parameters']
        assert params['inflation_layer']['inflation_radius'] >= .9
    assert cfg['controller_server']['ros__parameters']['FollowPath']['desired_linear_vel'] <= .08
    assert cfg['velocity_smoother']['ros__parameters']['max_velocity'][0] <= .10
    assert cfg['velocity_smoother']['ros__parameters']['min_velocity'][0] == 0
    assert cfg['planner_server']['ros__parameters']['GridBased']['allow_unknown'] is False
    real = yaml.safe_load((ROOT / 'config' / 'real_nav2.yaml').read_text())
    sim_progress = cfg['controller_server']['ros__parameters']['progress_checker']
    real_progress = real['controller_server']['ros__parameters']['progress_checker']
    assert sim_progress['required_movement_radius'] == real_progress['required_movement_radius']
    assert sim_progress['movement_time_allowance'] == real_progress['movement_time_allowance']
    for direction in ('forward', 'reverse'):
        monitor = yaml.safe_load((ROOT / 'config' / f'real_collision_monitor_{direction}.yaml').read_text())
        assert (monitor[f'collision_monitor_{direction}']['ros__parameters']['stop_pub_timeout'] >=
                sim_progress['movement_time_allowance'] + 2.)
    launch = (ROOT / 'launch' / 'corridor_sim.launch.py').read_text()
    assert "'collision_monitor_forward', 'collision_monitor_reverse'" in launch
    assert "executable='directional_safety_selector'" in launch
    assert "'scan_topic': '/scan_safety'" in launch


def test_dynamic_obstacle_starts_outside_the_corridor():
    world = ET.parse(ROOT / 'worlds' / 'corridor_c1.sdf')
    obstacle = world.find(".//model[@name='path_obstacle']")
    assert obstacle is not None
    assert float(obstacle.findtext('pose').split()[0]) > 20


def test_auto_sim_can_reuse_saved_map_for_fixed_goal_reroute():
    launch = (ROOT / 'launch' / 'corridor_sim.launch.py').read_text()
    assert "DeclareLaunchArgument('auto_saved_map', default_value='false')" in launch
    assert "'static_map': LaunchConfiguration('auto_saved_map')" in launch
    assert "'auto_saved_map'), \"' == 'true'\"" in launch
    reroute = (ROOT.parents[2] / 'scripts' / 'check_fixed_goal_reroute_sim.py').read_text()
    assert "'auto_saved_map:=true'" in reroute
