"""4G 备份链路维护脚本的纯逻辑测试（tools/fwr_4g_net.py）。

这些函数决定「哪块网卡是 RNDIS」「哪条默认路由要删」「要不要重新注册」，
都是踩过坑的地方，用样例输出钉死，避免以后改坏。
"""

from pathlib import Path
import sys

root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root / "tools"))
import fwr_4g_net as net  # noqa: E402


def test_rndis_recognised_by_mac():
    ifaces = {"enx020c29a39b6d": ("02:0c:29:a3:9b:6d", "cdc_ether")}
    assert net.rndis_names(ifaces) == ["enx020c29a39b6d"]


def test_rndis_recognised_by_driver_when_mac_differs():
    ifaces = {"enxdeadbeef00": ("aa:bb:cc:dd:ee:ff", "cdc_ether")}
    assert net.rndis_names(ifaces) == ["enxdeadbeef00"]


def test_plain_ethernet_is_not_rndis():
    ifaces = {
        "enp44s0": ("00:11:22:33:44:55", "e1000e"),
        "enx207bd51a0d30": ("aa:bb:cc:00:11:22", "r8152"),
        "wlo1": ("70:9c:d1:11:22:33", "iwlwifi"),
    }
    assert net.rndis_names(ifaces) == []


def test_rndis_defaults_picks_only_that_interface():
    text = (
        "default via 192.168.1.1 dev wlo1 proto dhcp metric 600 \n"
        "default via 192.168.43.1 dev enx020c29a39b6d proto dhcp metric 20100 \n"
        "default dev ppp0 proto static scope link metric 20700 \n"
    )
    assert net.rndis_defaults(text, "enx020c29a39b6d") == [
        "default via 192.168.43.1 dev enx020c29a39b6d proto dhcp metric 20100"
    ]
    assert net.rndis_defaults(text, "wlo1") == ["default via 192.168.1.1 dev wlo1 proto dhcp metric 600"]
    assert net.rndis_defaults(text, "ppp0") == ["default dev ppp0 proto static scope link metric 20700"]


def test_rndis_defaults_empty_when_clean():
    text = "default via 192.168.1.1 dev wlo1 proto dhcp metric 600\ndefault dev ppp0 metric 20700\n"
    assert net.rndis_defaults(text, "enx020c29a39b6d") == []


def test_needs_register_true_when_idle_or_detached():
    assert net.needs_register({"registration": "idle", "packet": "attached"}) is True
    assert net.needs_register({"registration": "home", "packet": "detached"}) is True
    assert net.needs_register({"registration": "idle", "packet": "detached"}) is True


def test_needs_register_false_when_healthy():
    assert net.needs_register({"registration": "home", "packet": "attached"}) is False


def test_needs_register_treats_missing_fields_as_broken():
    assert net.needs_register({}) is True


def test_cooldown_window():
    assert net.cooldown_ready(0, 10_000) is True
    assert net.cooldown_ready(1000, 1000 + net.COOLDOWN_S - 1) is False
    assert net.cooldown_ready(1000, 1000 + net.COOLDOWN_S) is True


def test_modem_fields_parse_mmcli_output():
    text = """  -------------------------
  Status   |                state: connected
           |          power state: on
           |          registration: idle
           |    operator name: CHINA BROADNET
           | packet service state: detached
  -------------------------"""
    fields = net.modem_fields(text)
    assert fields["state"] == "connected"
    assert fields["power state"] == "on"
    assert fields["registration"] == "idle"
    assert fields["operator name"] == "CHINA BROADNET"
    assert fields["packet"] == "detached"
    assert net.needs_register(fields) is True


def test_autoconnect_accepts_english_and_chinese():
    assert net.autoconnect_enabled("yes\n") is True
    assert net.autoconnect_enabled("是\n") is True
    assert net.autoconnect_enabled("no\n") is False
    assert net.autoconnect_enabled("否\n") is False
    assert net.autoconnect_enabled("") is False


def test_needs_register_accepts_chinese_values():
    assert net.needs_register({"registration": "已注册", "packet": "已附着"}) is False


def test_autoconnect_strips_terse_field_prefix():
    assert net.autoconnect_enabled("connection.autoconnect:yes\n") is True
    assert net.autoconnect_enabled("connection.autoconnect:no\n") is False
    assert net.autoconnect_enabled("connection.autoconnect:是\n") is True


def test_required_streak_acts_immediately_when_4g_is_primary():
    assert net.required_streak("ppp0") == 1


def test_required_streak_waits_when_4g_is_only_backup():
    assert net.required_streak("wlo1") == net.BACKUP_FAIL_STREAK
    assert net.required_streak("") == net.BACKUP_FAIL_STREAK
