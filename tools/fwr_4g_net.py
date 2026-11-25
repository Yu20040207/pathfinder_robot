#!/usr/bin/env python3
"""4G 备份链路维护：清掉 RNDIS 抢路由 + 链路静默死亡自愈（2026-11-25 现场定制）。

为什么需要（两条都是实机踩出来的）：

1) Quectel EC200A 除了 ppp0（真正的数据口）还会暴露一个 **RNDIS 网口**
   （MAC `02:0c:29:a3:9b:6d`，地址 192.168.43.100，网关 192.168.43.1）。
   它自带 **metric 20100** 的默认路由，比 ppp0 的 **20700** 更优 ——
   WiFi 一断，默认路由就落到这个**不通外网**的网口上，手机端看到的是「车失联」。
   所以每轮维护先把它清掉，并把网口 down 掉（不 down 的话 DHCP 一续租又把路由加回来）。

2) 4G 会静默死亡：`ppp0` 还在、ModemManager 说 connected，但
   `registration: idle`、`packet service: detached`、回包 0。
   只有 `mmcli --3gpp-register-home` + 重拨能救回来（现场人工自愈一次 ~93 s）。

本脚本**不碰 WiFi、不碰 ROS、不重启任何 fwr-\* 服务**，只动 ppp0 / RNDIS 网口 /
`4g` 这个 NetworkManager 连接。

用法（由 systemd 调用，见 deploy/systemd/）：
    fwr_4g_net.py maintain      # 一轮维护：清 RNDIS 路由 + 4G 健康检查/自愈（默认）
    fwr_4g_net.py fix-rndis     # 只清 RNDIS 抢路由
    fwr_4g_net.py watch         # 只看 4G
    任意模式加 --dry-run        # 只打印决策，不动网络
"""

from __future__ import annotations

import json
import os
import re
import shlex
import subprocess
import sys
import time

PUBLIC_PING = "223.5.5.5"  # 阿里公共 DNS，只用来判断「4G 回包通不通」
PPP_IFACE = "ppp0"
CON_NAME = "4g"
RNDIS_MAC = "02:0c:29:a3:9b:6d"
STATE_PATH = "/run/fwr-4g-net.json"
LOCK_PATH = "/run/fwr-4g-net.lock"
COOLDOWN_S = 300  # 两次自愈尝试之间的最小间隔（5 分钟）
BACKUP_FAIL_STREAK = 3  # 4G 只是备用（默认路由不在 ppp0）时，连续失败几轮才动手
PING_TRIES = 3  # 判定「不通」前连测几次
PING_GAP_S = 4


def log(msg: str) -> None:
    print(time.strftime("[%H:%M:%S] ") + msg, flush=True)


def run(cmd, timeout=20):
    """跑一条命令；超时/命令不存在都当失败返回，不抛异常。

    ⚠️ 统一用 LC_ALL=C：车机是中文 locale，nmcli/mmcli 会把 `yes`、
    `registration: home` 这类字段翻译成中文，直接字符串比较会误判
    （2026-11-25 首装就踩到：autoconnect 读出来是「是」，脚本当成「没开」直接跳过）。
    """
    env = dict(os.environ, LC_ALL="C", LANG="C")
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, env=env)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return subprocess.CompletedProcess(cmd, 1, "", str(exc))


# ---------------------------------------------------------------- 纯函数（有单测）

def rndis_names(ifaces) -> list[str]:
    """挑出 4G 模块的 RNDIS 网口：认 MAC，或「enx 开头 + cdc_ether 驱动」。"""
    return [
        name
        for name, (mac, driver) in ifaces.items()
        if mac == RNDIS_MAC or (name.startswith("enx") and driver == "cdc_ether")
    ]


def rndis_defaults(route_text: str, name: str) -> list[str]:
    """从 `ip route show default` 输出里挑出走这个网口的默认路由行。"""
    return [
        line.strip()
        for line in route_text.splitlines()
        if line.strip().startswith("default") and f"dev {name}" in line
    ]


OK_REGISTRATION = {"home", "已注册", "本地"}  # home / 中文 locale
OK_PACKET = {"attached", "已附着"}


def needs_register(fields: dict) -> bool:
    """ModemManager 的注册/附着状态不正常时返回 True（需要重新注册）。"""
    registration = (fields.get("registration") or "").strip().lower()
    packet = (fields.get("packet") or "").strip().lower()
    return registration not in OK_REGISTRATION or packet not in OK_PACKET


def required_streak(primary_dev: str, backup_streak: int = BACKUP_FAIL_STREAK) -> int:
    """需要连续失败几轮才自愈。

    4G 是**唯一**在用的链路（默认路由就在 ppp0）时立刻动手（1 轮，约 8 s）；
    只是备用链路时等 3 轮（约 3 分钟），避免一次 20 s 的闪断就重拨。
    """
    return 1 if primary_dev == PPP_IFACE else backup_streak


def cooldown_ready(last_ts: float, now: float, cooldown_s: int = COOLDOWN_S) -> bool:
    """距上次自愈是否已过冷却期。"""
    return last_ts <= 0 or (now - last_ts) >= cooldown_s


def modem_fields(text: str) -> dict:
    """解析 `mmcli -m N` 的竖排字段（`|  key: value`）。

    `packet service state` 统一缩成 `packet`，调用方只认这一个键。
    """
    out = {}
    for raw in text.splitlines():
        if "|" not in raw:
            continue
        left, sep, value = raw.split("|", 1)[1].strip().partition(":")
        if sep:
            out[left.strip()] = value.strip()
    if "packet service state" in out:
        out["packet"] = out["packet service state"]
    return out


# ---------------------------------------------------------------- 真实环境读取

def read_ifaces(root="/sys/class/net") -> dict:
    """返回 {网卡名: (MAC, 驱动名)}；读不到的键给空串。"""
    out = {}
    try:
        names = os.listdir(root)
    except OSError:
        return out
    for name in names:
        if name == "lo":
            continue
        mac = driver = ""
        try:
            with open(os.path.join(root, name, "address"), encoding="utf-8") as fh:
                mac = fh.read().strip().lower()
        except OSError:
            pass
        try:
            driver = os.path.basename(os.readlink(os.path.join(root, name, "device", "driver")))
        except OSError:
            pass
        out[name] = (mac, driver)
    return out


def load_state() -> dict:
    try:
        with open(STATE_PATH, encoding="utf-8") as fh:
            return json.load(fh)
    except (OSError, ValueError):
        return {}


def save_state(**kw) -> None:
    state = load_state()
    state.update(kw)
    try:
        with open(STATE_PATH, "w", encoding="utf-8") as fh:
            json.dump(state, fh)
    except OSError as exc:
        log(f"状态写不进 {STATE_PATH}：{exc}")


def ppp_present() -> bool:
    return run(["ip", "-o", "link", "show", PPP_IFACE]).returncode == 0


def ppp_ping_once() -> bool:
    return run(["ping", "-I", PPP_IFACE, "-c", "2", "-W", "2", PUBLIC_PING], timeout=15).returncode == 0


def ppp_healthy() -> bool:
    """连测 PING_TRIES 次，任一次有回包就算健康（偶发单次丢包不算死亡）。"""
    for i in range(PING_TRIES):
        if i:
            time.sleep(PING_GAP_S)
        if ppp_ping_once():
            return True
    return False


def autoconnect_enabled(text: str) -> bool:
    """nmcli 的开关值：`-t` 模式下会带字段名前缀（`connection.autoconnect:yes`）。

    英文 yes / 中文「是」都算开着；取不到值按「没开」处理。
    """
    value = text.strip()
    if ":" in value:
        value = value.rsplit(":", 1)[1]
    return value.strip().lower() in {"yes", "是", "true", "1"}


def primary_iface() -> str:
    """当前默认路由走哪块网卡（判断 4G 是"唯一在用"还是"备用"）。"""
    out = run(["ip", "route", "get", PUBLIC_PING]).stdout
    return out.split(" dev ")[1].split()[0] if " dev " in out else ""


def wait_ppp_ok(seconds: int) -> bool:
    """轮询等 ppp0 出现且回包正常。"""
    for _ in range(seconds):
        if ppp_present() and ppp_ping_once():
            return True
        time.sleep(1)
    return False


def modem_usb_dir(vendor="2c7c", product="6005", base="/sys/bus/usb/devices") -> str:
    """找 4G 模块的 USB sysfs 目录（按 VID:PID，别写死 3-1.3.3 —— 换口就会变）。"""
    try:
        names = os.listdir(base)
    except OSError:
        return ""
    for name in names:
        try:
            with open(os.path.join(base, name, "idVendor"), encoding="utf-8") as fh:
                if fh.read().strip() != vendor:
                    continue
            with open(os.path.join(base, name, "idProduct"), encoding="utf-8") as fh:
                if fh.read().strip() == product:
                    return os.path.join(base, name)
        except OSError:
            continue
    return ""


def con_autoconnect() -> bool:
    """读 4g 连接的 autoconnect 开关。

    ⚠️ 字段名是 `connection.autoconnect`；写成 `autoconnect` 会被 nmcli 判为
    invalid field（2026-11-25 首装就踩到，脚本每轮都以为用户关了自动连接）。
    读不到时按「开着」处理：宁可多维护一次，也别整段跳过。
    """
    for field in ("connection.autoconnect", "autoconnect"):
        result = run(["nmcli", "-t", "-f", field, "con", "show", CON_NAME])
        if result.returncode == 0 and result.stdout.strip():
            return autoconnect_enabled(result.stdout)
    return True


def modem_index():
    match = re.search(r"/Modem/(\d+)", run(["mmcli", "-L"]).stdout)
    return match.group(1) if match else None


# ---------------------------------------------------------------- 两个动作

def fix_rndis(dry=False, tries=15, gap=2.0) -> int:
    """清掉 RNDIS 网口上的默认路由并 down 掉它。返回动作次数。"""
    acted = 0
    for _ in range(tries):
        names = rndis_names(read_ifaces())
        if not names:
            return acted
        dirty = False
        for name in names:
            # v4 和 v6 的默认路由都要清（NM 两族都会下发）
            for v6 in (False, True):
                show = ["ip", "-6", "route", "show", "default"] if v6 else ["ip", "route", "show", "default"]
                delete = ["ip", "-6", "route", "del"] if v6 else ["ip", "route", "del"]
                for line in rndis_defaults(run(show).stdout, name):
                    dirty = True
                    acted += 1
                    log(f"清除 RNDIS 默认路由（{name}{' v6' if v6 else ''}）：{line}")
                    if dry:
                        continue
                    result = run(delete + shlex.split(line))
                    if result.returncode != 0:
                        log(f"  删除失败：{result.stderr.strip()}")
            if dry:
                continue
            # 已经在 DOWN 就别重复动它（避免每分钟都算一次动作）
            link = run(["ip", "-o", "link", "show", name]).stdout
            if "state DOWN" not in link:
                if run(["ip", "link", "set", name, "down"]).returncode != 0:
                    log(f"down {name} 失败（可能已被别的进程占用）")
                else:
                    dirty = True
                    acted += 1
        if not dirty:
            return acted
        time.sleep(gap)
    return acted


def watch(dry=False) -> int:
    """健康检查 + 静默死亡自愈。返回 0 = 正常/已处理，1 = 自愈失败。"""
    if not con_autoconnect():
        log("4g 连接被设为不自动连接（用户改过），跳过")
        return 0
    if not ppp_present():
        log(f"{PPP_IFACE} 不在 → 拉起 {CON_NAME}")
        if dry:
            return 0
        run(["nmcli", "con", "up", CON_NAME], timeout=90)
        ok = False
        for _ in range(45):
            if ppp_present() and ppp_ping_once():
                ok = True
                break
            time.sleep(1)
        log("拉起后 4G 可用" if ok else "拉起后仍不通，等下一轮")
        return 0 if ok else 1
    state = load_state()
    if ppp_healthy():
        save_state(fail_streak=0)
        log("4G 健康")
        return 0

    streak = int(state.get("fail_streak") or 0) + 1
    save_state(fail_streak=streak)
    primary = primary_iface()
    need = required_streak(primary)
    if streak < need:
        log(f"4G 不通（连续第 {streak}/{need} 轮，默认路由在 {primary or '?'} → 先观察）")
        return 0

    last_ts = float(state.get("last_fix_ts") or 0)
    now = time.time()
    if not cooldown_ready(last_ts, now):
        log(f"4G 连续 {streak} 轮不通，但距上次自愈只有 {now - last_ts:.0f}s（冷却 {COOLDOWN_S}s），跳过")
        return 0

    idx = modem_index()
    fields = modem_fields(run(["mmcli", "-m", idx]).stdout) if idx else {}
    if dry:
        log(f"[dry-run] 4G 不通 → modem={idx} 需要重注册={needs_register(fields)} 字段={fields}")
        return 0

    log(f"4G 不通（modem={idx} state={fields.get('state')} "
        f"registration={fields.get('registration')} packet={fields.get('packet')}）→ 开始自愈")
    if idx and needs_register(fields):
        log("ModemManager 显示未注册/未附着 → --3gpp-register-home")
        result = run(["mmcli", "-m", idx, "--3gpp-register-home"], timeout=40)
        if result.returncode != 0:
            log(f"  重注册失败：{result.stderr.strip()}")
        time.sleep(8)
    # 第一级：重拨（注意 nmcli 本身可能卡住，必须靠 subprocess timeout 强杀）
    log(f"重拨 {CON_NAME}")
    run(["nmcli", "con", "down", CON_NAME], timeout=30)
    time.sleep(2)
    run(["nmcli", "con", "up", CON_NAME], timeout=60)
    ok = wait_ppp_ok(40)

    # 第二级：射频复位（现场实测：模块"半死"时只有 disable/enable + register-home 能救）
    if not ok and idx:
        log("重拨无效 → 复位射频（disable / enable / register-home）")
        run(["mmcli", "-m", idx, "--disable"], timeout=30)
        time.sleep(3)
        run(["mmcli", "-m", idx, "--enable"], timeout=30)
        time.sleep(6)
        run(["mmcli", "-m", idx, "--3gpp-register-home"], timeout=40)
        time.sleep(10)
        run(["nmcli", "con", "up", CON_NAME], timeout=60)
        ok = wait_ppp_ok(40)

    # 第三级：USB 重新授权（等于拔插一次模块；半小时最多一次）
    if not ok:
        usb_dir = modem_usb_dir()
        if usb_dir and cooldown_ready(float(state.get("last_usb_ts") or 0), time.time(), 1800):
            log(f"射频复位无效 → USB 重新授权（{usb_dir}）")
            try:
                with open(os.path.join(usb_dir, "authorized"), "w", encoding="utf-8") as fh:
                    fh.write("0")
                time.sleep(2)
                with open(os.path.join(usb_dir, "authorized"), "w", encoding="utf-8") as fh:
                    fh.write("1")
            except OSError as exc:
                log(f"  USB 授权失败：{exc}")
            save_state(last_usb_ts=time.time())
            time.sleep(12)
            run(["nmcli", "con", "up", CON_NAME], timeout=60)
            ok = wait_ppp_ok(60)

    save_state(last_fix_ts=time.time(), last_fix_ok=ok, fail_streak=0 if ok else streak)
    log("自愈成功" if ok else "自愈失败（三级恢复都没救回来），等下一轮再试")
    return 0 if ok else 1


def acquire_lock():
    """防止上一轮还没跑完又被 timer 拉起来。"""
    import fcntl

    handle = open(LOCK_PATH, "w")
    try:
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        log("已有实例在跑，本轮退出")
        sys.exit(0)
    return handle


def main(argv=None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    dry = "--dry-run" in args
    mode = next((a for a in args if not a.startswith("-")), "maintain")
    _lock = None if dry else acquire_lock()
    try:
        if mode == "fix-rndis":
            acted = fix_rndis(dry=dry)
            log(f"RNDIS 处理完成（{acted} 次）")
            return 0
        if mode == "watch":
            return watch(dry=dry)
        acted = fix_rndis(dry=dry)
        if acted:
            log(f"RNDIS 处理 {acted} 次")
        return watch(dry=dry)
    finally:
        if _lock is not None:
            _lock.close()


if __name__ == "__main__":
    sys.exit(main())
