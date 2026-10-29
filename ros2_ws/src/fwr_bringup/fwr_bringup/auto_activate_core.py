"""Readiness gate for automatically activating the Nav2 lifecycle group."""

SCAN_FRESH_S = 2.0


def ready_to_activate(map_seen, scan_age, tf_ok, scan_limit=SCAN_FRESH_S):
    """True when the latched map arrived, scan is fresh and map->base TF exists.

    /map 由 map_server 以 transient_local 锁存发布，只发一次；不能用“新鲜度”
    判断，否则等一个永远不会再来的新消息。
    """
    if not map_seen:
        return False
    if scan_age is None or not (0.0 <= scan_age <= scan_limit):
        return False
    return bool(tf_ok)
