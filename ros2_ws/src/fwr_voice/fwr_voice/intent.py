"""Deterministic offline command recognition."""

import re

from .contracts import TaskDecision, TaskRequest


def normalize(text: str) -> str:
    decimals = re.sub(r"(?<=\d)\.(?=\d)", "__decimal__", text)
    return re.sub(r"[\s，。！？、,.!?]", "", decimals).replace("__decimal__", ".").lower()


def parse_local(text: str, locations: dict[str, tuple[str, ...]]) -> TaskDecision:
    phrase = normalize(text)
    if not phrase:
        return TaskDecision("clarify", None, "请再说一遍。")
    if phrase.startswith(("停止", "取消任务", "停下来")):
        return TaskDecision("task", TaskRequest("", "STOP"), "正在停止当前任务。")
    if phrase in ("机器人状态", "现在什么状态", "查询状态"):
        return TaskDecision("task", TaskRequest("", "QUERY_STATUS"), "正在查询状态。")
    jog_directions = {
        "前进": "forward", "前进一下": "forward:0.12",
        "后退": "backward", "后退一下": "backward:0.12",
        "左转": "left", "左转一下": "left:0.35",
        "右转": "right", "右转一下": "right:0.35",
    }
    if phrase in jog_directions:
        return TaskDecision("task", TaskRequest("", "SIM_JOG", jog_directions[phrase]),
                            "收到")
    motion = re.fullmatch(r"(前进|后退)([零一二两三四五六七八九十\d]+(?:\.\d+)?)米", phrase)
    if motion:
        direction = {"前进": "forward", "后退": "backward"}[motion.group(1)]
        numerals = {"零": "0", "一": "1", "二": "2", "两": "2", "三": "3", "四": "4",
                    "五": "5", "六": "6", "七": "7", "八": "8", "九": "9", "十": "10"}
        raw = motion.group(2)
        try:
            meters = float("".join(numerals.get(char, char) for char in raw))
        except ValueError:
            meters = 0
        if 0 < meters <= 10:
            return TaskDecision("task", TaskRequest("", "SIM_JOG", f"{direction}:{meters:g}"),
                                "收到")
        return TaskDecision("clarify", None, "请说十米以内的距离。")
    match = re.fullmatch(r"(?:去|前往|到)(.+)", phrase)
    if match:
        target = match.group(1)
        ids = [location_id for location_id, aliases in locations.items()
               if target in {normalize(alias) for alias in aliases}]
        if len(ids) == 1:
            return TaskDecision("task", TaskRequest("", "GO_TO_NAMED_LOCATION", ids[0]),
                                "正在准备前往。")
        return TaskDecision("clarify", None, "地点不明确，请说配置中的地点名称。")
    return TaskDecision("reply", None, "")
