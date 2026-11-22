"""充电判定（纯逻辑，可单测；被 `battery_modbus` 调用）。

## 为什么单独抽出来

2026-11-22 用户报「充电中一直跳动」：手机端在"充电中/未充电"之间每秒跳一次。
实测 20 s 采样（1 Hz）：

```
21.38 V  1.09 A  21.3%  status=2    ← 未充电
21.37 V  1.09 A  21.2%  status=1    ← 充电中
21.37 V  1.06 A  21.2%  status=2
21.35 V  1.46 A  20.8%  status=1
...
```
**每秒都在翻**。根因是旧规则三段互相打架：

1. `i >= +0.20 A 且 V >= 24.60 V` → 充电中；
2. 已经在充电、但电压 `< 24.55 V` → **立刻**改回未充电；
3. 否则按 600 s 窗口最早值比："高 0.10 V 算充电"。

低 SOC（21.4 V，离 24.6 V 还差 3 V）时：规则 3 每秒把状态拉成"充电中"，
规则 2 下一秒又打回"未充电" → 1 Hz 方波。

## 现在怎么判

- **电压长窗斜率**（600 s 最小二乘）：充电 ⇒ 端电压持续上升；
  拔掉 ⇒ 带载缓慢下降；满电 CV ⇒ 几乎不动。
  - 斜率 ≥ `rise_vps`（默认 +8e-5 V/s ≈ **+0.29 V/h**）→ 充电中
  - 斜率 ≤ `fall_vps`（默认 -3e-5 V/s ≈ -0.11 V/h）→ 未充电
  - 中间带（含噪声）→ **保持原状态**，不猜
- **CV 顶点兜底**：斜率≈0 但电压已到 `top_v`（默认 24.60 V）且电流 ≥ 0.2 A → 充电中
  （恒压阶段电压不动，这是唯一能靠电压以外的信息判定的场景）
- **去抖**：候选状态必须**连续稳定** `debounce_s`（默认 30 s）才真的切换。
  即使规则再抖，最多 30 s 才动一次，页面不会再闪。
- **先平滑再求斜率**：电压逐秒抖动 σ≈0.02 V，直接求斜率在短窗里噪声会盖过信号。
  先做 EMA（α 0.2，等效 ~5 s 低通）再最小二乘，并把"可信斜率"的最短跨度抬到
  **240 s**（`min_span_s`）。代价是**插上充电器后约 4~5 分钟**才显示"充电中"，
  但显示出来就不会再跳；拔枪后同样 4~5 分钟判回"未充电"。

实测依据：21.4 V 的 6S 锂电在充电时电压约 **+0.8 V/h**（08:56 20.81 V → 09:40 21.40 V），
远高于 +0.29 V/h 的门限；拔掉充电器带 1.25 A 负载时约 -0.2 V/h，也在 -0.11 V/h 之下。
"""

from __future__ import annotations

WINDOW_S = 600.0


def slope_v_per_s(history, window_s: float = WINDOW_S,
                  min_span_s: float = 30.0):
    """对 ``[(t, v), ...]`` 的最后 ``window_s`` 秒做最小二乘，返回 V/s 斜率。

    数据不足（<3 点或跨度 < ``min_span_s``）返回 ``None``
    （调用方按"不知道"处理，保持原状态，不猜）。
    """
    if not history:
        return None
    t_end = history[-1][0]
    pts = [(t, v) for t, v in history if t >= t_end - window_s]
    if len(pts) < 3:
        return None
    span = pts[-1][0] - pts[0][0]
    if span < float(min_span_s):
        return None
    n = float(len(pts))
    t_mean = sum(t for t, _ in pts) / n
    v_mean = sum(v for _, v in pts) / n
    num = sum((t - t_mean) * (v - v_mean) for t, v in pts)
    den = sum((t - t_mean) ** 2 for t, _ in pts)
    if den <= 0.0:
        return None
    return num / den


class ChargeDetector:
    """带斜率判据 + 去抖的充电状态机。"""

    def __init__(self, rise_vps: float = 8e-5, fall_vps: float = -3e-5,
                 top_v: float = 24.60, top_current_a: float = 0.20,
                 debounce_s: float = 30.0, initial: bool = False,
                 ema_alpha: float = 0.2, min_span_s: float = 240.0):
        self.rise_vps = float(rise_vps)
        self.fall_vps = float(fall_vps)
        self.top_v = float(top_v)
        self.top_current_a = float(top_current_a)
        self.debounce_s = max(0.0, float(debounce_s))
        self.ema_alpha = min(1.0, max(0.05, float(ema_alpha)))
        self.min_span_s = float(min_span_s)
        self._hist = []
        self._v_f = None
        self._state = bool(initial)
        self._cand = None
        self._cand_since = 0.0
        self.slope = None

    @property
    def state(self) -> bool:
        return self._state

    def feed(self, now_s: float, voltage: float, current: float) -> bool:
        """喂一个采样，返回**去抖后**的充电状态。"""
        v = float(voltage)
        self._v_f = v if self._v_f is None else (self.ema_alpha * v +
                                                 (1.0 - self.ema_alpha) * self._v_f)
        self._hist.append((float(now_s), self._v_f))
        while len(self._hist) > 2 and now_s - self._hist[0][0] > WINDOW_S:
            self._hist.pop(0)
        self.slope = slope_v_per_s(self._hist, WINDOW_S, self.min_span_s)

        cand = None
        if self.slope is not None:
            if self.slope >= self.rise_vps:
                cand = True
            elif self.slope <= self.fall_vps:
                cand = False
            elif v >= self.top_v and float(current) >= self.top_current_a:
                cand = True

        if cand is None or cand == self._state:
            self._cand = None            # 没有明确候选 / 候选就是现状 → 不动
            return self._state
        if cand != self._cand:
            self._cand = cand
            self._cand_since = float(now_s)
            return self._state
        if now_s - self._cand_since >= self.debounce_s:
            self._state = cand
            self._cand = None
        return self._state
