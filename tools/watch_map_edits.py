#!/usr/bin/env python3
"""后台守护：Map Edit 面板一保存新地图，就自动生成修正版 *_fixed。

基准图来自 /tmp/fwr_runs/current_baseline.txt（启动导航时写入的那张地图）。
新生成的地图写入同一目录的 <名字>_fixed.pgm/.yaml，并记日志。
"""
from __future__ import annotations

import os
import time

from fix_map_unknown import apply_edits, load

MAPS = os.path.abspath(os.path.join(os.path.dirname(__file__), '..',
                                    'ros2_ws/src/fwr_bringup/maps'))
BASELINE_FILE = '/tmp/fwr_runs/current_baseline.txt'
LOG = '/tmp/fwr_runs/map_fix.log'
POLL_S = 2.0


def log(msg: str) -> None:
    line = time.strftime('%H:%M:%S ') + msg
    print(line, flush=True)
    with open(LOG, 'a', encoding='utf-8') as handle:
        handle.write(line + '\n')


def baseline() -> str | None:
    try:
        path = open(BASELINE_FILE, encoding='utf-8').read().strip()
    except OSError:
        return None
    return path if path and os.path.exists(path) else None


def main() -> None:
    log('地图监视已启动（目录 %s）' % MAPS)
    seen = set()
    for name in os.listdir(MAPS):
        if name.endswith('.yaml'):
            seen.add(name)
    while True:
        try:
            base_path = baseline()
            for name in sorted(os.listdir(MAPS)):
                if not name.endswith('.yaml') or name in seen:
                    continue
                if name.endswith('_fixed.yaml') or name.endswith('_padded.yaml'):
                    seen.add(name)
                    continue
                path = os.path.join(MAPS, name)
                # 等文件写完
                size0 = os.path.getsize(path)
                time.sleep(0.5)
                if os.path.getsize(path) != size0:
                    continue
                seen.add(name)
                if base_path is None:
                    log('发现新地图 %s，但没有基准图（current_baseline.txt），跳过' % name)
                    continue
                stem = name[:-5]
                out = os.path.join(MAPS, stem + '_fixed.yaml')
                try:
                    base, meta, _ = load(base_path)
                    edited, _, _ = load(path)
                    fixed, n_draw, n_erase = apply_edits(base, edited)
                    from PIL import Image
                    import yaml
                    Image.fromarray(fixed).save(out[:-5] + '.pgm')
                    m = dict(meta)
                    m['image'] = stem + '_fixed.pgm'
                    with open(out, 'w', encoding='utf-8') as handle:
                        yaml.safe_dump(m, handle, default_flow_style=None, sort_keys=False)
                    log('已修正 %s → %s_fixed（画墙 %d，擦墙 %d，基准 %s）' % (
                        name, stem, n_draw, n_erase, os.path.basename(base_path)))
                except Exception as exc:                       # noqa: BLE001
                    log('修正 %s 失败: %s' % (name, exc))
            time.sleep(POLL_S)
        except Exception as exc:                               # noqa: BLE001
            log('监视异常: %s' % exc)
            time.sleep(POLL_S)


if __name__ == '__main__':
    main()
