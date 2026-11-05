"""[模拟] 后退让位→暂停→恢复→跳点（顺序多段）。独立 ROS 域运行。

模拟：后退让位 → 暂停 → 恢复 → 超限跳过（用真实 explorer 代码，不接实机）。

不发任何运动指令；只驱动 /reverse/active 回调与状态机判定。
"""
import os
import sys

if os.environ.get('ROS_DOMAIN_ID') in (None, '', '0'):
    sys.exit('拒绝在默认 ROS 域运行：请设置 ROS_DOMAIN_ID=77 等独立域（避免影响实机）')

import rclpy
from std_msgs.msg import Bool

from fwr_bringup.frontier_explorer import FrontierExplorer

rclpy.init()
node = FrontierExplorer()

# 隔离：不真的发目标/取消
node._dispatch = lambda: None
node._cancel_goal = lambda: None
node._end_recovery = lambda success: None

# 造一个 3 段任务，已经派发到第 2 段
node._fixed_goals = [(1.0, 0.0, 0.0), (2.0, 0.0, 0.0), (3.0, 0.0, 0.0)]
node._multi_points = None
node._sm.start_fixed(legs=3)
node._sm.reset_dispatched()
node._sm._fixed_dispatched = 2          # 第 2 段已派发
node._status = 'active'
node._mission_suspended = False

def show(tag):
    print('  %-22s enabled=%-5s paused=%-5s leg=%d/%d status=%s episodes=%d' % (
        tag, node._sm.enabled, node._paused_for_reverse,
        node._sm.fixed_leg_index + 1, node._sm._fixed_legs,
        node._status, node._reverse_episodes))

show('初始')
ok = True

# ① 让位开始 → 应暂停而不中止
node._on_reverse_active(Bool(data=True))
step1 = (node._paused_for_reverse is True and node._sm.enabled is True)
show('让位开始')
ok &= step1

# ② 暂停期间 tick 不推进
node._tick()
step2 = (node._paused_for_reverse is True)
ok &= step2

# ③ 让位结束（第 1 次）→ 恢复，段不变
node._on_reverse_active(Bool(data=False))
step3 = (node._paused_for_reverse is False and node._sm.enabled is True
         and node._sm.fixed_leg_index == 1 and node._reverse_episodes == 1)
show('结束(第1次)')
ok &= step3

# ④ 第 2 次让位（同一点）→ 仍不跳过
node._on_reverse_active(Bool(data=True))
node._on_reverse_active(Bool(data=False))
step4 = (node._sm.fixed_leg_index == 1 and node._reverse_episodes == 2)
show('结束(第2次)')
ok &= step4

# ⑤ 第 3 次让位 → 超过上限 2 → 跳过该点
node._on_reverse_active(Bool(data=True))
node._on_reverse_active(Bool(data=False))
step5 = (node._sm.fixed_leg_index == 2 and node._reverse_episodes == 3)
show('结束(第3次)')
ok &= step5

# ⑥ 第 4、5 次让位 → 应继续跳，直到最后一段后结束
for i in range(2):
    node._on_reverse_active(Bool(data=True))
    node._on_reverse_active(Bool(data=False))
show('继续让位两次')
step6 = node._sm.enabled is False        # 最后一段跳过后结束任务
ok &= step6

print('\n  结果:', '✅ 链路全部符合预期' if ok else '❌ 有断言不成立')
node.destroy_node()
rclpy.shutdown()
raise SystemExit(0 if ok else 1)
