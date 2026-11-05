"""[模拟] 后退让位→暂停→恢复→跳点（一次穿过）。独立 ROS 域运行。

模拟 2：一次穿过（through-poses）模式的让位→暂停→恢复→跳点。"""
import os
import sys

if os.environ.get('ROS_DOMAIN_ID') in (None, '', '0'):
    sys.exit('拒绝在默认 ROS 域运行：请设置 ROS_DOMAIN_ID=77 等独立域（避免影响实机）')

import rclpy
from std_msgs.msg import Bool
from fwr_bringup.frontier_explorer import FrontierExplorer

rclpy.init()
node = FrontierExplorer()
node._dispatch = lambda: None
node._cancel_goal = lambda: None
node._end_recovery = lambda success: None

# 4 点一次穿过路线
node._fixed_goals = None
node._multi_points = [(1.0, 0.0, 0.0), (2.0, 0.0, 0.0), (3.0, 0.0, 0.0), (4.0, 0.0, 0.0)]
node._sm.start_fixed(legs=1)
node._sm._fixed_dispatched = 1
node._status = 'active'

ok = True
def show(tag):
    print('  %-16s pts=%d enabled=%-5s paused=%-5s episodes=%d' % (
        tag, len(node._multi_points or []), node._sm.enabled,
        node._paused_for_reverse, node._reverse_episodes))

show('初始')
# 两次让位：不跳过
for i in range(2):
    node._on_reverse_active(Bool(data=True))
    node._on_reverse_active(Bool(data=False))
show('两次让位后')
ok &= (len(node._multi_points) == 4 and node._sm.enabled)

# 第三次：超过上限 → 跳过最近（无位姿 → 丢掉最前面一个）
node._on_reverse_active(Bool(data=True))
node._on_reverse_active(Bool(data=False))
show('三次让位后')
ok &= (len(node._multi_points) == 3)

# 继续让位直到不足 2 点 → 结束任务
for _ in range(4):
    node._on_reverse_active(Bool(data=True))
    node._on_reverse_active(Bool(data=False))
show('多次让位后')
ok &= (node._multi_points is None and node._sm.enabled is False)

print('\n  结果:', '✅ 一次穿过链路符合预期' if ok else '❌ 有断言不成立')
node.destroy_node()
rclpy.shutdown()
raise SystemExit(0 if ok else 1)
