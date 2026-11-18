"""静态守卫：用 pyflakes 扫未定义名字（"节点启动即崩"的常见原因）。

2026-11-18 连踩两次：
  · trail_reverser 写了 QoSProfile(...) 但只导入了 qos_profile_sensor_data
  · trail_reverser 用了 sector_min_range 但 import 锚点没匹配上
两者 py_compile 都查不出来（语法合法），表现是"进程还在、节点不在 ROS 图里"，
现场极难排查。pyflakes 能直接抓到 undefined name。
"""
import subprocess
import sys
from pathlib import Path

PKG = Path(__file__).resolve().parents[1] / 'fwr_bringup'


def test_no_undefined_names_in_package():
    files = sorted(str(p) for p in PKG.glob('*.py'))
    assert files, '没找到源文件'
    proc = subprocess.run([sys.executable, '-m', 'pyflakes', *files],
                          capture_output=True, text=True)
    if proc.returncode != 0 and 'No module named pyflakes' in proc.stderr:
        return                     # 环境没装 pyflakes 就跳过，不让测试假红
    bad = [ln for ln in (proc.stdout + proc.stderr).splitlines()
           if 'undefined name' in ln]
    assert not bad, '存在未定义名字（运行时必崩）:\n' + '\n'.join(bad)
