"""守护：节点里读取的 self.xxx 必须在本类中被赋过值。

2026-11-04 两次事故都是这个原因：
  1. 新增回调方法没插进文件 → AttributeError → 探索器启动即崩
  2. 时间戳字段没在 __init__ 赋值 → 第一帧 tick 崩溃 → 车完全不动

纯静态扫描，只统计**本类自己的方法体**（排除嵌套类、类常量、rclpy 继承方法）。
"""
from pathlib import Path
import ast

PKG = Path(__file__).resolve().parents[1] / 'fwr_bringup'

# rclpy Node 及常见基类提供的方法（不作为"未定义属性"）
INHERITED = {
    'create_client', 'create_publisher', 'create_service', 'create_subscription',
    'create_timer', 'create_rate', 'destroy_node', 'destroy_timer', 'declare_parameter',
    'declare_parameters', 'get_clock', 'get_logger', 'get_name', 'get_parameter',
    'get_parameters', 'set_parameters', 'add_on_set_parameters_callback',
    'get_node_names', 'get_publishers_info_by_topic', 'get_subscriptions_info_by_topic',
    'count_publishers', 'count_subscribers', 'spin', 'now', 'thisown',
}


def _self_attrs(node: ast.AST, ctx) -> set:
    return {
        s.attr for s in ast.walk(node)
        if isinstance(s, ast.Attribute) and isinstance(s.ctx, ctx)
        and isinstance(s.value, ast.Name) and s.value.id == 'self'
    }


def _class_problems(cls: ast.ClassDef):
    methods = [n for n in cls.body
               if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))]
    class_names = set()
    for stmt in cls.body:                      # 类常量 / 类级赋值
        if isinstance(stmt, ast.Assign):
            for t in stmt.targets:
                if isinstance(t, ast.Name):
                    class_names.add(t.id)
        elif isinstance(stmt, ast.AnnAssign) and isinstance(stmt.target, ast.Name):
            class_names.add(stmt.target.id)
    assigned = set().union(*[_self_attrs(m, ast.Store) for m in methods]) if methods else set()
    methods_names = {m.name for m in methods}
    read = set().union(*[_self_attrs(m, ast.Load) for m in methods]) if methods else set()
    return sorted(read - assigned - methods_names - class_names - INHERITED)


def test_no_attribute_read_before_assignment():
    problems = []
    for path in sorted(PKG.glob('*.py')):
        try:
            tree = ast.parse(path.read_text())
        except SyntaxError:
            continue
        for cls in [n for n in ast.walk(tree) if isinstance(n, ast.ClassDef)]:
            missing = _class_problems(cls)
            if missing:
                problems.append(f'{path.name}:{cls.name} -> {missing}')
    assert not problems, (
        '这些类读取了从未赋值过的属性（运行时会 AttributeError，节点会崩）：\n  '
        + '\n  '.join(problems))
