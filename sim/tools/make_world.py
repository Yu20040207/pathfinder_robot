"""生成"走廊 + 四房间"仿真世界。

布局（俯视，+X 向右，+Y 向上）：
    走廊沿 Y 方向，宽 2 m（x ∈ [-1, +1]），长 16 m（y ∈ [-8, +8]）
    左侧两个房间：x ∈ [-8, -1]，以 y=0 分隔（上/下各一间）
    右侧两个房间：x ∈ [+1, +8]，以 y=0 分隔（上/下各一间）
    每个房间在走廊一侧开 1.0 m 宽的门
    墙高 5 m、厚 0.2 m

用法：.venv/bin/python sim/tools/make_world.py
"""
import math
import os

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
OUT = os.path.join(ROOT, 'sim', 'worlds', 'four_rooms.sdf')

WALL_H = 5.0
WALL_T = 0.2
HALF_X = 8.0          # 外墙到中心距离
HALF_Y = 8.0
CORR_HALF = 1.0       # 走廊半宽（2 m 宽）
DOOR_W = 1.0          # 门宽
DOOR_Y = 4.0          # 门中心（上下房间各一个）


def box(name, cx, cy, sx, sy, color='0.55 0.55 0.6'):
    return f'''    <model name="{name}">
      <static>true</static>
      <pose>{cx:.3f} {cy:.3f} {WALL_H / 2:.3f} 0 0 0</pose>
      <link name="l">
        <collision name="c"><geometry><box><size>{sx:.3f} {sy:.3f} {WALL_H:.3f}</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>{sx:.3f} {sy:.3f} {WALL_H:.3f}</size></box></geometry>
          <material><ambient>{color} 1</ambient><diffuse>{color} 1</diffuse></material></visual>
      </link>
    </model>
'''


def wall_seg_y(name, x, y0, y1, color='0.55 0.55 0.6'):
    """沿 Y 方向的墙段（x 固定）。"""
    return box(name, x, (y0 + y1) / 2, WALL_T, abs(y1 - y0), color)


def wall_seg_x(name, y, x0, x1, color='0.6 0.6 0.65'):
    """沿 X 方向的墙段（y 固定）。"""
    return box(name, (x0 + x1) / 2, y, abs(x1 - x0), WALL_T, color)


def furniture(name, cx, cy, sx, sy, sz, color):
    return f'''    <model name="{name}">
      <static>true</static>
      <pose>{cx:.3f} {cy:.3f} {sz / 2:.3f} 0 0 0</pose>
      <link name="l">
        <collision name="c"><geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>
          <material><ambient>{color} 1</ambient><diffuse>{color} 1</diffuse></material></visual>
      </link>
    </model>
'''


def cylinder(name, cx, cy, radius, height, color):
    return f'''    <model name="{name}">
      <static>true</static>
      <pose>{cx:.3f} {cy:.3f} {height / 2:.3f} 0 0 0</pose>
      <link name="l">
        <collision name="c"><geometry><cylinder><radius>{radius:.3f}</radius><length>{height:.3f}</length></cylinder></geometry></collision>
        <visual name="v"><geometry><cylinder><radius>{radius:.3f}</radius><length>{height:.3f}</length></cylinder></geometry>
          <material><ambient>{color} 1</ambient><diffuse>{color} 1</diffuse></material></visual>
      </link>
    </model>
'''


parts = []
parts.append(f'''<?xml version="1.0" ?>
<!-- 自动生成：sim/tools/make_world.py
     中央 2 m 走廊 + 左右各两个房间，墙高 {WALL_H} m，每个房间在走廊侧开门 -->
<sdf version="1.8">
  <world name="four_rooms">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="ignition-gazebo-physics-system"
            name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-user-commands-system"
            name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system"
            name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-sensors-system"
            name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="ignition-gazebo-imu-system"
            name="ignition::gazebo::systems::Imu"/>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 12 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.3 0.2 -1</direction>
    </light>

    <scene>
      <ambient>0.65 0.65 0.65 1</ambient>
      <background>0.85 0.87 0.9 1</background>
    </scene>

    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>
          <material><ambient>0.78 0.78 0.78 1</ambient><diffuse>0.78 0.78 0.78 1</diffuse></material>
        </visual>
      </link>
    </model>
''')

# ---------- 外墙 ----------
parts.append(wall_seg_x('outer_south', -HALF_Y, -HALF_X - WALL_T, HALF_X + WALL_T))
parts.append(wall_seg_x('outer_north', HALF_Y, -HALF_X - WALL_T, HALF_X + WALL_T))
parts.append(wall_seg_y('outer_west', -HALF_X, -HALF_Y - WALL_T, HALF_Y + WALL_T))
parts.append(wall_seg_y('outer_east', HALF_X, -HALF_Y - WALL_T, HALF_Y + WALL_T))

# ---------- 走廊两侧的墙（含门洞）----------
for side, x in (('w', -CORR_HALF), ('e', CORR_HALF)):
    # 门洞：y 在 ±DOOR_Y 各一个，宽 DOOR_W
    segs = [(-HALF_Y, -DOOR_Y - DOOR_W / 2),
            (-DOOR_Y + DOOR_W / 2, DOOR_Y - DOOR_W / 2),
            (DOOR_Y + DOOR_W / 2, HALF_Y)]
    for i, (y0, y1) in enumerate(segs):
        parts.append(wall_seg_y(f'corr_{side}_{i}', x, y0, y1, '0.5 0.52 0.58'))

# ---------- 左右房间之间的分隔墙 ----------
parts.append(wall_seg_x('div_west', 0.0, -HALF_X, -CORR_HALF - WALL_T / 2))
parts.append(wall_seg_x('div_east', 0.0, CORR_HALF + WALL_T / 2, HALF_X))

# ---------- 房间家具 ----------
# 左上房间 (x -8..-1, y 0..8)
parts.append(furniture('lt_table', -4.5, 4.0, 1.6, 1.0, 0.75, '0.55 0.4 0.25'))
parts.append(cylinder('lt_chair1', -4.5, 2.9, 0.24, 0.5, '0.3 0.35 0.45'))
parts.append(cylinder('lt_chair2', -4.5, 5.1, 0.24, 0.5, '0.3 0.35 0.45'))
parts.append(furniture('lt_cabinet', -7.2, 1.5, 0.6, 2.2, 1.2, '0.4 0.4 0.3'))

# 左下房间 (x -8..-1, y -8..0)
parts.append(furniture('lb_sofa', -5.5, -7.0, 2.4, 0.9, 0.7, '0.35 0.45 0.5'))
parts.append(furniture('lb_tea', -5.5, -5.6, 1.1, 0.6, 0.44, '0.6 0.5 0.4'))
parts.append(furniture('lb_shelf', -7.2, -3.0, 0.4, 3.0, 1.8, '0.45 0.35 0.3'))

# 右上房间 (x 1..8, y 0..8)
parts.append(furniture('rt_desk', 4.5, 4.5, 2.6, 0.8, 0.75, '0.5 0.45 0.4'))
parts.append(cylinder('rt_bin1', 7.0, 2.5, 0.26, 0.6, '0.35 0.5 0.35'))
parts.append(cylinder('rt_bin2', 7.0, 3.3, 0.26, 0.6, '0.35 0.5 0.35'))
parts.append(furniture('rt_cabinet', 1.8, 6.5, 0.6, 2.0, 1.2, '0.4 0.4 0.3'))

# 右下房间 (x 1..8, y -8..0)
parts.append(furniture('rb_table', 4.0, -4.0, 1.4, 0.9, 0.75, '0.55 0.4 0.25'))
parts.append(cylinder('rb_pillar', 2.5, -2.0, 0.3, 1.0, '0.5 0.5 0.5'))
parts.append(furniture('rb_boxes', 6.5, -6.0, 1.0, 1.0, 0.6, '0.65 0.55 0.35'))
parts.append(furniture('rb_shelf', 1.8, -6.8, 0.4, 2.6, 1.8, '0.45 0.35 0.3'))

parts.append('  </world>\n</sdf>\n')

open(OUT, 'w').write(''.join(parts))
print(f'已生成 {OUT}')
print(f'  范围 x±{HALF_X} y±{HALF_Y}，走廊宽 {2 * CORR_HALF} m，墙高 {WALL_H} m')
print(f'  房间门：走廊两侧 y=±{DOOR_Y} 各一个 {DOOR_W} m 宽')
