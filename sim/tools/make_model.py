"""把真实 URDF 变成 Gazebo 仿真用的 URDF：
   1. package:// → 绝对 file:// 路径（Gazebo 不认 ROS 包路径）
   2. 注入 diff_drive 插件、lidar/imu/超声传感器、配色
"""
import math
import os
import re

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
DESC = os.path.join(ROOT, 'ros2_ws/src/fwr_bringup/description')
src = open(os.path.join(DESC, 'fwr_chassis_demo.urdf')).read()

# 1) mesh 路径
src = src.replace('package://fwr_bringup/description/', f'file://{DESC}/')

# 2) 传感器与插件（位姿相对各自 link）
GAZEBO = f'''
  <!-- ================= 仿真专用：物理插件 + 传感器 ================= -->
  <gazebo>
    <plugin filename="ignition-gazebo-diff-drive-system"
            name="ignition::gazebo::systems::DiffDrive">
      <left_joint>left_drive_wheel_preview_joint</left_joint>
      <right_joint>right_drive_wheel_preview_joint</right_joint>
      <!-- 与 URDF 一致：轮半径 0.0325，轮距 0.315（y=±0.1575） -->
      <wheel_separation>0.315</wheel_separation>
      <wheel_radius>0.0325</wheel_radius>
      <max_linear_acceleration>0.3</max_linear_acceleration>
      <max_angular_acceleration>1.5</max_angular_acceleration>
      <odom_publish_frequency>30</odom_publish_frequency>
      <topic>cmd_vel</topic>
      <odom_topic>odom</odom_topic>
      <frame_id>odom</frame_id>
      <child_frame_id>base_footprint</child_frame_id>
    </plugin>
    <plugin filename="ignition-gazebo-joint-state-publisher-system"
            name="ignition::gazebo::systems::JointStatePublisher">
      <topic>joint_states</topic>
    </plugin>
    <!-- 万向轮是 continuous 关节但没有驱动，设成小摩擦的自由轮 -->
  </gazebo>

  <!-- 雷达：装在 laser_front 上，**倒装 180°**（scan 角 0 = 车尾，与实车一致） -->
  <gazebo reference="laser_front">
    <sensor type="gpu_lidar" name="scan">
      <!-- laser_front 关节已含 180°（物理倒装补偿），这里不再重复 -->
      <pose>0 0 0.02 0 0 0</pose>
      <topic>scan</topic>
      <gz_frame_id>laser_front</gz_frame_id>
      <always_on>1</always_on>
      <update_rate>10</update_rate>
      <visualize>true</visualize>
      <lidar>
        <scan>
          <horizontal>
            <samples>720</samples>
            <resolution>1</resolution>
            <!-- 实车 C1 是 360°，但车身挡住后方 ±45°，再遮 ±50° → 有效 260°。
                 仿真里直接把雷达做成 ±130°（原生 260°），避免依赖"事后遮罩"，
                 因为实测：给 Cartographer 喂带 inf/NaN 遮罩的扫描会让它位姿乱跳。 -->
            <min_angle>{-2.268928}</min_angle>
            <max_angle>{2.268928}</max_angle>
          </horizontal>
        </scan>
        <range><min>0.15</min><max>12.0</max><resolution>0.01</resolution></range>
      </lidar>
    </sensor>
  </gazebo>

  <!-- IMU -->
  <gazebo reference="imu_link">
    <sensor type="imu" name="imu">
      <topic>imu_raw</topic>
      <gz_frame_id>imu_link</gz_frame_id>
      <always_on>1</always_on>
      <update_rate>100</update_rate>
      <imu/>
    </sensor>
  </gazebo>
'''

# 3) 4 路后向超声（放在车体后缘外侧，3 束窄 lidar 模拟 60° 锥角）
US = (('left_out', 0.105, math.radians(-10.0)),
      ('left_in', 0.05, 0.0),
      ('right_in', -0.05, 0.0),
      ('right_out', -0.105, math.radians(10.0)))
for name, y, yaw in US:
    GAZEBO += f'''
  <gazebo reference="base_link">
    <sensor type="gpu_lidar" name="us_{name}">
      <!-- base_link 比 base_footprint 高 0.0325；传感器离地 0.14 -->
      <pose>-0.33 {y} 0.1075 0 0 {math.pi + yaw}</pose>
      <topic>us_{name}</topic>
      <gz_frame_id>ultrasonic_rear_{name}</gz_frame_id>
      <always_on>1</always_on>
      <update_rate>10</update_rate>
      <visualize>true</visualize>
      <lidar>
        <scan><horizontal><samples>3</samples><resolution>1</resolution>
          <min_angle>-0.5236</min_angle><max_angle>0.5236</max_angle></horizontal></scan>
        <range><min>0.05</min><max>3.5</max><resolution>0.01</resolution></range>
      </lidar>
    </sensor>
  </gazebo>
'''

# 4) 配色（Gazebo 里没有这些就是默认灰）
COLORS = '''
  <gazebo reference="base_link"><material>Gazebo/Blue</material></gazebo>
  <gazebo reference="left_drive_wheel"><material>Gazebo/Black</material></gazebo>
  <gazebo reference="right_drive_wheel"><material>Gazebo/Black</material></gazebo>
  <gazebo reference="front_left_caster_wheel"><material>Gazebo/DarkGrey</material></gazebo>
  <gazebo reference="front_right_caster_wheel"><material>Gazebo/DarkGrey</material></gazebo>
  <gazebo reference="laser_front"><material>Gazebo/DarkGrey</material></gazebo>
  <gazebo reference="motor_driver_body"><material>Gazebo/Green</material></gazebo>
'''

out = src.replace('</robot>', GAZEBO + COLORS + '</robot>')
os.makedirs(os.path.join(ROOT, 'sim/models/fwr_bot'), exist_ok=True)
path = os.path.join(ROOT, 'sim/models/fwr_bot/robot.urdf')
open(path, 'w').write(out)
print('已生成', path, len(out.splitlines()), '行')
