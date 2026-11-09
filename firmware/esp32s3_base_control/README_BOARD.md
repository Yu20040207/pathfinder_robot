# 底盘固件：两种硬件配置（一键切换）

固件源码只有一份，靠 `main/board_config.h` 里的 `DRIVER_WSDC2412D` 开关选择硬件。
**旧固件没有被删除**，永远可以编回来。

## 两个预编译固件（`bin/`）

| 文件 | 适用硬件 | 轮径 / 每圈计数 |
| --- | --- | --- |
| `bin/base_control_at8236_66mm_1468.bin` | **旧**：AT8236 双 PWM + 66mm 轮 | 210 mm / 1468 |
| `bin/base_control_wsdc2412d_100mm_6400.bin` | **新**：WSDC2412D-V3.0 + 100mm 轮 + 100:1 电机 | 314.159 mm / 6400 |

烧录（**先断电机的 12V/24V，车轮悬空**）：

```bash
source ~/esp/esp-idf/export.sh
cd firmware/esp32s3_base_control
idf.py -p /dev/ttyACM0 flash
# 或直接写 bin：
python -m esptool --chip esp32s3 -b 460800 --before default_reset --after hard_reset \
  write_flash 0x0 bin/base_control_wsdc2412d_100mm_6400.bin
```

## 从源码编译

```bash
source ~/esp/esp-idf/export.sh
cd firmware/esp32s3_base_control

# 新硬件（默认）
idf.py build

# 旧硬件（AT8236 + 66mm 轮）
idf.py -DDRIVER_WSDC2412D=0 -DWHEEL_CIRC_MM=210.0f -DCOUNTS_PER_REV=1468.0f build
```

## 引脚差异

| GPIO | 旧（AT8236 双 PWM）| 新（WSDC2412D）|
| --- | --- | --- |
| 4 | 左 正转 PWM | **PWM1（P1）** |
| 5 | 左 反转 PWM | **INA1（A1）** |
| 6 | 右 反转 PWM | **PWM2（P2）** |
| 7 | 右 正转 PWM | **INA2（A2）** |
| 12 | — | **INB1（B1）** |
| 13 | — | **INB2（B2）** |
| 8/9 | 左编码器 A/B | 同（需 3.3V 电平转换）|
| 10/11 | 右编码器 A/B | 同（需 3.3V 电平转换）|

## 新增的安全行为（换驱动板时补的）

1. **上电顺序**：先把 4 个方向脚钉成 LOW/LOW（制动），再配置 LEDC。
   > 之前出现过"PWM 脚悬空(=高) + 方向脚被拉低 → 全速正转失控"，就是从这来的。
2. **换向归零**：正反转切换时强制多走一个 20 ms 周期（PWM=0 后再换向），
   满足手册"大能量正反转切换前先把 PWM 降到 0%"的要求。
3. **PWM 频率**：新板 18 kHz（手册要求 18–60 kHz）；旧板仍 10 kHz。
4. **停车**：INA=INB=L（制动），与旧板"两路 PWM 全低"行为一致。

## 部署前必须标定

| 项 | 怎么测 | 当前假设 |
| --- | --- | --- |
| 每圈计数 | 手转一圈车轮，看 `S2` 帧计数变化 | **6400**（16PPR×4×100）|
| 轮周长 | 卷尺：直行 2 m 对比 `/odom` | **314.159 mm** |
| 轮距 | 左右轮接地点中心距 | 待实测（旧车 262 mm）|
| 里程方向 | 前进时左轮计数应为负、右轮为正 | 沿用旧约定 |

> ⚠️ 编码器 **5V 供电**，A/B 必须经电平转换（BSS138 四路或分压）后再进 GPIO8~11。
