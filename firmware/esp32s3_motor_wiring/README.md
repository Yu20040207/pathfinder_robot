# D157B 接线检查固件

用于 ESP32-S3-N16R8 与 D157B 驱动板接线的第一步验收。GPIO4→AIN1、GPIO5→AIN2、GPIO6→BIN1、GPIO7→BIN2，另接 GND→GND。固件只将这四根线持续置低并通过 USB Serial/JTAG 输出心跳；不会使电机转动，也不读取编码器。

烧录时断开 D157B 的 12 V 电机电源并关闭板上开关。ESP32-S3 仅由 USB 供电，不连接 D157B 的 5 V、3.3 V 或 VM 端子。

```bash
source /home/manbaout/esp/esp-idf/export.sh
idf.py set-target esp32s3 build
idf.py -p /dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_90:70:69:F7:F0:B0-if00 flash monitor
```

确认串口反复报告 `AIN1=0 AIN2=0 BIN1=0 BIN2=0` 后退出监视器。再用万用表在驱动板输入排针对 GND 测量四路低电平；显示心跳只证明固件运行，不证明线接对。后续通电和轮子运动测试需另用带超时停车、限速和急停的固件，并先确认编码器输出电平不超过 ESP32-S3 输入范围。
