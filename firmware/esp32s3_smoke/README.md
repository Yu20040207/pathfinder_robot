# ESP32-S3 串口冒烟测试

本固件只读取芯片信息，并每秒向 USB Serial/JTAG 控制台输出一次心跳。
它不配置任何电机控制 GPIO，不能用于驱动底盘。使用 ESP-IDF v5.5.5。

```bash
source /home/manbaout/esp/esp-idf/export.sh
idf.py set-target esp32s3 build
idf.py -p /dev/ttyACM0 flash monitor
```

烧录会覆盖板上原有程序。首次烧录前读取芯片型号、Flash 容量，并备份完整 Flash。
`/dev/ttyACM0` 需对当前用户开放读写权限；设备重新插拔后端口号和临时 ACL
均可能改变。若有电机驱动板连接，先断开电机动力电源再进行此测试。
