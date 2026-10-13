# Jetson 主控迁移约束

日期：2026-10-12。用户确定后续主控目标为 **NVIDIA Jetson Orin Nano Super Developer Kit（8GB）**。当前先在 Ubuntu 开发电脑上调试，待取得 Jetson 后迁移并完成实机验收。

## 开发阶段划分

1. 开发电脑运行 ROS 2 节点、RViz、建图/定位、路径规划、视觉和语音；ESP32-S3 负责底盘编码器采样、速度闭环、故障停车及与主机的有线通信。样机先用一只底盘前置雷达。
2. Jetson 到位后，将主机上的 ROS 2 工作负载迁移到 Jetson；保留与 ESP32-S3 的速度命令、编码器反馈及故障状态协议，保留雷达、IMU、里程计和任务层 ROS 消息接口。设备端口、模型路径、推理后端、启动参数和资源限额通过配置适配。
3. 迁移验收包括 USB/串口和雷达驱动、HP60C 厂商 SDK 的 ARM64 支持与实机稳定性、IMU 驱动、TF 和时间戳、Nav2 实机控制，以及视觉/语音/导航并发时的内存、温度、功耗和响应时间。开发电脑上通过不等于 Jetson 上通过。

## 后续开发要求

- 新代码避免写死开发电脑的绝对路径、串口设备名或 x86_64 专用二进制；设备名称、参数、模型及数据目录可配置。第三方依赖先检查目标 JetPack 系统和 aarch64 可用性，必要时把平台专用实现隔离在适配层。
- 以标准 ROS 2 消息和 TF 连接传感器、底盘与导航；上层业务不直接依赖某台电脑或某款控制板的设备路径。ROS 2 发行版与 JetPack 的组合在拿到设备后核定，不预设当前电脑的软件包可以原样复制。
- 在开发电脑上先完成可重复的离线测试和传感器单项验收；迁移后重做硬件接入、标定、性能和安全验收。ESP32-S3 的通信超时停车与独立硬件急停不依赖 Jetson 进程正常运行。

## 参考

- [NVIDIA Jetson Orin Nano Super Developer Kit](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/nano-super-developer-kit/)
