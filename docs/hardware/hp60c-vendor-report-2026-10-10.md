# HP60C 问题反馈（可发给安思彊技术支持）

您好，我们在 Ubuntu / ROS 2 Humble、x86_64 电脑上使用 HP60C，遇到间歇停流和 USB 断连问题，希望协助确认 SDK、固件及连接要求。

设备序列号：`ASC60CG21001123`；USB VID:PID：`3482:6723`；厂商 SDK：`v1.2.28.20241021`；固件日志：`V01.09_2024110714:V01.19_2024110714:L01`。相机直连电脑 USB Bus 1 / Port 2，以 USB 2.0 High-Speed 480M 枚举。使用厂商 `ascamera` ROS 2 驱动，RGB 和深度均设置为 640×480、25 fps；曾用 15 fps 做对照。已更换一根数据线，仍能复现停流。

## 复现现象

1. 多次启动后，厂商日志先报 `get mjpeg from xu command failed`，随后 `start streaming`，约 1 秒后 `stop streaming`。有时几秒至几十秒后自行恢复，有时长时间无帧。驱动日志仍显示 MJPEG 尺寸为 640×480。
2. 更换数据线后的 25 fps 测试中，原厂驱动恢复出帧，单独订阅 RGB 的 20 秒收到 273 帧（约 13.6 Hz）；之后 SDK 日志又出现 `stop streaming`，深度话题连续等待 60 秒无帧，期间 USB 设备仍可由 `lsusb` 查到。15 fps 对照测试三路约 65 秒连续出帧，随后也出现 `stop streaming`。探针订阅端的帧率不一定等于相机真实输出帧率，但可确认停流。
3. 2026-10-09 15:29 的另一次启动中，内核反复报 `device descriptor read/64, error -110`、`device not accepting address, error -62`，设备从 USB 总线消失；驱动报 `uvc_open: No such device`。换线重插后设备重新枚举。
4. 2026-10-10 09:01 的实测中，RGB、深度和点云在 100 秒窗口分别收到 1285、1284、1284 帧；同时驱动日志在 09:01:35 停流、09:01:40 恢复、09:03:18 再停流、09:03:31 恢复。09:03:39 内核记录 `usb 1-2: USB disconnect, device number 16`，09:03:47 自动重新枚举为 device 17。整个过程无人为拔插。
5. 将工作区移入新项目目录并重新构建后，2026-10-10 09:54 再次实测：90 秒内收到 RGB 1140、深度 1139、点云 1139 帧，确认路径和 ROS 2 数据链路正常；厂商日志仍在 09:54:30 停流、09:54:48 恢复、09:56:15 再停流。最后一次停流至测试结束未恢复，`lsusb` 仍能看到相机，本轮内核没有 USB 断连日志。说明**设备在线停流并非由工作区搬迁路径错误造成**。
6. 2026-10-10 单实例帧率对照：设置 640×480、25 fps，关闭 `color_pcl` 后单独订阅 RGB 和深度各 25 秒，分别实收约 13.57 / 13.55 Hz，消息时间戳年龄中位数约 14 / 13 ms；开启 `color_pcl` 时单路 RGB 约 13.55 Hz。关闭彩色点云未改善帧率，实际观测仍明显低于设置值；请确认官方应达到的持续帧率和测量方法。
7. Windows Angstrong Viewer V2.1.10_240109 截图中，640×480 RGB、Depth 和 3D Cloud 均显示约 31 fps，点云可见。Linux 原厂 ROS 2 驱动设置 30 fps 时，同一相机单独订阅 RGB 的 25 秒窗口实收 374 帧、约 14.88 Hz，消息时间戳频率也为 14.88 Hz；临时完全跳过点云处理的 25 fps 测试仍约 13.57 Hz。Windows 是否使用同一主机和 USB 端口尚待确认，两个软件的计帧口径也可能不同；请协助说明 Linux SDK 与 Windows Viewer 的帧率差异。

## 适配层另一个问题

厂商提供的 `CameraSrv.cpp` 中，`CameraSrv::start()` 把 `listener_callback.onDetached` 赋值为 `onAttached`。我们试验性改成 `onDetached` 后，2026-10-10 09:03:39 的真实 USB 断开触发了清理回调；日志显示 `stop stream success`，随后进程因 `../../libusb/os/threads_posix.h:58: usbi_mutex_destroy: Assertion pthread_mutex_destroy(mutex) == 0 failed` 退出（exit code -6）。**此改动已回退**，当前驱动未包含该实验改动。请确认官方推荐的断线重连处理方式，以及 `AS_SDK_StopStream` / `AS_SDK_CloseCamera` 在断开回调中的安全调用顺序。

## 希望确认

1. 这台 HP60C 的 SDK `v1.2.28.20241021` 与上述固件版本是否匹配？是否有更新的 Linux / ROS 2 SDK 或稳定版固件？请提供对应版本与升级步骤；在得到确认前我们不会刷写固件。
2. `get mjpeg from xu command failed` 的原因和影响是什么？`start streaming` 后约 1 秒 `stop streaming`、随后反复恢复/停流是否是已知问题？
3. 640×480 RGB + 深度同时输出时，USB 2.0 链路的官方支持帧率、供电要求、线缆规格和建议连接方式是什么？
4. 设备自行 USB 断连以及 `error -110` / `error -62` 该如何定位？是否有相机端诊断工具或日志可提供？
5. 请确认 `onDetached` 回调赋值以及断开后清理、重新连接的官方修复方案，避免 libusb 断言。

完整测试过程与时间点见 [HP60C 实机验收记录](hp60c-validation.md)。
