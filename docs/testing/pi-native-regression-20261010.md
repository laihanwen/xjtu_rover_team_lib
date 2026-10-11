# Pi 原生回归：连接中断记录

## 本轮已完成

用户确认 Pi 恢复后，通过 PC 以太网地址 192.168.137.1 连接 pi@192.168.137.150，在独立目录运行验证，不替换服务，不使用真实 UART。

- `/home/pi/auv-offline-selected-20261010`：GCC C++17 编译并执行局部地图核心测试，退出码 0；语义地图适配 `semantic_map.cpp` 编译通过。
- 当前 C++ 真实素材回放再次编译通过。前视 17 / 下视 15 帧独立回放，不能找到完整闭合网格；未进入锥体分格分类，不将拒绝结果当分类失败率。
- 完整快照 `/home/pi/auv-full-regression-20261010-202704`：CMake 配置通过，使用 GCC 14.2、OpenCV 4.10，Release / Ninja / BUILD_TESTING=ON / 三路并行。
- 构建日志已到 34/57，包含 `runtime.cpp.o` 编译完成。尚未取得完整链接成功及 CTest 结果。

## 中断与诊断

SSH 连接被关闭，随后绑定以太网的 SSH 返回 WinError 10065。只读检查确认 PC 的“以太网”状态为 **Disconnected，0 bps**，不是凭据失败。不推断 Pi 重启或编译失败，也不删除已有构建缓存。

本地证据位于忽略目录 `build/task-video-review/pi-replay/` 和 `build/task-video-review/full-regression/`。后续新增了规划适配回归、轮廓类别一致性修正，以及 A2 模拟失败诊断保存；这些最新改动尚未完成完整 Pi 回归，不能套用此前独立编译结论。

本机最新局部地图 C++17 测试通过；相机安装几何 2/2、标定配置 1/1、模拟 Python 语法检查通过。

## 恢复步骤

恢复电脑到 Pi 的以太网物理链路后，先检查是否仍有远端构建进程；上传最新源文件和标定报告测试夹具，复用该目录增量构建。执行完整 CTest，保存 A2 的正常闭环和缺失表面观测场景结果。整个过程继续仅使用虚拟串口，不部署、不 ARM。

## 链路恢复后首轮完整回归

同一目录的最新完整 Release 构建通过。CTest 共 25 项，首轮 23 项通过、2 项失败：`a2_closed_loop` 在等待原点时进入 FAULT，原因是控制循环看门狗；`autonomous_simulation` 已完成 DISARM 确认，但测试立即断言夹爪串口消息，存在异步发送等待缺失。完整首轮日志保存在 `build/task-video-review/full-regression/ctest-first-failure.log`，A2 失败诊断在证据压缩包中。

代码核查发现控制线程读取定位快照会等待持有光流处理锁的定位线程。修复为独立短锁发布完整位姿：计算期间只读最近已完成样本，仍以原采样时间执行 0.3 秒有效期；失败和重置同步发布失效状态。没有放宽控制看门狗或定位新鲜度。夹爪测试增加实际虚拟 MCU 消息等待，不用状态改变代替发送完成。修复后的完整回归尚在执行，不能以首轮 23/25 宣称通过。

## 后续修复与当前结论

本轮源码完整 Release 构建已通过；最近完整 CTest 为 **24/25**，未通过项仍为 `a2_closed_loop`。后续新增节点关联后，CTest 配置为 26 项；针对节点关联、定位异常失效和 A2 的三项检查中，前两项通过，A2 仍失败。没有宣称 26/26 通过。

已完成的修复：下视文件回放按配置帧率限速（真实设备采集不变）；完全相同的矫正灰度帧复用恒等特征对应，但仍检查新鲜度、IMU/深度、地板几何、残差和速度；异常退出不能重新发布旧位置为新时间的有效位姿。现有移动图像回归仍要求非零位移。新增局部节点拓扑关联核心通过 Windows 与 Pi 测试，尚未接入 Runtime。

A2 证据已有一次到达 BUILD_MAP → SURFACE_FOR_CONES → RELOCALIZE_SURFACE → PLAN_CONES → VISIT_CONES，随后因位姿跳变停止，四锥访问并未完成。3.3 m 合成池深的低分辨率平面位姿抖动未放宽保护；正常场景改用 1.4 m 合成池深后，仍曾因下视定位快照过期停止。前视仍为合成垂直相机，不能替代真实 45° 可见性和连续导航验收。

## 当前需要处理的实机环境

两次只读采样均得到 `throttled=0x50005`、温度 58.4°C。按 [Raspberry Pi 官方位定义](https://www.raspberrypi.com/documentation/computers/os.html#get_throttled)，当前欠压和节流位均置位，同时有历史欠压/节流记录。需用户检查 Pi 供电，不能推断故障一定仅由供电造成。

独立模拟同时存在已部署摄像头进程 `/usr/local/bin/auv_runtime /etc/auv-runtime/runtime.yaml`，进程累计 CPU 约 185%，另有 rpicam-vid。硬件为 Raspberry Pi 4 Model B Rev 1.5，负载约 3.57 / 4.27 / 4.95。摄像头服务串口为空、运动关闭、自动启动和 ARM 关闭；ROV 桥为另一个仍运行的服务。

暂未停止任何现有服务。需确认当前任务和录像已停止，才可短暂停用摄像头 Runtime，做独立模拟后恢复。此隔离验证不能替代最终生产负载下的双摄、定位、录像并发性能验收。原始资源证据在 `build/task-video-review/full-regression/resource-status.log`。
