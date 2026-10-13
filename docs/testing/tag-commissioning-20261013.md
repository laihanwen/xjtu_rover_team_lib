# 标签测试标定补充：2026-10-13

用户确认压力计有有效原始读数，水面零点为 -0.05 m。自主固件源码已设置零点与确认标记，校正深度为原始读数 +0.05 m；Runtime 定位 depth_zero_m 保持 0，避免重复扣零。AUV_TAG_DOCK 编译零错误零警告，新 HEX SHA256 为 edc473968123b7ec4e6e858971ab104a8c98ac310134a22be2512794808e9489，尚未烧录；当前设备仍对应此前 dc6ef06c 镜像，本地新 HEX 与旧烧录记录不同，标签入口会拒绝记录不一致。

相机位置再次确认：前视 [0.15,0.03,0.02] m，下视 [0.09,0.01,-0.06] m，与已有配置一致。旋转沿用已有安装约定，不把位置确认当作导航精度验收。约 50 cm 接近范围记录为 maximum_radius_m=0.5，corridor_verified 仍为 false，等待净空及原点范围确认。

ROV 与 AUV 使用同一混控基础。ROV 相对定深与 AUV 绝对定深有独立 PI 参数、方向和标定门，不能将推荐参考 ROV 理解为全部自主增益已完成实测；本次没有修改推力增益、方向或总运动许可。

## Yaw 网页

```powershell
python tools/auv/yaw_test_web.py
```

打开 http://127.0.0.1:8771/ 。页面仅 GET Pi 的 /api/status，不提供控制命令。DISARM、串口/STATUS 新鲜时记录起点，俯视逆时针手动左转 30–90°，记录终点；再反向复测。建议符号只有超过 20° 才产生；±180° 自动展开，断线、会话变化、ARM、跳变使记录无效。综合传感器故障不能证明独立 IMU 健康，证据保留故障位；结果需左右转一致并由操作员确认，再应用到 IMU 转换。

导出 JSON 保存方向、角度变化、接收墙钟、会话和状态年龄；不把 PC 接收时间当 MCU 采样时间，不自动写回标定。已验证 JS 语法、角度展开、符号建议、会话失效与陈旧状态拒绝；本机页面和 JS HTTP 200。Pi SSH 超时，机载部署未成功，尚未获得实测 yaw 结果。

## 00:32 实测结果补充

Pi 恢复后发现 ROV 桥持有 UART、摄像头 Runtime 串口为空；本机网页增加只读复用 PC 8767 /state，不抢占串口。用户导出 yaw-calibration-1791822729435.json，证据已复制至 build/auv-entry/yaw-confirmed-20261013/，附 SHA256。有效左转累计 +87.5967°、右转累计 -84.7108°，均建议 yaw 符号 +1；初次 -16.9046° 未达到 20° 阈值，不作为结论。两次有效测试状态年龄最大约 0.102 s，未记录失效。故障位 2 仍存在，yaw 符号确认不替代深度和全系统安全验收。

标签模板 imu_signs 更新为 [1,1,1]：roll/pitch 沿用历史实测，yaw 使用本次双向证据。未部署该配置、未烧录新版固件、未请求 START/ARM。

## 00:34 后烧录与实机检查

按用户要求，零点修正版已烧录，完整 32 KiB 读回通过，原镜像已备份到 build/maintenance/20261012T163422679366Z/。先确认 DISARM/八路中立并停止冲突服务。Pi 标签配置已备份并更新用户确认的 IMU 符号、相机位置/历史安装旋转、传感器位置、池深、0.5 m 接近范围和实际烧录哈希；原生配置检查通过。运动、自动启动和自动 ARM 均保持关闭，几何/深度/走廊未验证标志未伪造。

空气中 MCU 只读内存采样显示 M10 持续发送 Depth:-0.20m；按 -0.05 m 零点得到 -0.15 m，绝对深度门拒绝符合设计。用户随后将传感器放到水面参考位置，十次 HTTP 采样测量约 0.02–0.03 m，压力序号 514–524 连续更新。串口、两路相机和记录器正常，保持 INIT/DISARM。前视检出 ID18，但出现 pixel_quality_rejected 或 geometry_unverified，下视未检出。

检查发现深度新鲜度使用 tag_docking observation_timeout_sec=0.3 的图像预算，约 0.55 s 的独立 M10 更新周期中出现间歇拒绝，需将传感器采样超时与图像超时分离并验证。MCU 综合故障位 2 不能在此次阶段直接归因于深度故障：独立 AUV 初始化 external_safety_seen/external_sensors_valid=0，工程没有调用 AuvLink_SetSafetyInputs；安全输入集成仍缺。自主推力增益/定深门、水平参考、定位原点及标签度量几何尚未验收，不具备任务 ARM 条件。没有放宽安全门、发送 START/ARM 或推进器测试命令。

本地证据：build/auv-entry/zero-flash-20261013/，包含空气中只读 MCU 采样、入水状态序列、双摄快照和 SHA256 清单；配置修改前后证据保存在 build/auv-entry/tag-measured-*/。软件问题与标定问题分别处理，不能把本轮配置检查当作控制闭环通过。

## 软件修复验证

标签压力采样改为独立 depth_sample_timeout_sec，默认 2.5 s、最大 3 s，不再用 observation_timeout_sec=0.3。检查仍要求有效标志、非负传感器年龄、接收时间有序及总年龄有界；相机、定位和控制循环门保持原值，重复采样仍不能推进定深完成判定。

AuvLink 初始化安全策略改为：仅 ROV 或 AUV_AUTONOMY_READY 完整标定构建可建立板载传感器安全输入资格；随后仍检查真实深度、IMU、水平校准、KILL 和心跳。未来外部安全输入可经现有接口覆盖资格。当前未标定 AUV 仍为拒绝状态，不伪造漏水检测，也不写入 GPIO。编译后的未标定默认镜像哈希未变，故无需为了该策略重复烧录相同镜像。

Windows MSVC 固件主机 16/16 通过；Keil AUV_TAG_DOCK 零错误零警告；OpenCV 4.10 / yaml-cpp 0.9 便携 tag_docking 1/1 通过，包含慢压力采样、过期/无效/时间倒退/NaN拒绝及 YAML 独立预算检查。这不是完整 Linux Runtime 构建或 PTY 集成验证。未将该 Runtime 修复部署到 Pi，未改变当前服务、未发送 START/ARM。

Linux 后续验证（已有完整构建目录）：

```fish
cmake --build build-native -j 3
ctest --test-dir build-native -R '^(tag_docking|tag_docking_simulation|runtime_pty)$' --output-on-failure
```

通过后再按独立标签发布流程备份、部署和检查。当前自主增益、定深、水平参考及标签度量验收仍有缺项，修复两个软件阻碍不等于已能启动动力任务。
