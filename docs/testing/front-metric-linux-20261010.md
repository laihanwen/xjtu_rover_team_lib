# 前视度量建图 Linux 软件验证

日期：2026-10-10。基线为远程主线 `fff5fbd` 与 PR #6 的 `80ff433`，加本次阶段矫正修复及回归补充。此记录不代表树莓派部署或水池验收。

## 已完成

- Ubuntu 原生 Release 编译包含 `auv_runtime`、HTTP 服务和两个回放程序；OpenCV 4.10、yaml-cpp、GCC 15.2、系统 Python 3.14。
- 完整 22 项 CTest 通过，包括 PTY 运行时、ARM/自主模拟、标签悬停模拟、地图回放、前视近水面回放、配置安全门、45° 斜视 PnP、外参拒绝和 A2 状态转换。
- 新增 `front_metric_streams`：使用两路模拟 CSI 和虚拟串口，主动前视度量配置在 INIT 下分别运行 5/20 与 20/5 fps。下视停帧不阻止前视处理；前视停帧不阻止下视采集，前视图像年龄超过新鲜度门限，恢复后重新处理。整个测试未启动任务、未 ARM，未访问真实设备。
- `dual_camera` 的模拟 CSI 按实际传入帧率发送，覆盖两种角色的低帧率、超过超时的停帧、子进程重启、另一相机和控制继续运行、退出清理。
- 水下/近水面阶段的组件回归检查保留原图、按各自 K/D 矫正、往返切换与重复矫正区别；现有 A2 状态机回归要求重定位后才能规划，不复用上浮前定位。前视索引回放校验选中前视模型与尺寸。尚未模拟完整 Runtime 从水下到四锥遍历的动态相机闭环。
- 阶段代码审查发现并修复：前视负责近水面定位时，下视仍保持自身水下矫正与元数据；不能选用此模式中未构造的下视近水面矫正器，否则预览会退回原图。视觉线程使用同样的阶段选择，复用正确缓存。
- 配置注释同步实现，删除未实测 30° 俯角假设；正式模板继续关闭运动、自动 ARM、前视度量及未经验证的标定。

## 复现

在装有 OpenCV、yaml-cpp、cpp-httplib 开发依赖的 Linux 仓库根目录：

```sh
cmake -S . -B build-native -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DPython3_EXECUTABLE=/usr/bin/python3
cmake --build build-native -j 3
ctest --test-dir build-native --output-on-failure -j 3
python3 tools/docs/audit.py
```

本机 cpp-httplib 未安装到系统；从 Ubuntu resolute 软件源下载 `libcpp-httplib-dev` 和 `libcpp-httplib0.26`（0.26.0+ds-2ubuntu3），解包到 `/tmp/auv-http-build-deps`，配置时额外指定 `-DHTTPLIB_ROOT=/tmp/auv-http-build-deps -DHTTPLIB_LIB_ROOT=/tmp/auv-http-build-deps`。构建目录为 `/tmp/auv-todo-linux-build`。没有修改系统 Python、训练环境或设备服务。

## 未完成

`ssh -o BatchMode=yes -o ConnectTimeout=5 pi@192.168.137.150` 连接超时，未到身份认证阶段。设备当前代码、配置、相机来源和服务版本无法核对；没有部署、烧录、重启或实机动作。

前视固定角度、内外参、近水面模型、相机与压力计偏移、场地尺寸、连续全图可见性仍需真实素材和实测。无动力安全联调、完整动态阶段切换、水池四锥闭环、并发资源预算和部署验收继续按 [待办清单](../auv-mapping-traversal-todo.md) 执行。模拟帧率不代表 Pi 实测性能。
