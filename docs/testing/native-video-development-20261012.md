# 现有录像驱动的任务一开发：2026-10-12

本轮以控制台原始录像继续开发，不要求泛化补录。没有烧录、部署、ARM 或启动实机运动。独立 ROV 控制与固件未修改。

## 录像事实与任务路线

新增会话 `20261011T164144_825943Z`（北京时间 10 月 12 日 00:41:44）包含约 139.55 秒下视素材，4191 帧；前视 5409 帧。侧车时间戳最大相邻间隔分别约 0.0477 / 0.0375 秒，没有超过 0.5 秒的间隔。抽查能看到黄色边与白管 L 转角，也有瓷砖线、遮挡、画外片段；这些统计不等于逐帧识别通过。

现阶段任务保持“标签触发 → 重叠局部观测累积 → 地图证据齐备后冻结 → 上浮 → 新阶段重定位 → 四锥访问”。不依赖某帧完整九宫格：L/T/十字的分支、颜色与相邻观测用于局部关联；重复内部十字不能单独确定绝对格号。画外和遮挡格保持 unknown。四锥完成后的出口控制仍待实现，不把目前状态机结束称作完整任务一完成。

## 本轮改动

- `tools/auv/prepare_native_video_replay.py`：读取实际会话的内参、畸变和原图/矫正标记；原图只矫正一次，保持原尺寸及 K，生成 FFV1 无损 AVI。按 Pi 采帧单调时间抽样，拒绝缺帧号或倒序时间；保存原文件/原帧/时间映射与 SHA256。输出为独立语义样本，播放时间人为设置，不能当连续运动或同步遥测。
- `auv_task_video_replay`：tag16h5 检测独立于完整外框，输出 ID 及四角像素坐标，并记录 ID18；完整外框失败不能抹掉标签证据。CSV 拒绝原因支持逗号和引号。格位与锥检测仍是原完整外框链，不冒称局部验证。
- `auv_local_mapping_replay`：修正未传入上一帧地标位姿的问题。在已有 0.3 秒、位置和旋转连续性门内允许时序消歧；阶段切换清空 prior。不会从稀疏样本生成虚假里程计。
- `LocalMappingPipeline`：时序关联失败后同步输出最新拒绝原因，避免诊断仍显示此前原因。

## 运行与结果

具备 Python OpenCV / NumPy 的环境：

```sh
python tools/auv/prepare_native_video_replay.py data/rov-recordings/20261011T164144_825943Z down build/task-native-review-20261012/l-down-lossless --interval 1
# 编译后逐个输出 AVI 执行，必须保持独立观察：
auv_task_video_replay build/task-native-review-20261012/l-down-lossless/down_0001.avi build/task-native-review-20261012/cpp-l-1 1 --independent
```

输出目录要求不存在，避免覆盖旧证据。L 素材已实际生成 138 个无损矫正样本（59、59、20），映射和哈希保存在本地忽略目录 `build/task-native-review-20261012/`。标签/锥/管线代表会话另准备双摄样本。时间戳规则的两项 Python 测试通过。

本次 Windows C++ 构建未通过配置阶段：旧缓存引用的 `4ebd/.../build/deps/opencv/build` 开发包已不存在，不能沿用历史编译结论。Python cv2 不能替代该依赖。新增 C++ 源码待恢复开发包或 Linux 隔离编译后验证，尚未取得这些真实样本的同版 C++ 识别结果。

## 下一步

恢复原生编译后，先运行真实语义样本，关联人工接头回访 ID 与锥体轮廓；逐帧米制局部回放须从真实遥测建立时间关联，不能将 PC 收帧 UTC 直接视为 Pi 传感器采样时刻。随后补局部 Runtime 动态闭环和出口控制。当前没有新增补录要求；只在实际回归发现不可用片段时提出具体补录区间。


## OpenCV 恢复后的验证

用户将 OpenCV 解压到 `C:/Dev/opencv` 后，新建 `build/windows-vision`，直接从 `core/auv_core` 配置 MSVC x64 Release，CMake 确认版本 4.10.0。该核心构建成功，9/9 CTest 通过；受限执行环境首次出现 DLL 加载失败，允许读取实际开发包后同一二进制全量重跑通过。未修改系统 PATH，仅为测试进程设置 DLL 路径。

同版 C++ 完成 33 段无损样本视频、978 个独立样本的回放。ID18 解码样本 44 个，完整外框几何有效样本 0 个。样本数不是准确率；没有逐帧人工真值，不计算召回率。外框拒绝不代表局部接头算法失败。逐帧结果、角点、原始映射及二进制哈希保存在 `build/task-native-review-20261012/`。

本次只编译核心，不含需要 yaml-cpp 的 Runtime 配置与局部回放目标；`runtime/src/local_mapping_replay.cpp` 的新增改动尚未编译验证。


## yaml-cpp 恢复后的完整便携回归

2026-10-12，从用户解压的 `C:/Dev/yaml-cpp-yaml-cpp-0.9.0` 编译 Release 静态库，安装在项目忽略目录 `build/deps/yaml`。根目录全新 `build/windows-native` 成功构建（OpenCV 4.10.0、yaml-cpp 0.9.0、MSVC x64、`AUV_PERCEPTION_ONLY=ON`），包括新增局部回放 prior 改动。全部 16/16 CTest 通过，覆盖局部管线、地图累积、关联、光流、任务核心、标签对准、矫正、记录、配置及表面回放。依赖源码有类型转换警告，CMake 有上游旧目标名弃用警告，未阻止编译。

二进制哈希与验证范围保存在 `build/windows-native/verification.json`，原始 CTest 输出在该构建目录 `Testing/Temporary/LastTest.log`。Windows 便携验证不包含完整 Linux Runtime 动态闭环，也不代表真实局部米制录像回归、部署或下水验收。
