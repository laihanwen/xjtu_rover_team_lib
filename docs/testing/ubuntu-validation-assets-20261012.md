# Ubuntu 验证代码与素材：2026-10-12

代码保存在 Git；素材为 GitHub Release `validation-20261012` 的独立附件 `ubuntu-validation-20261012.zip`，附 SHA256。录像、日志及 Windows 二进制不进入源码提交。

## 素材范围

- 978 个原始相机会话对应的 640×480 无损矫正样本，FFV1 AVI；包含标签、锥、白/黄管及补录 L 接头，配原文件/帧号/时间映射和标定清单。
- 32 个旧人工筛选原尺寸帧，无损 AVI；已矫正显示空间，不再套用新 K/D，配旧标注及 C++ 对照结果。
- 八个会话原始帧侧车与会话元数据；同期 57,821 条诊断日志的最小脱敏输入，只保留时间、有效性、缓存年龄、序号及姿态/深度。日志没有同钟定位输入，不能据此恢复精确米制定位。
- Windows 同版语义统计、同步审计统计和逐文件哈希。

不包含完整原始长录像、凭据、控制台完整缓存或构建产物。无损样本用于独立识别回归，不能把人工 1 FPS 播放时间当运动轨迹。完整 Runtime 动态模拟继续使用仓库合成夹具。

## 获取及构建

在干净或已保护本地改动的项目目录更新代码：

```sh
git pull --ff-only origin main
gh release download validation-20261012 --repo laihanwen/xjtu_rover_team_lib --pattern 'ubuntu-validation-20261012*' --dir /tmp/auv-validation
cd /tmp/auv-validation
sha256sum -c ubuntu-validation-20261012.zip.sha256
unzip ubuntu-validation-20261012.zip -d materials
# 回到仓库根目录继续执行以下命令
```

没有 gh 时从仓库 Releases 页面手动下载两个附件。素材可以放在任意目录，不要求复现 Windows 路径。

准备 C/C++、CMake、Ninja、OpenCV >=4.7、yaml-cpp、cpp-httplib；Runtime 回放夹具还需要 ffmpeg、Python OpenCV/NumPy/PyYAML。参考根目录原生构建说明，不使用 ROS/colcon。

```sh
cmake -S . -B build-ubuntu-validation -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build build-ubuntu-validation -j 3
ctest --test-dir build-ubuntu-validation --output-on-failure
python3 tools/auv/verify_ubuntu_materials.py /tmp/auv-validation/materials
python3 tools/auv/verify_ubuntu_materials.py /tmp/auv-validation/materials \
  --replay build-ubuntu-validation/core/auv_core/auv_task_video_replay \
  --output build-ubuntu-validation/real-semantic-replay
python3 tools/auv/audit_recording_sync.py \
  /tmp/auv-validation/materials/recordings \
  /tmp/auv-validation/materials/telemetry \
  build-ubuntu-validation/sync-audit
```

最后两项目录要求不存在，避免覆盖证据。包内清单绑定实际代码提交号，验证前检查当前 HEAD 与之对应；若不对应，保留各自版本，不能复用旧结果。

## 验收边界

Ubuntu 执行结果已保存于[验证记录](ubuntu-validation-results-20261012.md)：同版完整 Runtime 单次 27/27 CTest 通过，真实样本与同步审计汇总均复现参考结果；这不解除下述真实导航验收缺口。

本次 Windows 便携 16/16 通过不是 Ubuntu 完整 Runtime 通过；新附件也不是局部米制定位真值。完整外框拒绝、接头候选、unknown 锥类别以及缺同钟输入必须继续如实记录。实际 Runtime 局部路径默认关闭、运动关闭、自动 ARM 关闭；测试使用隔离目录和虚拟串口，不能连接生产 UART 或因此启动实机任务。

详见[前三项结果](first-three-offline-validation-20261012.md)及[局部 Runtime 验证边界](local-runtime-development-20261011.md)。
