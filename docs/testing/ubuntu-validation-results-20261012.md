# Ubuntu 验证结果：2026-10-12

源码：`94235ae4f3ded6aac9fe2d1047ca621951b99ce5`。素材清单绑定同一提交，工作区无源码改动。本次执行离线验证，未连接生产 UART、部署、烧录或启动实机任务。

## 结果

- Release 完整 Runtime、HTTP 服务及局部/表面/语义回放目标构建通过。OpenCV 4.10.0、yaml-cpp 0.8.0、GCC 15.2、系统 Python 3.14.4。仅有原有 auvctl write 返回值未处理警告。
- 全量 CTest 单次 27/27 通过，总计 82.54 秒。包含 A2 正常闭环和缺少表面观测场景（37.88 秒），双摄不同帧率/断流、PTY 门控及局部核心回归；不代表真实局部导航闭环验收。
- Release 压缩包 SHA256 校验通过，212 个素材文件逐一校验通过。
- 35 段视频、1,010 个独立语义样本回放完成：ID18 44 帧，红色组件 280 帧，管段候选 250 帧，完整网格几何有效 0 帧。所有完整外框拒绝原因为 `no closed grid boundary`。旧人工筛选前视17帧、下视15帧均未解码 ID18。汇总与 Windows 参考结果完全一致；这些计数不代表准确率或召回率。
- 同步审计与参考 JSON 完全一致：八个会话，181,708 帧、57,821 条遥测；PC 接收时间新鲜候选 151,546 帧；同钟定位输入 0 条，米制就绪帧 0。缺少同钟传感器输入及已验证绝对深度/完整 IMU 转换，未运行或伪造真实米制定位。
- 123 份 Markdown 文档链接审计通过。

## 证据

- `Testing/Temporary/LastTest.log`：完整 CTest 原始输出。
- `real-semantic-replay/`：逐帧 CSV、标签角点、unknown/unverified 候选与叠加图。
- `sync-audit/summary.json` 与逐会话 associations JSONL：同步审计证据。
- 下载素材：`/tmp/auv-validation/materials`；原包 `/tmp/auv-validation/ubuntu-validation-20261012.zip`。
- cpp-httplib 从 Ubuntu resolute 软件包解包至 `/tmp/auv-http-validation-deps`，没有安装到系统或修改系统 Python。临时依赖和素材目录可能在重启时丢失，结果 CSV、测试日志和本报告保存在仓库忽略的 build 目录。

SHA256：

```text
c00bc14aebf450006d02bf101ae664fbb60cc36fe2afb2c6427d661da2e8696f  ubuntu-validation-20261012.zip
28a02e5d9e165a01335d7b002976af1a662c6696ed908be8662fe7e60542c41f  runtime/auv_runtime
04eb8ea5c0ce798c68d309e4a5f3a388d6f656f4e1415b275a5636dd4f4ec910  core/auv_core/auv_task_video_replay
```

## 复现

从仓库根目录执行，回放和审计使用新的输出目录，不覆盖本次证据：

```sh
cmake -S . -B build/ubuntu-validation-20261012 -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DPython3_EXECUTABLE=/usr/bin/python3 -DHTTPLIB_ROOT=/tmp/auv-http-validation-deps -DHTTPLIB_LIB_ROOT=/tmp/auv-http-validation-deps
cmake --build build/ubuntu-validation-20261012 -j 3
ctest --test-dir build/ubuntu-validation-20261012 --output-on-failure
/usr/bin/python3 tools/auv/verify_ubuntu_materials.py /tmp/auv-validation/materials
/usr/bin/python3 tools/auv/verify_ubuntu_materials.py /tmp/auv-validation/materials --replay build/ubuntu-validation-20261012/core/auv_core/auv_task_video_replay --output NEW_SEMANTIC_OUTPUT
/usr/bin/python3 tools/auv/audit_recording_sync.py /tmp/auv-validation/materials/recordings /tmp/auv-validation/materials/telemetry NEW_SYNC_OUTPUT
```
