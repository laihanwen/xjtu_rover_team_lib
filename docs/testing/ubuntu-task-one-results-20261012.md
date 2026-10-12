# Ubuntu 任务一增量验证结果：2026-10-12

**完整 Runtime 编译通过，最终 CTest 33/34；局部任务闭环未通过，不可据此部署自主导航。** 本轮没有 SSH、部署、烧录、生产 UART 或实机运动。

## 版本与环境

- 接收基线：`122f831c6b124ae7b6d7ff2d7a30cfc0bea97093`。
- 配置/观测与夹具修复：`55569ff4e9a588c8339acfb694946ca5f29b48eb`。
- 最终执行源码：`d0577ff8e95db70cb0258459bcf069001f464cda`，测试开始时工作区干净。
- Ubuntu 26.04.1 / x86_64，GCC 15.2，OpenCV C++/Python 4.10.0（含 DNN），yaml-cpp 0.8.0，cpp-httplib 0.26.0，系统 Python 3.14.4、NumPy 2.3.5、PyYAML 6.0.3。
- cpp-httplib 使用 Ubuntu 软件包解包到 `/tmp/auv-task-one-http-deps`，没有安装或修改系统 Python、ROS、CUDA、训练环境。FFmpeg CLI / 可选 imageio_ffmpeg 未安装；HLS 与独立 FFmpeg 解码不在本轮通过范围。

完整构建包含 HTTP、`auv_runtime`、局部/表面/语义回放目标，非便携裁剪构建。保留原有 `auvctl` 忽略 write 返回值的编译警告。

```bash
cmake -S . -B build/task-one-ubuntu-20261012 -G Ninja \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DAUV_PERCEPTION_ONLY=OFF \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DHTTPLIB_ROOT=/tmp/auv-task-one-http-deps \
  -DHTTPLIB_LIB_ROOT=/tmp/auv-task-one-http-deps
env PYTHON=/usr/bin/python3 JOBS=3 bash tools/auv/validate_task_one_ubuntu.sh build/task-one-ubuntu-20261012
```

## 回归结果与未通过项

| 执行源码/范围 | 结果 | 证据目录（位于 build/ubuntu-task-one-evidence） |
|---|---|---|
| 接收基线完整检查 | CTest 32/34，HTTP 配置与局部启动失败 | `20261012T121355Z` |
| `55569ff` 完整检查 | CTest 33/34，局部位姿对照发现约一列误关联 | `20261012T124510Z` |
| `d0577ff` 最终完整检查 | CTest 33/34，0 个 CTest 跳过，总计 89.33 秒 | `20261012T125040Z` |
| 最终 Python 专项 | 29 项，28 通过、1 跳过 | 最终目录的各专项日志 |

Python 细项：同钟记录 2/2、同钟回放准备 3/3、训练清单 2/2、ROV 协议 10/10；录像 12 项中 11 通过、1 项可选独立 FFmpeg 解码因依赖缺失跳过。没有真实主体掩膜，不运行真实训练或宣称方案 B 权重 ready。

最终通过的 CTest 包括 `perception_http_readonly`、`perception_view_freshness`、`shape_model_inference`、`dual_cone_vision`、`tag_grid_binding`、`grid_exit_closed_loop`、局部地图/关联/融合核心、原点/门控、双摄与完整外框 `a2_closed_loop`。HTTP 测试在 INIT/DISARM、空串口、关闭运动下实际解码 ID18、返回 JPEG 与 unknown 地图。形状推理使用合成 ONNX，只证明接口可执行。

唯一失败为 `local_task_one_closed_loop`，最终阶段为 `INIT → SELF_CHECK → SEARCH_APRILTAG → BUILD_MAP`。位姿对照断言捕获：估计 `[1.49748, 1.66804]`，虚拟 MCU 积分位置 `[1.6884314, 1.7683688]`，相差约 0.2157 格；夹具格距 0.5 m，约 0.108 m，超过夹具 0.2 格对照限值。失败时地图未冻结，四锥未确认。

前置定向运行还记录了唯一地标丢失、重复管网歧义、里程计预算耗尽、顶部格 unknown，最后观测走廊耗尽进入 FAULT。不能将这些拒绝写成导航通过。组合测试先执行正常局部场景，正常场景失败后其“缺失表面观测”子场景没有执行；局部上浮、阶段重定位、四锥及 EXIT_GRID 完整链路均未验收。完整外框 A2 的正常/缺失场景通过不替代此项，退出组件单测也不替代 Runtime 离网格。

## 已保留的修复

1. 仅允许 `127.0.0.1` 用于 debug、空串口、关闭运动/自动启动/自动 ARM 的只读配置；新增硬件串口、运动与 `0.0.0.0` 拒绝回归。原网线地址与控制安全门不变。
2. 局部地图在标签触发后继续已标定观测走廊，直到证据齐备；原完整外框路径继续保持标签后定点观测。走廊耗尽记录“地图未完成”，不误称没有标签。
3. 合成地板保持在夹具地板颜色范围内，加入可经缩放/JPEG 保留的纹理；修正局部投影、标签手性及虚拟 MCU 横移方向，并新增有效位姿与实际积分位置对照。未扩大定位误差限值，夹具前视仍为原 250 焦距、320×240，持续断言不能看到整个外框。
4. 长时间丢地标后，连续同会话里程计可否决相邻格误重定位；它不能在预算外提供控制位姿。新增长间隔相邻列拒绝与一致新地标恢复回归通过，但这不解决全部 PnP/关联偏差，最终动态对照仍失败。
5. `LOCAL_GRID` 增加独立几何拒绝原因，避免只看到融合层预算过期而丢失上游诊断。缺失表面场景必须实际进入重定位阶段才能算负向验证。

管中心细化、角度估计及扩大合成视野的诊断尝试没有解决完整闭环，已撤回；差异留在证据包的 `rejected-geometry-experiments.patch`，不是最终源码。没有放宽生产定位、关联、帧龄、漂移、未知格或越界门，也未修改实测硬件参数。

## 真实素材复核

复用原附件的素材来源 `94235ae`，新审阅包交接来源 `122f831`、Windows 分析来源 `f447fca`；本轮最终执行程序来自 `d0577ff`。两份压缩包及 212/55 个清单文件哈希通过。已矫正稀疏样本不再次去畸变，不当作运动时间。

| 样本 | 帧数 | ID18 | 完整网格 | 红色组件帧 | 管段候选帧 | 管中心候选帧 |
|---|---:|---:|---:|---:|---:|---:|
| 旧素材（含32个筛选帧） | 1010 | 44 | 0 | 280 | 250 | 70 |
| 已填工作簿采用帧 | 39 | 1（F035） | 0 | 23 | 30 | 18 |

旧统计与参考汇总一致；39 帧通过包内分组 index 的 F 编号对齐，`frames.csv` 逐帧结果与 Windows 对照一致。39 帧均只有部分外框；完整四角链拒绝不是局部算法验收结论。颜色/管线/中心线仍为 unknown/unverified 候选，自动锥分类/格位真值未成立。未启用生产 `local_pipe_centres`，不训练或生成虚假真实权重。

同步审计仍为八个会话、181,708 帧、57,821 条诊断记录，151,546 个 PC 收帧新鲜候选；同钟定位输入与米制就绪帧均为 0。没有用 PC UTC、1 FPS、池深常数或重复深度归零补造位姿。

## 证据与回传

证据附件发布在 [Ubuntu 感知交接 Release](https://github.com/laihanwen/xjtu_rover_team_lib/releases/tag/ubuntu-perception-validation-20261012)，文件名 `ubuntu-task-one-evidence-20261012-d0577ff.tar.gz`，附 SHA256。包内包括首轮与最终日志、失败和定向检查诊断、配置、事件、UART 目标、末帧、源码差异、逐帧回放/F编号对齐、同步审计及可移植逐文件哈希清单。普通 Git 不放原始日志、录像、权重、构建产物或凭据。

证据包大小 29,026,033 字节，包含 604 个证据文件及清单，SHA256 为 `8b826c86f16405a5f9a91add64883ea285588d2019b8459c6422735a3ba90ee4`。

本地证据：`build/ubuntu-task-one-evidence/`；最新样本输出：`build/task-one-ubuntu-20261012/filled-review-final` 和 `real-semantic-final`；汇总：`semantic-summary.json`。初版与中间失败证据均保留，不覆盖用户工作簿。

最终二进制 SHA256：

```text
4da394d0a98e0b380e11e7db3f6910d6b326a21d4092b7ff98acbaf7601539f2  runtime/auv_runtime
b9ac4b3dfd3478aa3a51a32f59fa34fcb8b9b13802b1882b45f4e45fe624c186  runtime/auv_local_mapping_replay
6b756338a87f1ec82852a4c22c449b0ddbd8d3e5d978402f1c895cc4ad7bbfb3  core/auv_core/auv_task_video_replay
```

下一步优先从最终失败帧核对管中心坐标、局部 PnP 高度/姿态、候选关联与连续里程计误差，修复后重跑局部正常与缺失表面子场景。真实同钟遥测、精细主体/格位/节点真值仍缺；完整软件与水池验收之前继续保持生产局部导航、退出、运动和自动 ARM 门控。
