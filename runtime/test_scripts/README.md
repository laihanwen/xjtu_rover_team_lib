# runtime/test_scripts — 任务一运行时验收测试

对 `runtime/auv_runtime` 做「视觉建图 + 完整流程」黑盒断言：从 `input/read/` 读取
选定的合成输入 jpg，喂给运行时，验证 AprilTag 门控、九宫格语义地图、锥形物
circle/square 分类、任务一 FSM 阶段顺序与规划路径覆盖，并把全部产出归档到
`output/<时间戳>/`。

## 测试原则

- **确定性**：相同输入一定得出相同输出。场景用固定 seed 生成，断言只针对语义地图
  cell、FSM 阶段、分类与访问覆盖，不依赖墙钟/延迟/帧率。归档内容确定，仅目录时间戳
  不同（用于区分两次测试）。
- **安全**：`harness.py` 强制 `motion_commands_enabled: false`，serial 为 PTY 虚拟
  设备，不会发出真实推进器输出。

## 目录结构

```
/tmp/auv-observe/
├── input/
│   ├── scene_*.jpg        # 生成步骤产出：可能的输入池
│   ├── ground_truth.json  # 文件名 → 布局 / 是否含 tag / 期望 cells
│   └── read/              # 本次选中的输入（测试只读这里，删掉即不测）
└── output/
    └── <时间戳>/          # 每次运行新建
        ├── map_<场景>.png            # 3×3 语义地图
        ├── path_<场景>.png           # 规划路径（节点顺序）
        ├── grid_<场景>.png           # 建图角点叠加图
        ├── summary_<场景>.txt        # 阶段顺序 + 路径 + 每格分类 + 关键状态
        ├── events_<场景>.ndjson      # 原始事件日志
        └── status_<场景>_<阶段>.json # 各关键阶段 status 快照
```

## 两步流程

1. **生成输入**（场景集变化时运行一次）：

   ```fish
   python3 runtime/test_scripts/generate_scenes.py
   ```

   把所有可能场景写进 `input/`，并默认全选进 `input/read/`。想只测部分场景，删除
   `input/read/` 里不要的 jpg。

2. **构建 + 测试**：

   ```fish
   cmake --build build-lightweight -j 3
   python3 -m pytest runtime/test_scripts -v
   ```

## 文件

| 文件 | 作用 |
| --- | --- |
| `generate_scenes.py` | 生成步骤：合成所有输入 jpg + `ground_truth.json` |
| `scenes.py` | 合成渲染 + 布局/真值 + 输入 IO（读 `read/`、拼视频） |
| `harness.py` | 协议 crc/frame、PTY 虚拟 STM32、状态/阶段快照轮询 |
| `visualize.py` | 输出归档：map/path/grid/summary/events/status 快照 |
| `conftest.py` | `runtime_binary` fixture |
| `test_vision_mapping.py` | 每个含 tag 场景：建图/分类/阶段顺序/路径覆盖断言 |
| `test_mission_flow.py` | 无 tag 场景：AprilTag 门控断言 |

## 构建

```fish
cmake -S . -B build-lightweight -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
cmake --build build-lightweight -j 3
```

## 已知待办

- **SURFACE 时序差异**：比赛规则要求建图后上浮再遍历，但 `task_one` 目前是
  `BUILD_MAP → PLAN_CONES → VISIT_CONES`，中间无上浮阶段。等补齐 SURFACE 再补断言。
- **移动相机模拟（第二阶段）**：离线固定相机无法产生位姿变化，实际遍历/连续轨迹需
  移动相机模拟。当前「不重复」由 planner 层 CTest 兜底。
- **阶段快照粒度**：视觉线程先于任务跑，`SEARCH_APRILTAG` / `BUILD_MAP` 各只持续约
  一个控制 tick（~50ms），`status` 快照靠 10ms 轮询尽力抓取（`VISIT_CONES` 与无 tag
  场景的 `SEARCH_APRILTAG` 稳定抓取）；完整阶段顺序始终在 `events_<场景>.ndjson`。
- **动力输出可观测性（后续）**：后续需在测试里验证自动控制系统是否发出动力/推进器
  指令（当前 `motion_commands_enabled: false`，不发出运动指令）。

## 验收范围说明

本目录测试旧 task_one 模式的视觉、阶段顺序与规划路径覆盖；关闭运动输出，因此不证明真实访问。A2 的上浮后遍历、定位门控、连续路径与观测访问由 a2_surface_traversal 和 a2_indexed_surface_replay CTest 验证；这些也是模拟回归，仍需实机验收。

白边模式使用 vision.single_yellow_edge=true 和 vision.white_grid_edges=true；黑边场景设置 white_grid_edges=false。黄色边连续确认帧数使用 yellow_oriented_frames。
