# 最短标签路线与人工九宫格

2026-10-13 的软件范围：以启动后建立的相对机体原点为红框起终点，起点附近可沿实测航点小范围寻找 ID18，前视看到标签后接近，近距短时失明交给下视；下视连续确认 ID18 后沿实际出航位置回退到原点并停机。下视黄条连续三帧只记为 `YELLOW_EDGE_CANDIDATE`，不是网格方向或地图完成证明。人工在网页确认黄边现场方向和九宫格交通锥。

本配置默认 `motion_commands_enabled: false`、`observation_search.enabled: false`、`return_to_origin: true`。因此新源码部署后也不会自行绕行、ARM 或返航。标签摆在起点前视可见范围内时，无须另填终点坐标；回退使用本次运行记录的相对位置轨迹。启用前仍须实测全程无障碍半径、工作深度、运动方向和深度门，并完成带动力失效检查。若起点看不到标签，需额外填入相对起点（X 前、Y 左，米）的安全搜索航点，不能凭“略微绕场”推断可走区域。搜索航点必须在 `tag_docking.maximum_radius_m` 内；搜索速度不能超过标签接近上限。较远的标签如果处于这个已测走廊之外，当前任务仍会拒绝前往。

配置位置：[Pi 标签任务模板](../../runtime/config/pi-auv-tag-docking.yaml)。`observation_search.waypoints_m` 是找标签时经过的点；`tag_docking.finish_waypoints_m` 可选，用于替代轨迹回退，最后一项为终点。任一标签相机取得连续有效的 ID18 都可退出搜索。搜索航点走完仍未找到标签、定位过期、越界或超时都会停止并进入故障；不会漫无目标地低速前进。回到原点才 `APPROACH_COMPLETE` 并 DISARM。若关闭 `return_to_origin` 且不配置终点航点，保留原本下视确认即停机行为。

网页 [最短任务与人工九宫格](../../runtime/web/dashboard/minimal-task.html) 只读监视状态，提供九宫格人工填写、黄边现场方向以及当前机载事件段或本地 `events.ndjson` 的导入。输出 `manual-grid-*.json` 是人工提交地图；`annotated-events-*.ndjson` 保留所有原始事件并追加 `MANUAL_GRID_ANNOTATION`，列出与机载地图不一致的格位。不要把人工填写冒称传感器识别。机载 `events.ndjson` 可能轮转；网页接口只返回当前段，完整比赛日志还须取回轮转段、录像和运行配置。

本次增量范围：Windows 便携 `tag_docking` 单项测试、网页 JavaScript 语法检查。Pi 原生 Runtime 构建、模拟闭环、在线部署及水池路线仍需设备恢复后验证；此前版本的通过不能代表本次修改已部署。
