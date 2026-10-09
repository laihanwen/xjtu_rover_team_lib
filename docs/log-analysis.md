# 日志系统与 AI 分析

ROV 控制台自动写入 `logs/rov/<UTC时间_随机后缀>/`，普通快照间隔由 5 秒改为 1 秒；可选诊断仍为 0.1 秒。记录线程只读取缓存，不创建 UART 或相机连接。每段上限保持 10 MB，60 条近期记录供页面查看。磁盘写入失败经 `/logs.error` 显式报告。

保持 schema=1 兼容旧消费者，新增 schema_name、schema_revision=2、session_id、sequence、record_type、单位、validity、received_cache 和 changes。received_cache 是可能过期的接收缓存，不是有效实时值；imu/pid 仍只在新鲜时提供。changes 是相邻采样的差异，不是精确 MCU 事件；5 秒旧日志和 1 秒新日志都可能漏掉短暂变化。manifest.json 说明文件布局、时钟和限制。

ROV 日志区新增 `/log-viewer`；AUV 独立页新增 `logs.html`。查看器支持多文件 JSONL/JSON、本地搜索、故障/变化筛选、深度曲线、原始字段折叠及 AI 分析包导出。文件不上载。单次最多 20 MB、100000 条；显示最近 300 条匹配记录；解析错误包含文件名和行号。曲线按会话/文件与时钟分组，各组独立缩放；不做跨设备时间减法或自动故障归因。

AUV 原生日志与录像帧索引保持原格式，本轮增强其离线分析能力，不改实时 C++ 记录线程。机载文件需先复制到 PC，再导入。重复帧携带的遥测不能当作独立深度样本；未知值不填零。AI 包包含原始记录与来源，须结合运行配置、标定和视频核对，不视作运动许可。

已验证 ROV 日志测试、Python 回归、查看器脚本语法及页面检查；没有部署 Pi 或执行实机控制。启动离线模拟服务：`python tools/console/preview.py --port 8771`；分析页 `/dashboard/logs.html`。
