# 2026-10-10 仓库整理与交付

当前文档按入口、操作手册、日期证据和归档分层：根 README / docs/README / project-status 负责当前路线；docs/reports/20261009 与 20261010 保存近期部署和复盘；docs/archive/dual-mode 保存已弃用的双模式设计，禁止作为操作流程。工具按 rov、auv、console、docs 分组，现有运行脚本路径保留，避免破坏用户入口和服务配置。

对全仓库 Markdown 检查本地目标链接并生成 catalog.md。当前说明更新独立固件、日志分析、人工任务证据和最新平台验证；历史验收记录保留原时点事实。新增 tools/docs/audit.py，供后续增改文档时复查和刷新索引。

本次包含待提交日志增强、离线证据工具和 SurfaceRouteExecutor 到位停移改进。验证范围：Windows 便携核心 12/12 CTest（本次相同算法源码）、ROV Python 43 项中 42 通过/1 可选跳过、AUV Python 14/14；前端日志脚本语法及 Markdown 本地链接检查通过。完整 Linux Runtime 20/20 属于 10 月 9 日对应源码记录，本轮没有新 Linux 全量构建。

未提交实际录像、遥测、凭据、虚拟环境、依赖和编译输出；没有烧录、部署实机、ARM 或运动参数变更。本机模拟仍可运行，但不代表机器人任务验收。
