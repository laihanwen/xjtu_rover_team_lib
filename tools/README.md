# 工具入口

工具按实际职责分组，不把独立 ROV / AUV 控制入口合并。

| 目录 | 职责 | 说明 |
|---|---|---|
| rov/ | 原遥控控制台、串口桥、维护、录像、日志和人工采集 | [ROV 手册](rov/README.md) |
| auv/ | 独立固件构建、标签任务检查/待命、离线证据分析 | [AUV 工具](auv/README.md) |
| console/ | 独立 ROV / AUV 浏览器模拟，无硬件控制 | [模拟说明](../docs/reports/20261010/console-refresh-20261010.md) |
| docs/ | Markdown 本地链接检查及文档索引生成 | `python tools/docs/audit.py --write-index` |

维护入口不会因为编译、记录、文档更新而得到烧录或 ARM 授权。依赖、凭据、日志和视频放在忽略目录中，路径与配置见对应手册。
