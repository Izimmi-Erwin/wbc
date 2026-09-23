# 文档入口

当前版本的启动、五终端初始化、日常采集及转换命令统一见 [根目录 README](../README.md)。

| 文档 | 内容 |
| --- | --- |
| [厨房配置](wbc_kitchen.md) | 资产下载、碰撞、物体生成和机器人初始位姿 |
| [自动采集](wbc_kitchen_collection.md) | 成功判据、超时与复位边界、HDF5 字段 |
| [LeRobot 转换](wbc_lerobot.md) | 安装转换环境、筛选、字段和时间对齐 |

`source/` 保留 GR00T-WholeBodyControl 上游的算法、训练和部署参考。它不代表本项目的厨房操作流程，也不表示本仓库已发布该上游网站。

需要构建上游参考文档时，在独立文档环境安装 `requirements.txt`，然后在本目录执行 `make html`；输出为 `build/html/`。
