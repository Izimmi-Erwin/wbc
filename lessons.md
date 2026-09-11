# Project lessons

- 2026-09-11 — Context: 用户校正当前遥操 README 的实际操作流程。Mistake: 将键盘初始化描述为可选临时操作，仅列四终端，遗漏 ABXY 标定前的 k → 9 → backspace 初始化，以及 PICO Head、Controller、Send 勾选。Rule: 当前流程按五终端描述，终端 5 分步发送 k、9、backspace；放在模型和 manager 就绪之后、ABXY 标定之前，初始化已释放辅助后不再重复发送 9；PICO 设置同时写明 FullBody 与 Head、Controller、Send。README 仅保留用户要求的核心操作内容。
