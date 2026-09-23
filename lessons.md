# Project lessons

- 2026-09-11 — Context: 用户校正当前遥操 README 的实际操作流程。Mistake: 将键盘初始化描述为可选临时操作，仅列四终端，遗漏 ABXY 标定前的 k → 9 → backspace 初始化，以及 PICO Head、Controller、Send 勾选。Rule: 当前流程按五终端描述，终端 5 分步发送 k、9、backspace；放在模型和 manager 就绪之后、ABXY 标定之前，初始化已释放辅助后不再重复发送 9；PICO 设置同时写明 FullBody 与 Head、Controller、Send。README 仅保留用户要求的核心操作内容。

- 2026-09-11 — Context: 用户明确确认第一视角相机的实际位置异常。Mistake: 用静态 USD 的预期偏移与 gui_perspective 启动参数提出可能选错视角，未拿到运行时相机位置。Rule: 停止选错视角方向，针对运行时相机局部 / 世界变换、父节点变换及重置前后状态采证；不能把静态资产计算当作相机实际位置正常的证据。

- 2026-09-11 — Context: 用户在 Isaac Sim 4.5 找不到所给的 Script Editor 菜单。Mistake: 未核实本机菜单注册就给出 Window → Script Editor 路径。Rule: 给出 Isaac GUI 操作前先检查已安装扩展的菜单注册与启用情况，写全中间子菜单；若未启用，提供对应扩展名称，不要求用户反复寻找错误入口。

- 2026-09-11 — Context: 用户复核后确认第一视角的问题是朝向过于向下，而不是相机位置。Mistake: 此前围绕位置异常与裁剪面解释展开，未最终定位视觉症状。Rule: 当前修正方向改为抬高相机俯仰，保持平移、已修好的视场角和裁剪范围不变；用光轴方向验证抬头正负号，不能再把位置异常作为已确认原因。

- 2026-09-11 — 用户要求记住并暂时撤销第一视角上抬方案，逐项测试。方案：在 gear_sonic/simulation_server/isaac_server.py 中定义 ISAAC_EGO_CAMERA_TILT_UP_DEGREES = 20.0；仅在 ego_view 分支读取 MJCF 欧拉角后、转换四元数前执行 local_euler_xyz[1] -= np.deg2rad(ISAAC_EGO_CAMERA_TILT_UP_DEGREES)。这使 Y 角从 -0.8 变为约 -1.14906585 rad，躯干直立时光轴从向下 44.163° 改为向下 24.163°；位置、上游 XML、第三人称和光学参数不变。已对独立 wbc 与原 FluxVLA/wbc 两份源码撤销此方案，恢复原俯仰，保留垂直 FOV 45°、近裁剪 0.02 m、远裁剪 100 m 修复。后续仅在用户要求时恢复上抬，逐项验证，不自动重新应用；运行中进程需用户重启后才加载源码变化。

- 2026-09-11 — 用户明确当前 PICO 应显示第三人称，保留现有第一视角相机参数供本地检查。旧 gui_perspective 发布器绑定 GUI 当前视口的 render product，GUI 切到 ego_view_camera 会把第一人称串入 PICO，恢复时还会因活动相机不是 /OmniverseKit_Persp 报错。Rule: PICO 第三人称输出固定绑定 /OmniverseKit_Persp 的独立 render product，不能跟随 GUI 活动相机切换；第一视角的姿态 / 光学调整不得改变视频路由。

- 2026-09-11 — 用户批准单项回退，以核查第三人称改动后的 PICO 视频断连。当前已撤销独立 render product 及其清理逻辑，两份 isaac_server.py 恢复复用 GUI 视口；保留 FOV 45°、裁剪 0.02–100 m 和原 MJCF 姿态，不恢复上抬 20°。此前“GUI / PICO 独立视角”的实现现已暂停，测试时 GUI 必须保持 Perspective。该回退用于对照验证，不能提前认定断连根因或恢复成功；不自动重启用户进程。

- 2026-09-23 — Context: 用户要求按 VS Code 当前打开的 WBC 对比远程仓库。Mistake: 助手沿用历史 FluxVLA/wbc 路径，未确认最近活动的独立 WBC 工作区。Rule: 先核实 VS Code 工作区路径及其 Git 根目录；本目录对应 Izimmi-Erwin/wbc，不能将另一份 FluxVLA/wbc 中的厨房、采集或转换功能视为本目录已有改动，未经要求不自动同步两份副本。
