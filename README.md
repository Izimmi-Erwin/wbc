# Isaac WBC：PICO 遥操作与厨房数据采集

使用 Isaac Sim 4.5.0、GEAR-SONIC WBC 和 PICO / XRoboToolkit 遥操作 G1（29 个身体关节 + 14 个手部关节），在 FluxBisim 厨房中采集“把香蕉放进红盘子”的轨迹。

红盘子固定生成在中岛，香蕉在水池右侧台面的 3 cm 圆内随机生成，机器人初始化在香蕉右侧。中岛和 L 型橱柜已启用碰撞。厨房接入保留原 WBC 控制器和五终端流程。

## 运行前准备

首次使用先完成文末的 [安装与模型准备](#安装与模型准备)，再下载 FluxBisim 代码和厨房资产。

命令中的 `/path/to/...` 和 `YOUR_PICO_IP` 是占位符，执行前替换为自己的实际路径和 PICO IP。五个终端分别设置所需变量，使用同一份 WBC 代码；终端间不会自动共享环境变量。

1. 确认 XRoboToolkit PC Service 运行。需要手动启动时执行 `bash /opt/apps/roboticsservice/runService.sh`；manager 也会尝试启动服务。
2. 戴好追踪器和手柄，完成身体追踪标定。在 PICO 原生 XRoboToolkit 中选择 **FullBody**，勾选 **Head、Controller、Send**，连接 PC 的局域网 IP。
3. 核对 PICO 自己的 IP，供终端 4 使用。PC 与 PICO 需能通过局域网互通。
4. 如果旧副本仍在运行，先停止原遥操再切换；同一台电脑不要同时启动两套仿真或 manager。

## 首次准备：下载 FluxBisim 代码和资产

首次安装需要另外下载 [FluxBisim 代码](https://github.com/FluxVLA/FluxBisim)。本仓库的 Git 忽略了 `FluxBisim/`，仅克隆 WBC 不会自动带上它。在 **WBC 根目录**执行：

```bash
cd /path/to/wbc
git clone https://github.com/FluxVLA/FluxBisim.git FluxBisim
```

将 `cd` 路径替换为自己的 WBC 根目录。若已克隆 FluxBisim，可复用该目录，保留其中已下载的资产。

**代码与资产需要分别下载**：GitHub 克隆完成后，还需按 [厨房资产下载说明](#下载厨房资产) 从 Hugging Face 下载厨房、盘子和香蕉到 `FluxBisim/assets/`。目录应为：

```text
wbc/
├── gear_sonic/
├── gear_sonic_deploy/
├── run_wbc_kitchen.sh
└── FluxBisim/                 # GitHub 代码
    └── assets/               # Hugging Face 资产
        ├── environments/KitchenRoom/
        └── pick_place_fruit/
            ├── plate/
            └── banana/
```

当前接入复用 FluxBisim 的场景资产和相关实现，无需另行启动它的双臂 benchmark 或安装 ROS Noetic。

### 下载厨房资产

安装 [uv](https://docs.astral.sh/uv/getting-started/installation/) 后，在已克隆 FluxBisim 代码的 WBC 根目录下载厨房、盘子和香蕉。代码与资产分别来自 GitHub 和 Hugging Face；仅克隆代码不能启动厨房场景。

```bash
cd /path/to/wbc
uvx --from huggingface_hub hf download limxdynamics/FluxBisimAssets \
  --repo-type dataset \
  --revision 1a5b6336d7752c3b605b196ae9a4e159bef3c028 \
  --include 'environments/KitchenRoom/**' 'pick_place_fruit/plate/**' 'pick_place_fruit/banana/**' \
  --local-dir FluxBisim/assets
```

厨房资产约 1.8 GB。保持目录结构，场景包装层通过相对路径引用这些文件：

- `FluxBisim/assets/environments/KitchenRoom/kitchen_room.usd`
- `FluxBisim/assets/pick_place_fruit/plate/base.usd`
- `FluxBisim/assets/pick_place_fruit/banana/banana.usd`

### 场景与生成位置

配置位于 `gear_sonic/data/scenes/fluxbisim/kitchen.usda`：

| 对象 | 初始设置 |
| --- | --- |
| 红盘子 `/World/plate` | 中岛 `(0.60, -0.05, 0.75)`，随机半径为 0，质量 0.2 kg |
| 香蕉 `/World/banana` | 水池右侧 `(0.55, -2.00, 0.87)`，XY 随机半径 0.03 m，质量 0.15 kg |
| 机器人 | `(-0.10, -1.90, 0.757)`，yaw 为 -9°，辅助锚点和目标朝向同步调整 |

这些是重置时的生成位姿，物体随后受重力自然落下；固定生成不代表锁定盘子的物理运动。中岛的 16 个碰撞体和 L 型橱柜的 53 个碰撞体启用，柜门和抽屉保持静态。其他厨房物体仍按背景配置处理。

香蕉按圆面积均匀采样：`theta = uniform(0, 2π)`，`r = radius * sqrt(uniform(0, 1))`。每次 Backspace 恢复盘子固定位置、重新采样香蕉位置并清零物体速度。圆心来自 USD 初始位姿，不随上轮物体移动而改变；修改圆心或半径后需检查台面边缘和水池。

## 五终端运行指令

### 终端 1：厨房仿真与采集

```bash
export WBC_ROOT="/path/to/wbc"
export ISAAC_PYTHON="/path/to/isaac-sim-4.5.0/python.sh"
cd "$WBC_ROOT"
bash run_wbc_kitchen.sh
```

脚本加载厨房、G1 43-DoF、DDS 与 relay，并默认启用自动 HDF5 采集。GUI 第三人称缩放为 `4.0`，机器人初始位置 `(-0.10, -1.90, 0.757)`、yaw `-9°`。

等待厨房、机器人和物体加载完成。无需另开 relay；脚本已启用键盘 `k` 转发。

### 终端 2：WBC 控制器

```bash
export WBC_ROOT="/path/to/wbc"
export TensorRT_ROOT="/path/to/TensorRT"
cd "$WBC_ROOT/gear_sonic_deploy"
source scripts/setup_env.sh
bash deploy.sh --input-type zmq_manager sim
```

按提示确认，等待模型和 TensorRT 引擎就绪。首次运行可能需要编译引擎。本文的 `sim` 配置仅用于仿真。

### 终端 3：PICO 追踪与 manager

```bash
export WBC_ROOT="/path/to/wbc"
export TELEOP_PYTHON="/path/to/wbc-deps/.venv_teleop/bin/python"
cd "$WBC_ROOT"
PYTHONPATH="$WBC_ROOT${PYTHONPATH:+:$PYTHONPATH}" "$TELEOP_PYTHON" \
  gear_sonic/scripts/pico_manager_thread_server.py --manager --port 5563
```

`TELEOP_PYTHON` 指向安装步骤创建的遥操 Python 环境，`PYTHONPATH` 指定 WBC 源码目录。等待出现：

```text
Manager controls: A+X=toggle mode, A+B+X+Y=start/stop policy
[Manager] episode control: 127.0.0.1:5564
```

一直显示 `waiting for body data...` 时，先确认 PICO 已选择 FullBody，再停止并重启终端 3。**Connected 不代表身体数据已到达**；等待期间按 ABXY 不能完成 manager 标定。

### 终端 4：画面回传 PICO

在 PICO Remote Vision 中选择 `PICO4U` 并点击 **Listen**，然后运行。IP 改为 PICO 当前地址：

```bash
export WBC_ROOT="/path/to/wbc"
export ISAAC_PYTHON="/path/to/isaac-sim-4.5.0/python.sh"
export PICO_IP="YOUR_PICO_IP"
cd "$WBC_ROOT"
PYTHONPATH="$WBC_ROOT${PYTHONPATH:+:$PYTHONPATH}" "$ISAAC_PYTHON" \
  gear_sonic/scripts/run_xrobotoolkit_remote_vision.py --headset-host "$PICO_IP"
```

看到 `connected to PICO receiver` 且发送计数增加后，在头显确认画面。桥接断开后可能需要重新 Listen；不要用端口探测消耗单连接的视频入口。

### 终端 5：分步初始化

在终端 2 模型就绪、终端 3 出现 controls 提示之后、**ABXY 标定之前**，依次执行。每一步都观察机器人状态，勿把三步循环发送。

先发送 `k`，启动部署控制：

```bash
export WBC_ROOT="/path/to/wbc"
export TELEOP_PYTHON="/path/to/wbc-deps/.venv_teleop/bin/python"
cd "$WBC_ROOT"
"$TELEOP_PYTHON" gear_sonic/scripts/send_keyboard_cmd.py k
```

状态稳定后，发送一次 `9` 释放初始 ElasticBand 辅助：

```bash
"$TELEOP_PYTHON" gear_sonic/scripts/send_keyboard_cmd.py 9
```

等待落地稳定，再发送 `backspace` 复位：

```bash
"$TELEOP_PYTHON" gear_sonic/scripts/send_keyboard_cmd.py backspace
```

`k` 和 `9` 是切换操作；辅助释放后不要再按 `9`。Backspace 保留辅助开关状态。终端 5 的每条命令执行后退出，无需常驻。

## 开始遥操与下一轮采集

1. 完成上述 `k → 9 → backspace`，等待机器人稳定。
2. 保持中性标定姿态，按一次 **A+B+X+Y**，manager 标定并从 OFF 进入 PLANNER。
3. **完全松开组合键**，再按一次 **A+X** 进入 POSE 跟踪。ego 图像就绪后开始记录本轮。
4. 左右 Trigger 控制对应手的抓握。把香蕉放进红盘子，松手并放稳。
5. 系统确认成功后，要求 manager 切到 PLANNER 并等待确认，保存 HDF5，然后 reset。
6. 看到 `reset complete; press A+X manually for the next episode`，再手动按一次 **A+X** 开始下一轮，无需重复初始化或标定。

| 结束方式 | 文件处理 | 复位后的操作 |
| --- | --- | --- |
| 香蕉入盘并放稳 | 保存 `outcome=success`、`success=True` | 等待手动 A+X |
| 本轮达到 180 秒 | 保存 `outcome=timeout`、`success=False` | 等待手动 A+X |
| 采集中手动 Backspace | 丢弃当前临时文件，不保存本轮 HDF5 | 保持当前模式，图像就绪后自动重开记录并重新计时 |

三分钟从本轮正式记录开始按实际时间累计，途中暂停跟踪不暂停计时。等待下一轮时不计时；这时按 Backspace 只复位，仍需 A+X。自动结束已进入握手时手动 Backspace，会丢弃尚未落盘的当前轮，完成握手后等待 A+X。历史文件不受影响。

成功检测要求香蕉位于盘内、接触盘子、脱离手部且两物体运动足够小，连续满足约 0.6 秒，并有有效 ego 图像。悬空经过、仍被抓住或跨在盘沿外不算成功。

A+X 使用组合键从未按下到按下的上升沿切换模式。自动结束通过 `127.0.0.1:5564` 接口明确请求 PLANNER；等待阶段暂停物理，只有新的手动 A+X 才能开始下一轮。详细判据、状态边界与 HDF5 字段见 [采集判据与文件字段](#采集判据与文件字段)。

### 常用按键与停止

| 按键 | 功能 |
| --- | --- |
| ABXY，当前 OFF | 标定并进入 PLANNER |
| ABXY，已经运行 | 停止策略并退出 manager；下次需重启终端 3 |
| A+X | PLANNER ↔ POSE |
| B+Y | POSE ↔ 冻结上身的 PLANNER 模式 |
| POSE 中按住左菜单键 | 暂停 POSE 跟踪，松开恢复；本轮计时继续 |
| 左 / 右 Trigger > 0.5 | 对应手抓握，释放则张开 |
| Backspace | 复位；采集中丢弃本轮并重新计时 |
| 终端 2 的 `O` / `o` | WBC 键盘急停 |

停止时先停策略，再结束视频桥、manager、WBC 和仿真。进程重启后需重新完成初始化与标定。

## 图像与数据位置

PICO 显示 GUI 的第三人称画面，**GUI 保持 Perspective（`/OmniverseKit_Persp`）**。它复用活动视口，切换 GUI 相机会影响头显画面。HDF5 使用独立 ego 第一人称相机，不占用 PICO 的图像端口。

| 内容 | 位置 / 配置 |
| --- | --- |
| 原始 HDF5 | `work_dirs/kitchen_episodes/episode_*.hdf5` |
| 正在记录或中断的文件 | 同目录 `*.partial.hdf5`，不用于成功示范转换 |
| LeRobot 数据集 | `datasets/<数据集名>/` |
| 厨房和物体资源 | `FluxBisim/assets/` |
| ego 图像 | 640×480 RGB JPEG，独立写入 HDF5 |
| 状态与控制 | 43 维关节状态 / 目标，另存 29 维身体 LowCmd、根节点及物体位姿 |

仿真配置 200 Hz、HDF5 最高采样 30 Hz、PICO 图像发布请求 60 FPS，实际速率以时间戳为准。视频桥默认输出 30 FPS、2160×810；双眼复制同一张图像。

ego 使用原 MJCF 相机姿态，内参 fx=620.80、fy=625.22、cx=320、cy=240，Isaac 以平均焦距近似（垂直 FOV 约 42.14°），裁剪范围 0.02～100 m。

### 采集判据与文件字段

采集由 `--isaac-episode-directory` 启用，厨房启动脚本默认设置该参数；其他入口省略它时不采集。`--isaac-episode-hz` 控制最高采样频率。manager 的 `--episode_control_port` 与仿真的 `--isaac-episode-control-port` 必须相同，默认都是 5564。

<details>
<summary>展开成功判据、HDF5 字段和失败处理</summary>

#### 成功条件

成功检查使用 PhysX 的实际接触力，而不是只比较水平距离。下列条件需连续满足 0.6 秒：

- 香蕉的水平包围盒完全位于盘子中心半径 `0.105 m` 内，根节点高于盘子且高度差小于 `0.09 m`。
- 盘子正面朝上；香蕉与盘子的接触力大于 `0.03 N`。
- 香蕉与机器人手掌、手指、腕部的接触力总量小于 `0.02 N`。
- 香蕉、盘子线速度均小于 `0.05 m/s`，香蕉角速度小于 `0.5 rad/s`。
- 采样时有有效的新第一人称图像。

香蕉在盘子上方悬空、从上方经过、仍被抓住、盘子翻转或物体仍在明显运动时，不计为成功。
包围盒条件比较保守；香蕉跨在盘沿外面时需要放得更居中。

#### HDF5 文件

默认目录：`/path/to/wbc/work_dirs/kitchen_episodes/`。
每轮一个 `episode_<UTC时间>_<随机ID>.hdf5`，文件名不会覆盖已有轨迹。
记录过程中使用 `.partial.hdf5`，成功写入、flush/fsync 后才改为最终文件名。
中断时保留 partial 文件并标记 `complete=False`，不伪装成成功轨迹。

默认最高采样频率为 30 Hz；200 Hz 仿真步长下实际采样间隔由时间戳给出，不能假定严格等间隔 1/30 秒。
每个数据集第一维都是相同的采样帧数。状态是物理步之后的观测，动作是该物理步之前实际应用的最近指令。

| 数据集 | 内容 |
| --- | --- |
| `time/simulation`, `time/wall` | 仿真时间、PC Unix 时间（秒） |
| `observations/joint_position`, `joint_velocity` | 43 个关节，包含双手；顺序见 metadata 的 `joint_names` |
| `observations/root_pose`, `root_velocity` | 根节点 XYZ + wxyz 四元数，线速度 + 角速度 |
| `actions/joint_position_target` | 实际发送给 Isaac 的 43 关节位置目标（position 控制模式） |
| `actions/applied_effort` | effort 控制模式下应用的力矩；位置控制时为 NaN |
| `actions/lowcmd_q`, `lowcmd_dq`, `lowcmd_kp`, `lowcmd_kd`, `lowcmd_tau` | 29 维身体 DDS LowCmd，顺序见 `body_command_joint_names` |
| `actions/valid`, `received_wall_time` | 指令是否在本轮开始后收到，以及接收时间；无有效指令时不要用于监督训练 |
| `objects/banana_pose`, `plate_pose` | 世界坐标 XYZ + wxyz |
| `objects/banana_velocity`, `plate_velocity` | 世界坐标线速度 + 角速度 |
| `observations/images/ego_jpeg` | 每帧 JPEG 字节，解码为 640×480 RGB |
| `observations/images/ego_valid`, `ego_render_frame`, `ego_render_time` | 图像有效性、渲染帧编号、渲染时间；重复/未就绪图像会显式标记无效 |
| `task/plate_contact_force`, `hand_contact_force` | 香蕉与盘子、手部的接触力（N） |
| `manager/mode` | OFF=0、POSE=1、PLANNER=2、FROZEN=3、POSE_PAUSE=4、VR_3PT=5 |

文件属性包括 `schema_version`、`complete`、`success`、`outcome`、`num_frames` 和 `metadata_json`。
metadata 包含机器人型号、关节名、控制模式、场景 USD 文本及哈希、相机配置和采样参数。
训练数据应筛选 `complete=True`、`success=True`，并检查逐帧的动作和图像有效标记。

相机是独立的 `/World/G1/torso_link/episode_ego_camera`（实际父节点来自原 MJCF 配置），
使用现有 ego 的位置、朝向、内参和裁剪面；光学配置见 README。
它有自己的 render product，不绑定 GUI 当前视口，不占用 `5555`，也不改变 PICO 第三人称视频。

```python
import cv2
import h5py
import json

with h5py.File("episode_....hdf5", "r") as data:
    metadata = json.loads(data.attrs["metadata_json"])
    q = data["observations/joint_position"][:]
    encoded = data["observations/images/ego_jpeg"][0]
    rgb = cv2.cvtColor(cv2.imdecode(encoded, cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)
```

#### 边界和失败处理

- 成功轨迹截止到成功判定帧；退出跟踪、文件保存、复位和等待下一轮的过程不混入下一条轨迹。
- 采集中手动 Backspace 在下一循环边界复位并删除本轮临时文件，不保存 `manual_reset` 轨迹；
  相机就绪后自动重开本轮、重新计时。不会自动切换当前跟踪模式。
- 身体和双手的 DDS 缓存在复位、手动开始下一轮时都清空；早于边界进入回调的指令被丢弃。
  双手在收到新的手部指令之前保持当前关节位置。
- WBC 使用原有指令协议；不发送/等待 reset 编号，也不重新初始化 WBC 的历史、上一动作或 heading。
  保留仿真侧采集边界缓存清理，reset 后正常接收原版身体和双手指令。
- manager 确认缺失时不执行自动复位。录制中的连接长时间失效时暂停物理；文件写入失败或 manager
  在本轮中重启时保持暂停并报错，保留已有文件，排除问题后再重启采集。
- 该接口确认的是 manager 已停止 POSE 并发布 PLANNER；部署程序和真实 PICO 的完整链路仍需现场确认。
  等待阶段由仿真暂停和指令缓存清理共同防止旧指令继续推动机器人。

- WBC 使用墙钟控制循环，Isaac 使用 `World.step(render=...)` 步进方式。
  三分钟限制仍使用单调墙钟；采集保存/等待阶段仍由 collector 暂停仿真。


</details>

## HDF5 转 LeRobot

先安装 [uv](https://docs.astral.sh/uv/getting-started/installation/)，在 WBC 根目录创建独立的 Python 3.10 转换环境：

```bash
cd /path/to/wbc
uv venv work_dirs/wbc_lerobot_env --python 3.10
uv pip install --python work_dirs/wbc_lerobot_env/bin/python \
  --index-strategy unsafe-best-match -r tools/requirements-wbc-convert.txt
```

环境准备完成后，检查成功轨迹：

```bash
cd /path/to/wbc
work_dirs/wbc_lerobot_env/bin/python tools/convert_wbc_hdf5_to_lerobot.py \
  work_dirs/kitchen_episodes --dry-run
```

转换为 LeRobot v2.1；输出目录必须尚不存在：

```bash
work_dirs/wbc_lerobot_env/bin/python tools/convert_wbc_hdf5_to_lerobot.py \
  work_dirs/kitchen_episodes \
  --output datasets/wbc_banana_success \
  --repo-id local/wbc_banana_success \
  --fps 30
```

默认仅选择完整成功轮次。测试超时数据时显式加 `--outcomes timeout` 并使用新输出目录；超时不能当作成功示范。转换保留 43 维 state/action，图像字段为 `observation.images.ego`，不修改原 HDF5，也不上传数据。

输出 30 FPS 使用已有观测重采样，不会增加真实采样信息。字段、动作对齐、筛选及环境安装见 [转换字段与时间对齐](#转换字段与时间对齐)。

### 转换字段与时间对齐

转换器支持 schema_version=1、position 控制模式、43 关节和一台 ego 相机。默认仅接收完整的 `outcome=success`、`success=True` 文件；`--outcomes timeout` 用于超时测试数据，`--limit 1` 可只转换一条合格源文件。旧 `manual_reset`、`test_timeout` 和 partial 文件不纳入转换。

输出目录必须不存在，暂不支持追加。没有合格轨迹时不会创建空数据集。需要训练 43 维 state/action 和 `observation.images.ego`，不能直接套用 ALOHA 的 14 维输入配置。转换依赖固定 LeRobot commit `55198de096f46a8e0447a8795129dd9ee84c088c`。

<details>
<summary>展开字段映射、重采样和动作对齐规则</summary>

#### 字段与时间对应

| LeRobot 字段                                              | HDF5 字段/含义                          |
| --------------------------------------------------------- | --------------------------------------- |
| `observation.state`                                       | `observations/joint_position`，43 维    |
| `action`                                                  | `actions/joint_position_target`，43 维  |
| `observation.images.ego`                                  | `observations/images/ego_jpeg`，RGB MP4 |
| `observation.joint_velocity`                              | 43 维关节速度                           |
| `observation.root_pose`, `root_velocity`                  | 根节点位姿、速度                        |
| `observation.banana_pose`, `plate_pose`                   | 香蕉、盘子 XYZ + wxyz                   |
| `observation.banana_velocity`, `plate_velocity`           | 物体线速度、角速度                      |
| `source.observation_row`, `source.action_row`             | 源 HDF5 行号                            |
| `source.observation_wall_time`, `source.action_wall_time` | 源行 PC 时间                            |
| `source.simulation_time`, `source.ego_render_time`        | 保留的原仿真和相机时间                  |
| `source.action_received_wall_time`                        | 动作的原接收时间                        |

关节名称从 metadata 读取并保存到 features；所有源文件必须使用同一顺序，
不把 G1 裁成 ALOHA 的 14 维。原始 lowcmd、effort、接触力等仍保留在 HDF5，
不作为本次 LeRobot 模型输入。文件哈希和完整源 metadata 写入转换报告。

重采样使用 `time/wall`，每个连续有效片段从零开始，以 `1/fps` 为间隔。
每个目标时刻选取最近的、不晚于该时刻的源观测行；图像、状态和物体位姿
共同保持该行，避免仅给视频改帧率导致快放。不对姿态四元数做线性插值。
**30 FPS 不会把原来的 3–4 Hz 观测变成真实 30 Hz 数据，只会重复已有样本。**

动作对齐方式：

- 默认 `--action-alignment recorded`：状态和动作来自同一源行，忠实保留记录；
  该动作是观测前已施加的关节目标，不声称它是观测后的下一条指令。
- `--action-alignment next`：动作取下一条有效的原始采样行，不是下一张重复的视频帧；
  片段末尾没有后续动作的部分会截掉。这只是稀疏样本的一步偏移，不能恢复丢失的
  中间控制指令，也不保证等同“立即下一帧动作”。训练前需明确所用监督定义。

无效动作、NaN、无效/损坏图像、非 POSE 模式会剔除，并在相邻有效片段之间断开，
不把无效时间段两端拼在一起。源采样间隔超过 `--max-gap 0.5` 秒也会分段。
若正常采集更稀疏，可显式调整该阈值；不建议用大阈值掩盖中断。

源 PC 时间必须严格递增，否则该文件被排除并列出原因。采用墙钟是因为旧采集文件的
`time/simulation` 在渲染推进多步时可能少计；墙钟是记录行时间，不是精确曝光时间。
相机与动作的亚帧延迟不能靠现有稀疏 HDF5 消除。

#### 输出与验证

输出包含标准 `data/` Parquet、`videos/` MP4 和 `meta/` 元数据。
图像统计按解码后的 RGB 值计算；转换报告保存源文件与对齐信息。

额外的 `meta/wbc_conversion.json` 记录源文件 SHA256、原成功标记、分段、
筛选原因、采样频率及动作对齐方式。输出帧行号可以追溯至原 HDF5。

转换在新的临时目录中执行。完成并通过官方 LeRobot reader 的每段首尾视频解码后，
才改名为最终输出目录；失败会保留以 `.partial-` 命名的转换目录供排查。
脚本不上传到 Hugging Face，也不覆盖已有输出。

格式可被 LeRobot 读取不代表能直接使用 ALOHA 模型：训练需配置 43 维 state/action、
`observation.images.ego`，并显式选择是否使用速度和物体位姿等额外字段。


</details>

## 文件与接口

| 入口 | 用途 |
| --- | --- |
| [run_wbc_kitchen.sh](run_wbc_kitchen.sh) | 厨房启动参数与采集目录 |
| [kitchen.usda](gear_sonic/data/scenes/fluxbisim/kitchen.usda) | 场景引用、碰撞、盘子 / 香蕉生成配置 |
| [isaac_server.py](gear_sonic/simulation_server/isaac_server.py) | 物理步进、随机复位和相机 |
| [kitchen_episodes.py](gear_sonic/simulation_server/kitchen_episodes.py) | 成功判定、180 秒计时、HDF5 保存及复位 |
| [episode_control.py](gear_sonic/utils/teleop/episode_control.py) | 仿真与 manager 的分轮请求 / 确认 |
| [pico_manager_thread_server.py](gear_sonic/scripts/pico_manager_thread_server.py) | PICO 数据、模式与按键 |
| [gear_sonic_deploy/](gear_sonic_deploy/) | 原版 WBC 推理与模型 |
| [convert_wbc_hdf5_to_lerobot.py](tools/convert_wbc_hdf5_to_lerobot.py) | 离线转换 |

端口：manager `5563` → relay `5556` → WBC；WBC 状态反馈 `5557`；键盘入口 `5580`、部署键盘转发 `5562`；图像 `5555` → PICO TCP `12345`；分轮控制仅监听 `127.0.0.1:5564`。

`FluxBisim/`、`work_dirs/`、`datasets/` 由 Git 忽略，不随 WBC 仓库分发。迁移时需单独准备资源和数据，并重建 Python 环境；不能只复制一个 USD 或整个虚拟环境。

## 安装与模型准备

首次运行需安装 Isaac、TensorRT、XRoboToolkit 和通信依赖，并下载模型；厨房资产见 [厨房资产下载](#下载厨房资产)，离线转换环境见 [HDF5 转 LeRobot](#hdf5-转-lerobot)。下面保留安装和固定模型版本的步骤，尚未在空白机器上完成端到端验证。

<details>
<summary>展开安装、依赖和模型校验步骤</summary>

### 资源与外部依赖

#### 仓库内资源

| 内容 | 仓库位置 | 用途 |
| --- | --- | --- |
| WBC encoder | `gear_sonic_deploy/policy/release/model_encoder.onnx` | 编码运动 / 遥操条件，约 50.1 MB，已纳入 Git |
| WBC decoder | `gear_sonic_deploy/policy/release/model_decoder.onnx` | 根据观测与编码条件产生控制输出，约 40.9 MB，已纳入 Git |
| 配套观测配置 | `gear_sonic_deploy/policy/release/observation_config.yaml` | 定义模型输入、历史帧和 encoder 模式 |
| G1 USD、URDF、MJCF 与网格 | `gear_sonic/data/robots/`、`gear_sonic/data/robot_model/` | Isaac 加载、关节映射、相机参数与 FK 标定 |
| 场景与道具 | `gear_sonic/data/scenes/`、`gear_sonic/data/assets/` | 当前仿真环境及被引用资源 |
| 人体骨架辅助资源 | `gear_sonic/data/human/human_joints_info.pkl`、`gear_sonic/trl/utils/smplx/` | 人体姿态处理；不是需要另下载的完整训练数据集 |
| 原生 Unitree C++ SDK | `gear_sonic_deploy/thirdparty/unitree_sdk2/` | C++ 推理端 DDS 通信；不代替 Python SDK |

厨房的外部资源还需按 [厨房资产下载](#下载厨房资产) 下载到本仓库 `FluxBisim/assets/`；它不包含在 Git 中。

不要只复制某一个 `.usd`：它可能依赖同目录或相邻目录的网格、材质、配置和其他 USD。保留这些资源目录的完整结构。

#### 外部依赖

| 外部内容 | 建议版本 / 基线 | 下载或获取位置 | 安装 / 放置位置 |
| --- | --- | --- | --- |
| planner ONNX | 与本 README 固定的官方模型快照配套 | [Hugging Face：nvidia/GEAR-SONIC](https://huggingface.co/nvidia/GEAR-SONIC) | `gear_sonic_deploy/planner/target_vel/V2/planner_sonic.onnx` |
| Isaac Sim | 4.5.0 | [NVIDIA Isaac Sim 4.5 下载页](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/download.html) | 仓库外；使用其自带 `python.sh` |
| NVIDIA 驱动、CUDA | 参考版本 CUDA Toolkit 12.4；驱动需符合 Isaac 要求 | [CUDA Toolkit Archive](https://developer.nvidia.com/cuda-toolkit-archive) | 系统安装 |
| TensorRT C++ 头文件与库 | 参考版本 10.13.0 | [NVIDIA TensorRT 下载](https://developer.nvidia.com/tensorrt/download/10x) | 仓库外；设置 `TensorRT_ROOT` |
| ONNX Runtime C++ | 安装脚本默认 1.16.3 | [官方 v1.16.3](https://github.com/microsoft/onnxruntime/releases/tag/v1.16.3) | 通常 `/opt/onnxruntime`，可由仓库安装脚本准备 |
| XRoboToolkit PC Service | v1.0.0，Ubuntu 22.04 amd64 安装包 | [官方 Release](https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/tag/v1.0.0) | 安装后应有 `/opt/apps/roboticsservice/runService.sh` |
| PICO XRoboToolkit APK | v1.1.1 | [官方 Unity Client Release](https://github.com/XR-Robotics/XRoboToolkit-Unity-Client/releases/tag/v1.1.1) | 安装到 PICO，不是安装到 Python |
| XRoboToolkit Python SDK | 参考版本 1.0.2 | [PC-Service-Pybind](https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind) | 安装到终端 3 的 Python 环境 |
| Unitree Python SDK | 参考版本 1.0.1 | [unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python) | 安装到 Isaac 自带 Python 环境 |
| GStreamer 与 x264 插件 | Ubuntu 软件包 | `apt`，见下文 | 系统安装，终端 4 使用 |

**ONNX 并非一律不能放 GitHub。** 普通 GitHub Git 的单文件限制为 100 MiB；本仓库 encoder / decoder 小于该限制，已经提交。缺少的 planner 为 **773,952,989 字节，约 774 MB / 738 MiB**，应从模型站单独下载，而不是普通 `git add`。

`.trt` 引擎缓存在目标机器首次运行时生成，与 GPU / TensorRT 版本有关，不是要下载的通用模型，也不应作为跨电脑迁移的依赖。

### 安装步骤

已有正常工作的环境无需重装；缺少模型时按下面的固定版本下载步骤补齐。

#### 基础环境与代码

参考配置：Ubuntu 22.04 x86_64、NVIDIA RTX 3090、Isaac Sim 4.5.0、Python 3.10；PICO 4 Ultra、双手柄和腰部 / 双脚踝追踪器。其他 GPU、系统和 SDK 组合需自行验证，不把这份基线理解为最低硬件要求。

在准备存放项目的目录执行（如果已经克隆，不重复执行）：

```bash
git clone https://github.com/Izimmi-Erwin/wbc.git
cd wbc
export WBC_ROOT="$PWD"
```

这里的根目录直接包含 `gear_sonic/` 和 `gear_sonic_deploy/`，**不要再多进入一层 `wbc/`**。

为安装阶段设置路径，按实际安装位置修改以下值；外部依赖放在仓库之外：

```bash
export WBC_ROOT="/path/to/wbc"
export WBC_DEPS="/path/to/wbc-deps"
export ISAAC_PYTHON="/path/to/isaac-sim-4.5.0/python.sh"
export TELEOP_PYTHON="$WBC_DEPS/.venv_teleop/bin/python"
export TensorRT_ROOT="/path/to/TensorRT"
mkdir -p "$WBC_DEPS"
```

`/path/to/...` 是占位符，不能原样使用。终端之间不共享刚设置的变量；上方五终端运行指令分别包含所需变量。

#### 系统依赖、视频工具与 C++ 构建

先安装 Isaac Sim、匹配驱动 / CUDA、TensorRT。TensorRT 需包含 `include/NvInfer.h` 和 `libnvinfer.so` 等原生开发文件；仅 `pip install tensorrt` 不等于完整配置了本仓库的 C++ 构建。

视频与 Python 扩展构建所需常用包：

```bash
sudo apt update
sudo apt install -y build-essential cmake git python3.10-venv python3.10-dev \
  nlohmann-json3-dev \
  gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly
gst-inspect-1.0 x264enc
gst-inspect-1.0 rawvideoparse
```

部署端提供 [install_deps.sh](gear_sonic_deploy/scripts/install_deps.sh)。它会安装系统依赖、构建工具及 ONNX Runtime 等内容，涉及 `sudo` 和系统目录；**先阅读脚本，再决定在新机器上执行**，不要把它当成只检查环境的命令。

```bash
cd "$WBC_ROOT/gear_sonic_deploy"
bash scripts/install_deps.sh
source scripts/setup_env.sh
just build
```

构建产物应为 `gear_sonic_deploy/target/release/g1_deploy_onnx_ref`。`TensorRT_ROOT` 应指向包含 `include/`、`lib/` 的目录；Debian 提取式安装可能是包含 `lib/x86_64-linux-gnu/` 的 `usr/` 目录。

现有代码兼容部分 TensorRT 8 / 10 API，不代表官方 planner 在任意 TensorRT 8 环境都能构建。优先复用已使用的 TensorRT 10.13 基线，不要混用系统 TRT 8 头文件和本地 TRT 10 库。基础五终端流程不要求安装 ROS 2。

#### 终端 3：独立的遥操 Python 环境

在新路径创建环境，不复制另一台电脑的 `.venv`：

```bash
python3.10 -m venv "$WBC_DEPS/.venv_teleop"
"$TELEOP_PYTHON" -m pip install --upgrade pip setuptools wheel
"$TELEOP_PYTHON" -m pip install -e "$WBC_ROOT/gear_sonic[teleop]" \
  "pin==2.7.0" "eigenpy==3.5.1" "cmeel-urdfdom==3.1.1.1"
"$TELEOP_PYTHON" -m pip install pybind11 huggingface_hub
```

项目基础依赖固定了 NumPy 1.26.4、SciPy 1.15.3；上面给出配套的 Pinocchio 依赖版本，避免随意安装最新 `pin` 导致 NumPy ABI 冲突。manager 默认使用 CPU，不要求给它额外配置 CUDA 推理。

安装 PC Service Release 中适合 Ubuntu 22.04 amd64 的 `.deb`，并在 PICO 上安装 XRoboToolkit APK。不要用浏览器页面替代 APK。

```bash
# 将路径替换成实际下载的安装包
sudo apt install /path/to/XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb
test -f /opt/apps/roboticsservice/runService.sh
```

随后安装 Python 绑定。官方 [构建说明](https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind#building-the-project) 给出了原生 SDK 的编译和复制方法；需要 `PXREARobotSDK.h`、`libPXREARobotSDK.so`、nlohmann JSON 头文件与 pybind11。

对这里的 Ubuntu x86_64 `.deb` 安装，可复用已安装 PC Service 的 SDK 文件，避免误连另一个版本的原生库。以下用于新克隆的绑定目录：

```bash
export XRT_BINDINGS_DIR="$WBC_DEPS/XRoboToolkit-PC-Service-Pybind"
git clone https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind.git "$XRT_BINDINGS_DIR"
mkdir -p "$XRT_BINDINGS_DIR/include" "$XRT_BINDINGS_DIR/lib"
cp /opt/apps/roboticsservice/SDK/include/PXREARobotSDK.h "$XRT_BINDINGS_DIR/include/"
cp /opt/apps/roboticsservice/SDK/x64/libPXREARobotSDK.so "$XRT_BINDINGS_DIR/lib/"
cp -r /usr/include/nlohmann "$XRT_BINDINGS_DIR/include/"
export CMAKE_PREFIX_PATH="$("$TELEOP_PYTHON" -m pybind11 --cmakedir)${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
"$TELEOP_PYTHON" -m pip install --no-build-isolation -e "$XRT_BINDINGS_DIR"
"$TELEOP_PYTHON" -c 'import xrobotoolkit_sdk, pinocchio, numpy, scipy, torch, zmq; print("teleop imports OK")'
```

如果安装包没有上述 SDK 路径，或后续绑定版本改变了 API / 编译结构，停止套用复制命令，按官方说明编译对应版本。当前绑定源码没有在本仓库锁定 commit；迁移成功后应记录实际使用的 SDK commit，不能仅凭相同 Python 包版本号判断源码完全相同。

#### 终端 1 / 4：Isaac 自带 Python

这是另一个环境，终端 3 中安装成功不代表 Isaac 可以导入相同包。保留 Isaac 自带的 NumPy、SciPy、Torch，**不要在里面直接升级整套训练 / teleop 依赖**。

先检查，再仅补缺失的通信和视频依赖；以下版本适用于 NumPy 1.26 环境：

```bash
"$ISAAC_PYTHON" -m pip install "numpy==1.26.4" "opencv-python==4.10.0.84" \
  pyzmq msgpack msgpack-numpy PyYAML tyro h5py "cyclonedds==0.10.2"
export UNITREE_SDK_DIR="$WBC_DEPS/unitree_sdk2_python"
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git "$UNITREE_SDK_DIR"
"$ISAAC_PYTHON" -m pip install --no-deps -e "$UNITREE_SDK_DIR"
"$ISAAC_PYTHON" -c 'import numpy, yaml, zmq, msgpack, msgpack_numpy, cv2, cyclonedds, unitree_sdk2py, tyro, h5py; print("Isaac bridge imports OK")'
```

`--no-deps` 的前提是前一条命令已满足 SDK 1.0.1 的依赖；若未来 SDK requirements 改变，先核对其 `setup.py`。如果 CycloneDDS 报找不到原生库，按 [Unitree SDK 安装说明](https://github.com/unitreerobotics/unitree_sdk2_python#installation) 配置其要求的 CycloneDDS，再在这个 Isaac Python 中安装，不要装到另一个系统 Python 后就认为问题解决。

### 下载并校验 ONNX 模型

官方来源：[nvidia/GEAR-SONIC](https://huggingface.co/nvidia/GEAR-SONIC)。本文核对的模型快照为：

```text
6733128a3d8a523b1418b06bca3cdf61c8b0987f
```

| 文件 | SHA-256 |
| --- | --- |
| `model_encoder.onnx` | `013ab0287236aa2721e13f1e936d699db982302d0de0bfcdae76d5c3245362d3` |
| `model_decoder.onnx` | `c7241a123eaa36b5d64bad19540efde93cac1ad443bd4572fd12ca99898118ed` |
| `observation_config.yaml` | `466d05947c78af6c76388adfb86e3a2a77b2a1d921a64883ed3d085ebf58de1b` |
| `planner_sonic.onnx` | `39b553e197f62f077975ba38512bc04781a3fc37c2af7c6756e04629f760edea` |

仓库现有 encoder、decoder、配置与这个快照一致。推荐使用下面的补齐命令：**固定版本、检查校验值、只补缺失文件，不覆盖已有文件**。如现有文件校验不符，会停止并要求人工核对，而不是自动替换。

在已设置 `WBC_ROOT`、`TELEOP_PYTHON` 的安装终端执行；下载缓存和目标文件可能各占一份空间，建议预留至少 2 GB：

```bash
"$TELEOP_PYTHON" - <<'PY'
import hashlib
import os
import shutil
from pathlib import Path
from huggingface_hub import hf_hub_download

root = Path(os.environ["WBC_ROOT"]).resolve()
assert (root / "gear_sonic_deploy").is_dir(), "WBC_ROOT 不是本仓库根目录"
revision = "6733128a3d8a523b1418b06bca3cdf61c8b0987f"
files = {
    "model_encoder.onnx": ("policy/release/model_encoder.onnx", "013ab0287236aa2721e13f1e936d699db982302d0de0bfcdae76d5c3245362d3"),
    "model_decoder.onnx": ("policy/release/model_decoder.onnx", "c7241a123eaa36b5d64bad19540efde93cac1ad443bd4572fd12ca99898118ed"),
    "observation_config.yaml": ("policy/release/observation_config.yaml", "466d05947c78af6c76388adfb86e3a2a77b2a1d921a64883ed3d085ebf58de1b"),
    "planner_sonic.onnx": ("planner/target_vel/V2/planner_sonic.onnx", "39b553e197f62f077975ba38512bc04781a3fc37c2af7c6756e04629f760edea"),
}

def digest(path):
    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(8 * 1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()

for name, (relative, expected) in files.items():
    target = root / "gear_sonic_deploy" / relative
    source = target if target.exists() else Path(hf_hub_download(
        repo_id="nvidia/GEAR-SONIC", filename=name, revision=revision))
    if digest(source) != expected:
        raise RuntimeError(f"校验失败，请保留并核对文件：{source}")
    if not target.exists():
        target.parent.mkdir(parents=True, exist_ok=True)
        with target.open("xb") as out, source.open("rb") as src:
            shutil.copyfileobj(src, out)
    print(f"OK: {target}")
PY
```

如下载中断导致目标文件不完整，下次运行会报告校验失败；先保留 / 移走该不完整文件，再重试。不要绕过校验。

仓库也保留了快捷工具 [download_from_hf.py](download_from_hf.py)：

```bash
cd "$WBC_ROOT"
"$TELEOP_PYTHON" download_from_hf.py
```

**二选一即可**。快捷工具默认下载当时远端版本的四个部署文件，并覆盖目标位置；它不固定上述 revision。已有定制模型时不要直接运行。也不要把 HF 根目录随意下载到 `gear_sonic_deploy/` 就认为路径正确，部署脚本要求上述 `policy/release/` 与 `planner/target_vel/V2/` 结构。

注意：

- encoder、decoder 和观测 YAML 应作为配套版本使用。当前 YAML 使用 `*_10frame_*` 观测；C++ 支持 4-frame 字段不等于当前加载的是 4-frame 模型。
- `deploy.sh` 对 `zmq_manager` 允许缺 planner 的降级启动，但 manager 初始启动进入 `PLANNER`。要复现本文完整流程，仍应下载 planner；“进程启动了”不代表规划功能可用。
- 只做本流程，不需要训练权重 `sonic_release/last.pt`、完整 SMPL 训练数据或几十 GB 的动作数据集，也不需要另下载完整 FluxVLA/VLA 权重。


</details>

## 验证与来源

在仓库根目录运行采集与转换测试：

```bash
cd /path/to/wbc
export ISAAC_PYTHON="/path/to/isaac-sim-4.5.0/python.sh"
PYTHONPATH="$PWD" "$ISAAC_PYTHON" -m unittest discover \
  -s gear_sonic/tests -p test_kitchen_episodes.py -v
work_dirs/wbc_lerobot_env/bin/python -m unittest discover \
  -s test/test_tools -p test_wbc_conversion.py -v
```

测试覆盖采集状态机、文件写入与格式转换；实际 PICO 闭环遥操需在部署环境中验证。

代码基于 GEAR-SONIC / GR00T-WholeBodyControl、SonicStar 与 FluxVLA 的相关实现。FluxBisim 代码采用 Apache-2.0，厨房资产标注 CC-BY-NC-4.0。算法、训练及其他上游用法见 [GR00T-WholeBodyControl 文档](https://nvlabs.github.io/GR00T-WholeBodyControl/)。厨房操作、采集和转换说明统一维护在本文。

上游 WBC 源码采用 Apache-2.0，模型权重采用 NVIDIA Open Model License；完整条款见 [上游 LICENSE](https://github.com/NVlabs/GR00T-WholeBodyControl/blob/main/LICENSE)。分发模型需保留归属说明，并遵守 [NVIDIA Trustworthy AI Terms](https://www.nvidia.com/en-us/agreements/trustworthy-ai/terms/)。第三方代码及资产分别遵循其许可证。

<details>
<summary>GEAR-SONIC 论文引用</summary>

```bibtex
@article{luo2025sonic,
    title={SONIC: Supersizing Motion Tracking for Natural Humanoid Whole-Body Control},
    author={Luo, Zhengyi and Yuan, Ye and Wang, Tingwu and Li, Chenran and Chen, Sirui and Casta\~neda, Fernando and Cao, Zi-Ang and Li, Jiefeng and Minor, David and Ben, Qingwei and Da, Xingye and Ding, Runyu and Hogg, Cyrus and Song, Lina and Lim, Edy and Jeong, Eugene and He, Tairan and Xue, Haoru and Xiao, Wenli and Wang, Zi and Yuen, Simon and Kautz, Jan and Chang, Yan and Iqbal, Umar and Fan, Linxi and Zhu, Yuke},
    journal={arXiv preprint arXiv:2511.07820},
    year={2025}
}
```

</details>
