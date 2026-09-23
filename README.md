# Isaac WBC：PICO 遥操作与厨房数据采集

当前版本：2026-09-23。使用 Isaac Sim 4.5.0、GEAR-SONIC WBC 和 PICO / XRoboToolkit 遥操作 G1（29 个身体关节 + 14 个手部关节），在 FluxBisim 厨房中采集“把香蕉放进红盘子”的轨迹。

红盘子固定生成在中岛，香蕉在水池右侧台面的 3 cm 圆内随机生成，机器人初始化在香蕉右侧。中岛和 L 型橱柜已启用碰撞。厨房接入保留原 WBC 控制器和五终端流程。

## 运行前准备

本文命令对应本机独立 WBC 目录：

```text
/home/colin/Erwin/isaac-wbc-publish-20260910.s2j08O/wbc
```

所有终端都从这个目录加载代码。迁移机器时替换仓库、Isaac Python、遥操 Python 和 TensorRT 路径；安装说明见文末。

1. 确认 XRoboToolkit PC Service 运行。需要手动启动时执行 `bash /opt/apps/roboticsservice/runService.sh`；manager 也会尝试启动服务。
2. 戴好追踪器和手柄，完成身体追踪标定。在 PICO 原生 XRoboToolkit 中选择 **FullBody**，勾选 **Head、Controller、Send**，连接 PC 的局域网 IP。
3. 核对 PICO 自己的 IP，供终端 4 使用。PC 与 PICO 需能通过局域网互通。
4. 如果旧副本仍在运行，先停止原遥操再切换；同一台电脑不要同时启动两套仿真或 manager。

## 五终端运行指令

### 终端 1：厨房仿真与采集

```bash
export WBC_ROOT="/home/colin/Erwin/isaac-wbc-publish-20260910.s2j08O/wbc"
export ISAAC_PYTHON="/home/colin/Erwin/isaac-sim-4.5.0/python.sh"
cd "$WBC_ROOT"
bash run_wbc_kitchen.sh
```

脚本加载厨房、G1 43-DoF、DDS 与 relay，并默认启用自动 HDF5 采集。GUI 第三人称缩放为 `4.0`，机器人初始位置 `(-0.10, -1.90, 0.757)`、yaw `-9°`。

等待厨房、机器人和物体加载完成。无需另开 relay；脚本已启用键盘 `k` 转发。

### 终端 2：WBC 控制器

```bash
export WBC_ROOT="/home/colin/Erwin/isaac-wbc-publish-20260910.s2j08O/wbc"
export TensorRT_ROOT="/home/colin/opt/tensorrt-10.13.0/usr"
cd "$WBC_ROOT/gear_sonic_deploy"
source scripts/setup_env.sh
bash deploy.sh --input-type zmq_manager sim
```

按提示确认，等待模型和 TensorRT 引擎就绪。首次运行可能需要编译引擎。本文的 `sim` 配置仅用于仿真。

### 终端 3：PICO 追踪与 manager

```bash
export WBC_ROOT="/home/colin/Erwin/isaac-wbc-publish-20260910.s2j08O/wbc"
export TELEOP_PYTHON="/home/colin/Erwin/GR00T-WholeBodyControl/.venv_teleop/bin/python"
cd "$WBC_ROOT"
PYTHONPATH="$WBC_ROOT${PYTHONPATH:+:$PYTHONPATH}" "$TELEOP_PYTHON" \
  gear_sonic/scripts/pico_manager_thread_server.py --manager --port 5563
```

本机复用已有遥操 Python 环境，通过 `PYTHONPATH` 加载本仓库代码。等待出现：

```text
Manager controls: A+X=toggle mode, A+B+X+Y=start/stop policy
[Manager] episode control: 127.0.0.1:5564
```

一直显示 `waiting for body data...` 时，先确认 PICO 已选择 FullBody，再停止并重启终端 3。**Connected 不代表身体数据已到达**；等待期间按 ABXY 不能完成 manager 标定。

### 终端 4：画面回传 PICO

在 PICO Remote Vision 中选择 `PICO4U` 并点击 **Listen**，然后运行。IP 改为 PICO 当前地址：

```bash
export WBC_ROOT="/home/colin/Erwin/isaac-wbc-publish-20260910.s2j08O/wbc"
export ISAAC_PYTHON="/home/colin/Erwin/isaac-sim-4.5.0/python.sh"
export PICO_IP="192.168.180.214"
cd "$WBC_ROOT"
PYTHONPATH="$WBC_ROOT${PYTHONPATH:+:$PYTHONPATH}" "$ISAAC_PYTHON" \
  gear_sonic/scripts/run_xrobotoolkit_remote_vision.py --headset-host "$PICO_IP"
```

看到 `connected to PICO receiver` 且发送计数增加后，在头显确认画面。桥接断开后可能需要重新 Listen；不要用端口探测消耗单连接的视频入口。

### 终端 5：分步初始化

在终端 2 模型就绪、终端 3 出现 controls 提示之后、**ABXY 标定之前**，依次执行。每一步都观察机器人状态，勿把三步循环发送。

先发送 `k`，启动部署控制：

```bash
export WBC_ROOT="/home/colin/Erwin/isaac-wbc-publish-20260910.s2j08O/wbc"
export TELEOP_PYTHON="/home/colin/Erwin/GR00T-WholeBodyControl/.venv_teleop/bin/python"
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

A+X 使用组合键从未按下到按下的上升沿切换模式。自动结束通过本机 `5564` 接口明确请求 PLANNER；等待阶段暂停物理，只有新的手动 A+X 才能开始下一轮。详细判据、状态边界与 HDF5 字段见 [自动采集说明](docs/wbc_kitchen_collection.md)。

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

## HDF5 转 LeRobot

在本机已经创建好的独立转换环境中，先检查成功轨迹：

```bash
cd /home/colin/Erwin/isaac-wbc-publish-20260910.s2j08O/wbc
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

输出 30 FPS 使用已有观测重采样，不会增加真实采样信息。字段、动作对齐、筛选及环境安装见 [LeRobot 转换说明](docs/wbc_lerobot.md)。

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

端口：manager `5563` → relay `5556` → WBC；WBC 状态反馈 `5557`；键盘入口 `5580`、部署键盘转发 `5562`；图像 `5555` → PICO TCP `12345`；分轮控制仅监听本机 `5564`。

本机的 `FluxBisim/`、`work_dirs/`、`datasets/` 已由 Git 忽略。迁移时需单独准备资源和数据，并重建 Python 环境；不能只复制一个 USD 或整个虚拟环境。

## 新电脑安装与模型准备

现有电脑直接使用上面的五终端命令。新电脑还需准备 Isaac、TensorRT、XRoboToolkit、通信依赖及模型；厨房资产见 [厨房说明](docs/wbc_kitchen.md)，离线转换环境见 [转换说明](docs/wbc_lerobot.md)。下面保留安装和固定模型版本的步骤，尚未在空白机器上完成端到端验证。

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

厨房的外部资源还需按 [厨房资产说明](docs/wbc_kitchen.md) 下载到本仓库 `FluxBisim/assets/`；它不包含在 Git 中。

不要只复制某一个 `.usd`：它可能依赖同目录或相邻目录的网格、材质、配置和其他 USD。保留这些资源目录的完整结构。

#### 外部依赖

| 外部内容 | 建议版本 / 基线 | 下载或获取位置 | 安装 / 放置位置 |
| --- | --- | --- | --- |
| planner ONNX | 与本 README 固定的官方模型快照配套 | [Hugging Face：nvidia/GEAR-SONIC](https://huggingface.co/nvidia/GEAR-SONIC) | `gear_sonic_deploy/planner/target_vel/V2/planner_sonic.onnx` |
| Isaac Sim | 现有环境为 4.5.0 | [NVIDIA Isaac Sim 4.5 下载页](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/download.html) | 仓库外；使用其自带 `python.sh` |
| NVIDIA 驱动、CUDA | 现有 PC 使用 CUDA Toolkit 12.4；驱动需符合 Isaac 要求 | [CUDA Toolkit Archive](https://developer.nvidia.com/cuda-toolkit-archive) | 系统安装 |
| TensorRT C++ 头文件与库 | 现有构建使用 10.13.0 | [NVIDIA TensorRT 下载](https://developer.nvidia.com/tensorrt/download/10x) | 仓库外；设置 `TensorRT_ROOT` |
| ONNX Runtime C++ | 安装脚本默认 1.16.3 | [官方 v1.16.3](https://github.com/microsoft/onnxruntime/releases/tag/v1.16.3) | 通常 `/opt/onnxruntime`，可由仓库安装脚本准备 |
| XRoboToolkit PC Service | v1.0.0，Ubuntu 22.04 amd64 安装包 | [官方 Release](https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/tag/v1.0.0) | 安装后应有 `/opt/apps/roboticsservice/runService.sh` |
| PICO XRoboToolkit APK | v1.1.1 | [官方 Unity Client Release](https://github.com/XR-Robotics/XRoboToolkit-Unity-Client/releases/tag/v1.1.1) | 安装到 PICO，不是安装到 Python |
| XRoboToolkit Python SDK | 现有包版本 1.0.2 | [PC-Service-Pybind](https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind) | 安装到终端 3 的 Python 环境 |
| Unitree Python SDK | 现有包版本 1.0.1 | [unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python) | 安装到 Isaac 自带 Python 环境 |
| GStreamer 与 x264 插件 | Ubuntu 软件包 | `apt`，见下文 | 系统安装，终端 4 使用 |

**ONNX 并非一律不能放 GitHub。** 普通 GitHub Git 的单文件限制为 100 MiB；本仓库 encoder / decoder 小于该限制，已经提交。缺少的 planner 为 **773,952,989 字节，约 774 MB / 738 MiB**，应从模型站单独下载，而不是普通 `git add`。

`.trt` 引擎缓存由本机首次运行生成，与 GPU / TensorRT 版本有关，不是要下载的通用模型，也不应作为跨电脑迁移的依赖。

### 安装步骤

已有正常工作的环境无需重装；缺少模型时按下面的固定版本下载步骤补齐。

#### 基础环境与代码

当前使用基线：Ubuntu 22.04 x86_64、NVIDIA RTX 3090、Isaac Sim 4.5.0、Python 3.10；PICO 4 Ultra、双手柄和现有腰部 / 双脚踝追踪器配置。其他 GPU、系统和 SDK 组合需自行验证，不把这份基线理解为最低硬件要求。

在准备存放项目的目录执行（如果已经克隆，不重复执行）：

```bash
git clone https://github.com/Izimmi-Erwin/wbc.git
cd wbc
export WBC_ROOT="$PWD"
```

这里的根目录直接包含 `gear_sonic/` 和 `gear_sonic_deploy/`，**不要再多进入一层 `wbc/`**。

为安装阶段设置路径，先按本机修改以下值；外部依赖放在仓库之外：

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

项目基础依赖固定了 NumPy 1.26.4、SciPy 1.15.3；上面给出与现有 Pinocchio 组合对应的版本，避免随意安装最新 `pin` 导致 NumPy ABI 冲突。manager 默认使用 CPU，不要求给它额外配置 CUDA 推理。

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

先检查，再仅补缺失的通信和视频依赖；以下版本对应现有 NumPy 1.26 环境：

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

2026-09-23 同步后已通过 20 项采集测试、11 项转换测试，并成功试转一条真实超时 HDF5（5388 帧）。资源引用和文件哈希已核对；这些检查不替代实际 PICO 闭环遥操验收。

代码基于 GEAR-SONIC / GR00T-WholeBodyControl、SonicStar 与 FluxVLA 的相关实现。FluxBisim 代码采用 Apache-2.0，厨房资产标注 CC-BY-NC-4.0。原上游参考文档保留在 `docs/source/`，当前厨房运行方式以本文为准。版本记录见 [CHANGELOG](CHANGELOG.md)。
