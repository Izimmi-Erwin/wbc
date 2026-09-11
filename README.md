# Isaac WBC：PICO 全身遥操作 G1

本仓库将 PICO / XRoboToolkit 的全身追踪接入 GEAR-SONIC Whole-Body Control，在 Isaac Sim 中控制 G1 的 29 个身体关节，并通过手柄 Trigger 控制双手。机器人画面经 Remote Vision 回传 PICO。

本文对应 **2026-09-11 的原生 XRoboToolkit 五终端流程**。代码来源于 GEAR-SONIC / SonicStar / FluxVLA WBC 的演进，并包含本地 Isaac、DDS、遥操转发和视频桥接适配；不是独立开发的全部上游算法，也不是完整的 FluxVLA 训练系统。

> 适用范围：单台 Linux PC 上的 Isaac 仿真。以下 `sim`、回环 DDS 和关闭 CRC 检查的部署配置不能直接当作真机操作说明。首次启动按第 6～7 节初始化；遥操运行中需要重置场景时先停止控制，给操作者留出安全活动空间。
>
> 验证边界：启动参数、文件路径、控制状态机、模型校验值已按当前代码核对；五终端操作顺序按用户实际流程整理。本文新增的干净环境安装步骤尚未在另一台空白电脑端到端复现，不代表所有依赖最新版都已验证兼容。

## 目录

- [1. 整体数据流程](#1-整体数据流程)
- [2. 仓库包含什么，还要下载什么](#2-仓库包含什么还要下载什么)
- [3. 新电脑安装](#3-新电脑安装)
- [4. 下载并校验 ONNX 模型](#4-下载并校验-onnx-模型)
- [5. 每次运行前：PICO 和网络](#5-每次运行前pico-和网络)
- [6. 五终端运行指令](#6-五终端运行指令)
- [7. 开始遥操、按键和停止](#7-开始遥操按键和停止)
- [8. 第三人称与第一人称跟随视角](#8-第三人称与第一人称跟随视角)
- [9. 核心代码索引](#9-核心代码索引)

## 1. 整体数据流程

```text
PICO 全身追踪 + 手柄
  │ XRoboToolkit Unity Client：FullBody
  ▼
XRoboToolkit PC Service → Python SDK
  │
  ▼
终端 3：pico_manager_thread_server.py
  ├─ 人体坐标转换、SMPL 表达、重采样、G1 FK 标定
  ├─ POSE / PLANNER 模式、启停、手柄输入
  └─ 双手 Trigger → 预设开合目标
  │ ZMQ :5563
  ▼
终端 1 内部 IsaacDeployRelay ── ZMQ :5556 ──► 终端 2：Sonic WBC
  ▲ 键盘转发 :5562                                 │ encoder / decoder / planner
  │                                               │ DDS 身体 + 双手命令
  │                                               ▼
  └───────────────────────────────────────── Isaac G1 仿真
                                                  │ DDS 关节状态 → WBC
                                                  │ 相机 ZMQ :5555
                                                  ▼
                                     终端 4：Remote Vision Bridge
                                                  │ H.264 / TCP
                                                  ▼
                                          PICO 接收端 :12345

WBC 状态反馈 :5557 → manager 标定 / 可选数据采集
终端 5：send_keyboard_cmd.py ── ZMQ :5580 ──► Isaac：k → 9 → backspace 初始化
```

几个概念不要混淆：

- **PICO 24 点**由 PICO 追踪能力通过 XRoboToolkit 客户端和 SDK 提供，不是 WBC 从视频识别人。追踪点数不等于需要 24 个物理传感器。
- **SMPL** 是人体模型与姿态表达体系。manager 将追踪信息转换为策略需要的表示；不是把 PICO 的 24 个关节直接当成 G1 的 29 个电机目标。
- **ZMQ `5556`** 传递姿态、规划请求和启停等上层输入；**DDS** 传递机器人低层命令与状态，两者职责不同。
- **身体和手指分开控制**：当前身体由 WBC 驱动，Trigger 触发预设手部开合，不是逐根人手手指的精确重定向。双手状态反馈是机器人关节状态，不代表又获取了一套人体骨架。
- **Remote Vision Bridge 就是终端 4**。它只做视频显示，不计算身体动作，也不向 WBC 发送控制命令。

当前流程不需要 PICO Browser、WebXR body-tracking、CloudXR、IsaacTeleop 或 Televiz。旧文档中的浏览器版本限制不应套用到这条原生客户端路线。

## 2. 仓库包含什么，还要下载什么

### 2.1 已包含的关键文件

| 内容 | 仓库位置 | 用途 |
| --- | --- | --- |
| WBC encoder | `gear_sonic_deploy/policy/release/model_encoder.onnx` | 编码运动 / 遥操条件，约 50.1 MB，已纳入 Git |
| WBC decoder | `gear_sonic_deploy/policy/release/model_decoder.onnx` | 根据观测与编码条件产生控制输出，约 40.9 MB，已纳入 Git |
| 配套观测配置 | `gear_sonic_deploy/policy/release/observation_config.yaml` | 定义模型输入、历史帧和 encoder 模式 |
| G1 USD、URDF、MJCF 与网格 | `gear_sonic/data/robots/`、`gear_sonic/data/robot_model/` | Isaac 加载、关节映射、相机参数与 FK 标定 |
| 场景与道具 | `gear_sonic/data/scenes/`、`gear_sonic/data/assets/` | 当前仿真环境及被引用资源 |
| 人体骨架辅助资源 | `gear_sonic/data/human/human_joints_info.pkl`、`gear_sonic/trl/utils/smplx/` | 人体姿态处理；不是需要另下载的完整训练数据集 |
| 原生 Unitree C++ SDK | `gear_sonic_deploy/thirdparty/unitree_sdk2/` | C++ 推理端 DDS 通信；不代替 Python SDK |

不要只复制某一个 `.usd`：它可能依赖同目录或相邻目录的网格、材质、配置和其他 USD。保留这些资源目录的完整结构。

### 2.2 仓库之外的必需项

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

## 3. 新电脑安装

已有正常工作的环境可以跳到第 4 节补模型，再按第 5～7 节操作。**不要为看 README 而重装现有 Isaac 或删除原来的虚拟环境。**

### 3.1 基础环境与代码

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

`/path/to/...` 是占位符，不能原样使用。终端之间不共享刚设置的变量；第 6 节的运行指令分别包含所需变量。

### 3.2 系统依赖、视频工具与 C++ 构建

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

### 3.3 终端 3：独立的遥操 Python 环境

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

### 3.4 终端 1 / 4：Isaac 自带 Python

这是另一个环境，终端 3 中安装成功不代表 Isaac 可以导入相同包。保留 Isaac 自带的 NumPy、SciPy、Torch，**不要在里面直接升级整套训练 / teleop 依赖**。

先检查，再仅补缺失的通信和视频依赖；以下版本对应现有 NumPy 1.26 环境：

```bash
"$ISAAC_PYTHON" -m pip install "numpy==1.26.4" "opencv-python==4.10.0.84" \
  pyzmq msgpack msgpack-numpy PyYAML "cyclonedds==0.10.2"
export UNITREE_SDK_DIR="$WBC_DEPS/unitree_sdk2_python"
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git "$UNITREE_SDK_DIR"
"$ISAAC_PYTHON" -m pip install --no-deps -e "$UNITREE_SDK_DIR"
"$ISAAC_PYTHON" -c 'import numpy, yaml, zmq, msgpack, msgpack_numpy, cv2, cyclonedds, unitree_sdk2py; print("Isaac bridge imports OK")'
```

`--no-deps` 的前提是前一条命令已满足 SDK 1.0.1 的依赖；若未来 SDK requirements 改变，先核对其 `setup.py`。如果 CycloneDDS 报找不到原生库，按 [Unitree SDK 安装说明](https://github.com/unitreerobotics/unitree_sdk2_python#installation) 配置其要求的 CycloneDDS，再在这个 Isaac Python 中安装，不要装到另一个系统 Python 后就认为问题解决。

## 4. 下载并校验 ONNX 模型

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

## 5. 每次运行前：PICO 和网络

1. 启动 / 确认 XRoboToolkit PC Service 运行。需要手动启动时执行 `bash /opt/apps/roboticsservice/runService.sh`；终端 3 也会尝试启动该服务，不必反复开多个实例。
2. 戴好追踪器和手柄，在 PICO 中完成设备配对、身体追踪标定。
3. 打开原生 XRoboToolkit，**选择 FullBody 模式，并勾选 Head、Controller 和 Send**，连接到 PC 的实际局域网 IP。
4. 查看 PICO 自己的 IP，供终端 4 的 `--headset-host` 使用。PC 地址和 PICO 地址不是同一个。
5. 在 PICO 的 Remote Vision 页面选择 `PICO4U`，点击 **Listen**，然后启动终端 4。

可在 PC 检查地址和连接：

```bash
ip -4 -brief address
ss -tnp | rg ':63901'
```

`ss` 的本地端是 PC，对端是 PICO。历史使用过 PC `192.168.180.224`、PICO `192.168.180.214`，它们只是示例，DHCP / 换网络后必须重新确认。PICO 界面中的 PC Service 地址填 PC；终端 4 填 PICO。

| 地址 / 通道 | 提供方 → 使用方 | 用途 |
| --- | --- | --- |
| PC `:63901` | PICO 连接 PC Service | XR 数据传输 |
| PC `127.0.0.1:60061` | PC Service → Python SDK | SDK 本地服务，不是要在 PICO 填写的 IP |
| PC ZMQ `:5563` | manager → Isaac relay | 姿态与模式命令 |
| PC ZMQ `:5562` | Isaac 键盘转发 → relay | 部署键盘命令内部入口 |
| PC ZMQ `:5556` | Isaac relay → WBC | 统一遥操输入；只让 relay 绑定该发布端口 |
| DDS，仿真使用 `lo` / domain 0 | WBC ↔ Isaac | `rt/lowcmd`、`rt/lowstate` 与双手命令 / 状态 |
| PC ZMQ `:5557` | WBC → manager / exporter | `g1_debug` 状态反馈 |
| PC ZMQ `:5555` | Isaac → 视频桥 / exporter | 相机帧 |
| PICO TCP `:12345` | PC 视频桥连接 PICO | Remote Vision H.264 |
| PC ZMQ `:5580` | 终端 5 键盘工具 → Isaac | `k`、`9`、`backspace` 初始化 |

PC 与 PICO 应位于可互通的可信网络，避免 AP 客户端隔离。防火墙应允许相应设备间通信；不要把控制端口开放到公网。DDS 的 `lo` 是同机仿真网络选择，不是让 PICO 连接 `127.0.0.1`。

## 6. 五终端运行指令

下面提供**当前电脑的路径版**，每个终端可以单独粘贴。换电脑时替换 `WBC_ROOT`、两个 Python 路径、`TensorRT_ROOT` 和 PICO IP，不要复制旧电脑的虚拟环境或 build 缓存。

| 变量 | 当前电脑位置 |
| --- | --- |
| 仓库根目录 | `/home/colin/Erwin/isaac-wbc-publish-20260910.s2j08O/wbc` |
| Isaac Python | `/home/colin/Erwin/isaac-sim-4.5.0/python.sh` |
| 遥操 Python | `/home/colin/Erwin/GR00T-WholeBodyControl/.venv_teleop/bin/python` |
| TensorRT 根目录 | `/home/colin/opt/tensorrt-10.13.0/usr` |

现有环境复用了原 GR00T 目录里的 Python，但通过 `PYTHONPATH` 指定**本仓库代码**；新电脑按第 3 节建独立环境即可，不必额外克隆整个 GR00T 仓库。

### 终端 1：Isaac 仿真、DDS、相机和 relay

```bash
export WBC_ROOT="/home/colin/Erwin/isaac-wbc-publish-20260910.s2j08O/wbc"
export ISAAC_PYTHON="/home/colin/Erwin/isaac-sim-4.5.0/python.sh"
cd "$WBC_ROOT"
PYTHONPATH="$WBC_ROOT${PYTHONPATH:+:$PYTHONPATH}" "$ISAAC_PYTHON" \
  gear_sonic/scripts/run_sim_loop.py \
  --simulator isaac --enable-onscreen \
  --isaac-publish-camera --isaac-camera-source gui_perspective \
  --isaac-gui-perspective-zoom 2.0 \
  --camera-port 5555 --isaac-camera-fps 60 \
  --isaac-forward-k-to-deploy \
  --isaac-robot-model sonic_g1_43dof
```

等待 Isaac 场景加载完成、机器人和地面出现。`sonic_g1_43dof` 是 29 个身体关节 + 14 个手部关节；省略后可能使用默认 29-DoF 资产，不能据此测试双手。

`--isaac-forward-k-to-deploy` 同时启用这条流程所需的 relay；不要另启动一个占用 `5556` 的姿态发布器。相机请求 60 FPS，实际帧率受渲染负载影响。

### 终端 2：Sonic WBC 推理

```bash
export WBC_ROOT="/home/colin/Erwin/isaac-wbc-publish-20260910.s2j08O/wbc"
export TensorRT_ROOT="/home/colin/opt/tensorrt-10.13.0/usr"
cd "$WBC_ROOT/gear_sonic_deploy"
source scripts/setup_env.sh
bash deploy.sh --input-type zmq_manager sim
```

按部署脚本提示确认，等模型和 TensorRT 引擎准备完成。首次运行可能长时间编译引擎，不能仅凭等待就判断卡死。这里的 `sim` 会使用仿真网络与相关参数；不要改成真机网卡试错。

### 终端 3：PICO 全身数据与 manager

```bash
export WBC_ROOT="/home/colin/Erwin/isaac-wbc-publish-20260910.s2j08O/wbc"
export TELEOP_PYTHON="/home/colin/Erwin/GR00T-WholeBodyControl/.venv_teleop/bin/python"
cd "$WBC_ROOT"
PYTHONPATH="$WBC_ROOT${PYTHONPATH:+:$PYTHONPATH}" "$TELEOP_PYTHON" \
  gear_sonic/scripts/pico_manager_thread_server.py --manager --port 5563
```

必须等到出现：

```text
Manager controls: A+X=toggle mode, A+B+X+Y=start/stop policy
```

这行出现在 SDK 检测到身体数据、manager 完成相关初始化之后。一直显示 `waiting for body data...` 时按 ABXY 不会跨过等待：先确认 PICO 已选择 **FullBody**，然后 `Ctrl+C` 停止终端 3 并重新运行。**Connected 只表示连接建立，不等于全身数据已到达。**

### 终端 4：Isaac 画面回传 PICO

先在 PICO Remote Vision 中选择 `PICO4U` 并点击 `Listen`。以下 IP 必须替换为 PICO 当前地址：

```bash
export WBC_ROOT="/home/colin/Erwin/isaac-wbc-publish-20260910.s2j08O/wbc"
export ISAAC_PYTHON="/home/colin/Erwin/isaac-sim-4.5.0/python.sh"
export PICO_IP="192.168.180.214"  # 示例：先核对 PICO 当前 IP
cd "$WBC_ROOT"
PYTHONPATH="$WBC_ROOT${PYTHONPATH:+:$PYTHONPATH}" "$ISAAC_PYTHON" \
  gear_sonic/scripts/run_xrobotoolkit_remote_vision.py --headset-host "$PICO_IP"
```

看到 `connected to PICO receiver` 且发送计数继续增加，再到头显确认画面。默认输入 `5555`、输出 TCP `12345`，H.264 视频设置为 2160×810、30 FPS、约 8 Mbps。

默认 SBS 是**同一张相机图复制给双眼**，不是两台相机生成的真实双目深度。终端 1 的相机 60 FPS 与终端 4 的编码 30 FPS 是不同环节。

视频桥断开 / 重启后，PICO 的 Listen 可能需要重新点击。不要先用端口探测工具消耗这个单连接接收入口，再期待视频桥直接连上。

### 终端 5：发送 k → 9 → backspace 初始化

确认终端 2 模型就绪、终端 3 出现 controls 提示、Isaac 正常运行后，在 **ABXY 标定之前**执行。这个终端用于分步发送初始化命令，每条命令执行后退出，不是额外的常驻服务。

先设置路径并发送 `k`，启动部署规划控制；观察机器人状态稳定后再进行下一步：

```bash
export WBC_ROOT="/home/colin/Erwin/isaac-wbc-publish-20260910.s2j08O/wbc"
export TELEOP_PYTHON="/home/colin/Erwin/GR00T-WholeBodyControl/.venv_teleop/bin/python"
cd "$WBC_ROOT"
"$TELEOP_PYTHON" gear_sonic/scripts/send_keyboard_cmd.py k
```

然后在同一终端发送一次 `9`，释放初始开启的 ElasticBand 辅助，等待机器人落地稳定：

```bash
"$TELEOP_PYTHON" gear_sonic/scripts/send_keyboard_cmd.py 9
```

最后发送一次 `backspace`，重置到场景初始状态；该重置保留当前 ElasticBand 开关状态。确认仿真稳定后再进行第 7 节的 ABXY 标定：

```bash
"$TELEOP_PYTHON" gear_sonic/scripts/send_keyboard_cmd.py backspace
```

以上是首次进入遥操前的仿真初始化顺序，**逐步执行并观察，不要循环发送**。`k` 和 `9` 都是切换操作，重复执行可能反向切换；初始化完成后不要再次发送 `9`。已经进入人体遥操后若要重置，应先停止遥操，再重新初始化和标定。

## 7. 开始遥操、按键和停止

### 7.1 一次完整操作顺序

1. 完成 PICO 追踪标定，选择 FullBody 并勾选 Head、Controller、Send；依次启动终端 1、2、3，准备 PICO Listen 后启动终端 4，并打开终端 5。
2. 确认终端 2 模型就绪，终端 3 出现 controls 提示，Isaac 仿真在运行，机器人处于可安全开始的初始状态。
3. 在终端 5 按第 6 节**依次发送 `k → 9 → backspace`**，每一步观察状态，完成初始化并等待仿真稳定。
4. 操作者保持稳定的中性 / 标定姿态，按一次 **A+B+X+Y**。manager 此时获取标定并从 `OFF` 进入 `PLANNER`；前一步的 `k` 已启动部署端，此处是进入 manager 控制并标定。
5. **完全松开组合键**，待机器人站稳。辅助已在初始化中释放，此处不再发送 `9`。
6. 按一次 **A+X** 切换到 `POSE`，观察终端 3 的模式切换输出，再以小幅度动作测试身体跟随。
7. 分别测试左右 Trigger 开合；确认姿态和画面正常后再扩大动作范围。

### 7.2 常用按键

| 输入 | 当前逻辑 |
| --- | --- |
| A+B+X+Y，处于 OFF | 标定并启动，进入 PLANNER |
| A+B+X+Y，已经运行 | 发送 stop，进入 OFF，**当前实现会退出 manager 进程**；下一次需重启终端 3 |
| A+X | PLANNER ↔ POSE |
| B+Y | POSE ↔ PLANNER_FROZEN_UPPER_BODY；冻结上身目标的规划模式 |
| 左摇杆按下 | 在规划类模式与 VR 三点子模式之间切换，不是初次测试的必需操作 |
| POSE 中按住左菜单键 | POSE_PAUSE；松开恢复，不是独立急停 |
| 左 / 右 Trigger > 0.5 | 对应手部预设抓握；释放则张开 |
| `k` | 部署启停转发开关；终端 5 初始化的第一步，勿重复切换 |
| `9` | 切换 ElasticBand 辅助；初始辅助开启时按一次可释放 |
| `backspace` | 重置 Isaac 场景并保留辅助开关状态；初始化按终端 5 顺序执行，遥操运行中重置则先停止遥操 |
| 聚焦终端 2 后按 `O` / `o` | WBC 键盘急停入口；不要只依赖 PICO 无线链路停止 |

键盘初始化命令统一在终端 5 执行，完整指令见第 6 节。

停止时先停策略：追踪正常可用 ABXY；追踪或网络异常时用终端 2 的键盘急停入口。确认控制停止后，再依次结束视频桥、manager、WBC 和 Isaac。除上述启动初始化外，遥操运行中不要直接重置 / 重启仿真。

## 8. 第三人称与第一人称跟随视角

终端 1 示例使用：

```text
--isaac-camera-source gui_perspective
```

它发布 Isaac GUI Perspective 的第三人称画面；启动缩放通过 `--isaac-gui-perspective-zoom 2.0` 调整。

如需机器人第一人称跟随画面，先停止控制和视频桥，将终端 1 的相机来源替换为：

```text
--isaac-camera-source robot_ego
```

然后重新启动终端 1，并在 PICO 重新 Listen 后启动终端 4。其余部署进程若因仿真重启丢失状态，应重新初始化与标定。

这里是**两种可选相机来源，当前向头显发布一个选中的画面**，不是自动同时传两路视频。机器人第一人称跟随也不等于 PICO HMD 转头驱动相机。

## 9. 核心代码索引

| 文件 / 目录 | 在当前流程中的职责 |
| --- | --- |
| [run_sim_loop.py](gear_sonic/scripts/run_sim_loop.py) | 终端 1，启动 Isaac、选择资产 / 相机、管理控制转发 |
| [isaac_server.py](gear_sonic/simulation_server/isaac_server.py) | Isaac 场景、物理步进、关节控制、相机发布 |
| [isaac_unitree_bridge.py](gear_sonic/robot_interface/isaac_unitree_bridge.py) | 仿真与 Unitree DDS 身体 / 双手通信桥 |
| [Isaac configs.py](gear_sonic/utils/isaac_sim/configs.py) | 当前启动入口使用的 Isaac 参数 |
| [MuJoCo 配置目录](gear_sonic/utils/mujoco_sim/) | 保留的上游配置 / 资源；不是当前 Isaac 启动参数的主配置 |
| [pico_manager_thread_server.py](gear_sonic/scripts/pico_manager_thread_server.py) | 终端 3，SDK 取数、人体处理、遥操状态机、双手输入 |
| [g1_fk.py](gear_sonic/utils/teleop/g1_fk.py) | 用 G1 正运动学获取腕部 / 躯干参考，仍用于标定，不能因关闭旧相机跟随就删除 |
| [deploy.sh](gear_sonic_deploy/deploy.sh) | 终端 2 的编译、模型路径和推理启动封装 |
| `gear_sonic_deploy/src/g1/` | WBC 主控制、观测构造、推理与 DDS |
| `gear_sonic_deploy/src/TRTInference/` | TensorRT 推理与本地引擎缓存 |
| [run_xrobotoolkit_remote_vision.py](gear_sonic/scripts/run_xrobotoolkit_remote_vision.py) | 终端 4，JPEG 相机接收、H.264 编码、PICO TCP 视频协议 |
| [send_keyboard_cmd.py](gear_sonic/scripts/send_keyboard_cmd.py) | 终端 5，向 Isaac 依次发送初始化键盘命令 |
