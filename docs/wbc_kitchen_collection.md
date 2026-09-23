# 厨房自动分轮采集

当前任务为把香蕉从水池右侧台面放到中岛的红盘子里。厨房启动脚本默认启用自动采集。

每轮最多 3 分钟：A+X 进入跟踪且 ego 图像就绪、开始记录后，按单调时钟累计 180 秒实际时间。
香蕉提前入盘仍按成功条件结束；到期未完成则请求退出跟踪，确认后保存 HDF5、复位，等待下一次手动 A+X。
这是本轮总时长限制，中途暂停跟踪不会暂停计时；复位后的等待时间不计入下一轮。
超时轮标记为 `outcome=timeout, complete=True, success=False`，metadata 记录 `episode_timeout_seconds=180`。
计时到期触发结束请求，实际复位还需等待 manager 确认和文件落盘。

采集中手动按 Backspace 会丢弃当前尝试的临时 HDF5，复位后自动重开记录并重新计满 180 秒，
保持当前 PLANNER/POSE 模式，不要求再按 A+X。已保存的历史文件不受影响。
如果本来就在等待下一轮，Backspace 仅复位，仍等待 A+X；如果自动结束已进入退出跟踪握手，
手动 Backspace 会丢弃尚未落盘的当前轮，完成握手后继续等待 A+X。

## 操作入口

按 [README](../README.md) 启动五终端并初始化。采集的自动结束顺序为：暂停物理 → 请求 manager 进入 PLANNER 并确认 → 保存文件 → 复位 → 等待新的手动 A+X。

本机采集接口为 `127.0.0.1:5564`；终端 3 需显示 episode control 地址。`--episode_control_port` 是 manager 参数，`--isaac-episode-control-port` 是仿真端对应参数，二者需一致。

## 成功条件

成功检查使用 PhysX 的实际接触力，而不是只比较水平距离。下列条件需连续满足 0.6 秒：

- 香蕉的水平包围盒完全位于盘子中心半径 `0.105 m` 内，根节点高于盘子且高度差小于 `0.09 m`。
- 盘子正面朝上；香蕉与盘子的接触力大于 `0.03 N`。
- 香蕉与机器人手掌、手指、腕部的接触力总量小于 `0.02 N`。
- 香蕉、盘子线速度均小于 `0.05 m/s`，香蕉角速度小于 `0.5 rad/s`。
- 采样时有有效的新第一人称图像。

香蕉在盘子上方悬空、从上方经过、仍被抓住、盘子翻转或物体仍在明显运动时，不计为成功。
包围盒条件比较保守；香蕉跨在盘沿外面时需要放得更居中。

## HDF5 文件

默认目录：`/home/colin/Erwin/isaac-wbc-publish-20260910.s2j08O/wbc/work_dirs/kitchen_episodes/`。
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

## 边界和失败处理

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

## 代码及验证

- 自动采集与成功判定：`gear_sonic/simulation_server/kitchen_episodes.py`。
- 本机请求/确认及 A+X 等待逻辑：`gear_sonic/utils/teleop/episode_control.py`。
- 终端 3 接入：`gear_sonic/scripts/pico_manager_thread_server.py`。
- 配置参数：`--isaac-episode-directory`、`--isaac-episode-control-port`、`--isaac-episode-hz`。
  不指定 directory 的其他启动方式不启用此功能。

验证命令（在仓库根目录执行）：

```bash
PYTHONPATH="$PWD" /home/colin/Erwin/isaac-sim-4.5.0/python.sh \
  -m unittest discover -s gear_sonic/tests -p test_kitchen_episodes.py -v
```

20 项测试覆盖入盘判据、HDF5、manager 确认、180 秒超时、手动丢弃与重新计时、相机等待和采集边界缓存清理。测试不能证明真实头显延迟或任意姿态下的 reset 稳定性。
