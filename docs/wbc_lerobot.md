# WBC HDF5 转 LeRobot v2.1

入口：`tools/convert_wbc_hdf5_to_lerobot.py`。这是 G1 厨房采集的离线转换器，
不修改仿真、WBC 或原 HDF5。当前支持 schema_version=1、
position 控制模式、43 关节和一台 ego 相机。

## 当前机器直接使用

已单独创建 `work_dirs/wbc_lerobot_env`，与 Isaac 和遥操 Python 环境分开。

先检查成功轮次，不创建数据集：

```bash
cd /home/colin/Erwin/isaac-wbc-publish-20260910.s2j08O/wbc
work_dirs/wbc_lerobot_env/bin/python tools/convert_wbc_hdf5_to_lerobot.py \
  work_dirs/kitchen_episodes --dry-run
```

正式转换成功轨迹，输出目录必须尚不存在：

```bash
cd /home/colin/Erwin/isaac-wbc-publish-20260910.s2j08O/wbc
work_dirs/wbc_lerobot_env/bin/python tools/convert_wbc_hdf5_to_lerobot.py \
  work_dirs/kitchen_episodes \
  --output datasets/wbc_banana_success \
  --repo-id local/wbc_banana_success \
  --fps 30
```

默认仅接收 `complete=True`、`success=True`、`outcome=success` 的文件。
没有合格数据会明确退出，不创建空数据集。收集更多数据后使用新的输出目录；
当前不支持向已有 LeRobot 数据集追加。

只为检查转换过程，可以显式选择三分钟超时文件：

```bash
work_dirs/wbc_lerobot_env/bin/python tools/convert_wbc_hdf5_to_lerobot.py \
  work_dirs/kitchen_episodes --outcomes timeout \
  --output datasets/wbc_banana_timeout_test_new \
  --repo-id local/wbc_banana_timeout_test
```

超时数据不等于成功示范。转换器保留原 outcome 和 success，不重新标为成功。
`--limit 1` 可只转换第一个合格源文件。旧 `manual_reset`、`test_timeout`、
未完成文件和 `.partial.hdf5` 不纳入转换。

## 字段与时间对应

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

## 输出与验证

输出包含标准 `data/` Parquet、`videos/` MP4 和 `meta/` 元数据。
图像统计按解码后的 RGB 值计算；转换报告保存源文件与对齐信息。

额外的 `meta/wbc_conversion.json` 记录源文件 SHA256、原成功标记、分段、
筛选原因、采样频率及动作对齐方式。输出帧行号可以追溯至原 HDF5。

转换在新的临时目录中执行。完成并通过官方 LeRobot reader 的每段首尾视频解码后，
才改名为最终输出目录；失败会保留以 `.partial-` 命名的转换目录供排查。
脚本不上传到 Hugging Face，也不覆盖已有输出。

格式可被 LeRobot 读取不代表能直接使用 ALOHA 模型：训练需配置 43 维 state/action、
`observation.images.ego`，并显式选择是否使用速度和物体位姿等额外字段。

## 在另一台机器安装

使用 Python 3.10 和 uv；依赖清单固定 LeRobot commit `55198de096f46a8e0447a8795129dd9ee84c088c`。在仓库根目录执行：

```bash
uv venv work_dirs/wbc_lerobot_env --python 3.10
uv pip install --python work_dirs/wbc_lerobot_env/bin/python \
  --index-strategy unsafe-best-match -r tools/requirements-wbc-convert.txt
```

验证转换逻辑：

```bash
work_dirs/wbc_lerobot_env/bin/python -m unittest discover \
  -s test/test_tools -p test_wbc_conversion.py -v
```
