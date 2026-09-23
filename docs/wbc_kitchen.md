# WBC 厨房、盘子与香蕉

厨房资产位于 `FluxBisim/assets/environments/KitchenRoom/`。资产版本固定到
`limxdynamics/FluxBisimAssets` 的 `1a5b6336d7752c3b605b196ae9a4e159bef3c028`：
216 个文件，共 1,806,226,051 字节，已按上游文件大小和 Git/LFS 哈希校验。
下载清单保存于 `FluxBisim/assets/kitchen_download_manifest.json`。
盘子位于 `FluxBisim/assets/pick_place_fruit/plate/base.usd`，共 1,474,335 字节，
已通过上游 SHA-256 校验；清单为 `FluxBisim/assets/plate_download_manifest.json`。
香蕉位于 `FluxBisim/assets/pick_place_fruit/banana/banana.usd`，连同一张贴图共 2 个文件、
6,604,283 字节，已校验 Git/LFS 哈希；清单为 `FluxBisim/assets/banana_download_manifest.json`。

## 启动

完整五终端与初始化流程见 [README](../README.md)。厨房入口是仓库根目录的 `bash run_wbc_kitchen.sh`，默认启用自动分轮采集；采集行为见 [自动采集](wbc_kitchen_collection.md)。

脚本支持 `ISAAC_PYTHON` 指定 Isaac Sim 4.5 的 `python.sh`，并可在命令末尾追加 `run_sim_loop.py` 参数。未指定 `--isaac-scene-layer-path` 的原始入口仍使用 SonicStar 场景；不指定 `--isaac-episode-directory` 时不启用采集。

## 当前接入范围

- 厨房引用到 `/World/FluxBisimEnvironment`，替换 SonicStar 桌子、杯子和垃圾桶场景层。
- 中岛的 16 个原有碰撞体已启用，中岛保持固定，柜门不参与关节运动。
- L 型橱柜 `/World/FluxBisimEnvironment/Kitchen_Cabinet002` 的全部 53 个已有碰撞体已启用：
  柜体及台面 25 个、8 扇柜门及把手 16 个、2 个抽屉 12 个。台面顶面高度约 `0.790279 m`。
  柜体、柜门和抽屉均保持静态，刚体和关节运动关闭；此次仅启用碰撞，不增加开门、抽屉拉动功能。
  独立水槽、洗碗机和其他厨房资产仍保持原有视觉背景配置。
- `/World/plate` 是可移动刚体，质量设为 `0.2 kg`，启用重力和上游 `convexDecomposition` 碰撞。
  固定生成在中岛 `(0.60, -0.05, 0.75)`，使用上游配置的四元数，落下后由中岛支撑。
  `wbc:randomizationRadius = 0` 表示每次恢复相同的初始位置和朝向，不锁死盘子的物理运动。
  保留上游被引用几何的约 0.21 m 直径，不按源文件的厘米元数据再次缩小。
- `/World/banana` 是 `0.15 kg` 可移动刚体，启用上游 `convexDecomposition` 碰撞，使用上游香蕉朝向 `(0.7315778, 0.681758, 0, 0)` 和源模型缩放 `0.8`。
  以 `(0.55, -2.00, 0.87)` 为圆心，在 XY 平面半径 `0.03 m` 的圆内按面积均匀采样，
  位于水池右侧台面末端。厨房中有一个盘子和一个香蕉。
- 两物体均标记 `wbc:resetOnBackspace`。启动、控制初始化中的 World 重置和每次 Backspace
  都恢复盘子固定生成位置，并重新采样香蕉位置。香蕉的 `wbc:randomizationRadius = 0.03`，
  保持初始朝向、生成高度 `0.87 m`，通过 PhysX 清零线速度和角速度，再自然落到台面。
  圆心取物理启动前的初始世界坐标，不受盘子落下、移动或重新记录机器人快照影响。
  未设置随机半径的其他复位物体仍恢复快照记录的位姿；显式设为零则始终恢复 USD 初始位姿。
- WBC 原有的支撑地面及物理材质继续生效，隐藏其网格显示；厨房可见地面距 z=0 约 0.4 mm。
- 厨房平移 `(0.53, 0.2, 0)`、绕 Z 轴旋转 `-90°`。这是上游 `base_kitchen` 平面位姿的逆变换，
  保留原有厨房与物理世界坐标关系。
- 厨房启动脚本将 G1 根节点初始化到 `(-0.10, -1.90, 0.757)`，位于香蕉右侧、台面末端外侧，
  yaw 为 `-9°`，朝向香蕉随机区域中心。
  ElasticBand 锚点同步移到 `(-0.10, -1.90, 1.5)`，朝向目标同步为 `-9°`，防止拉回原点或转回零朝向。
  Backspace 恢复这个机器人位姿；辅助悬挂的开关流程不变，启用时仍会向原有 `1.5 m` 高锚点提拉。
  不使用厨房启动参数时，根节点 XY/yaw 默认仍为零。
- 仿真配置为 200 Hz；当前 ego 光学参数和 PICO 路由见 README。
- 本地包装层关闭上游缺失的重复墙体 `Kitchen_Wall001_01`，保留已有墙体几何；
  将架子材质中错误的 `3d66Model-19913701-files-024.JPG` 引用修正到已提供的 `19913701-files-024.JPG`。
  原始下载资产未改写。

包装层位于 `gear_sonic/data/scenes/fluxbisim/kitchen.usda`，通过相对路径引用厨房资产。
迁移项目时应保留仓库根目录下 `gear_sonic/` 与 `FluxBisim/` 的相对目录关系。
资产版本改变后，需要重新检查物理覆盖、引用和位姿，不能直接沿用当前配置。

## 固定与随机生成逻辑

配置在 `gear_sonic/data/scenes/fluxbisim/kitchen.usda` 的 `/World/plate` 和 `/World/banana`：
`xformOp:translate` 定义固定圆心，`wbc:randomizationRadius` 定义半径（米）。
实现位于 `gear_sonic/simulation_server/isaac_server.py` 的 `_reset_scene_bodies()`。
沿用 `FluxBisim/tasks/utils.py` 中 `generate_point_in_circle()` 的规则，以及
`FluxBisim/tasks/pick_place_fruit.py` 的 `RANDOMIZATION_RADIUS = 0.03`：

```python
theta = np.random.uniform(0, 2 * np.pi)
r = radius * np.sqrt(np.random.uniform(0, 1))
position = [center_x + r * np.cos(theta), center_y + r * np.sin(theta), center_z]
```

平方根使整个圆的面积上均匀采样。这里只移植采样规则，不引入 FluxBisim 的机器人或 World。
盘子半径为零，直接使用初始位置；香蕉使用上述公式。增大半径或移动圆心后需重新检查水池及台面边缘。

## 重新下载资产

在已安装 Hugging Face CLI 的工具环境中执行，无需修改 WBC Python 环境：

```bash
cd /home/colin/Erwin/isaac-wbc-publish-20260910.s2j08O/wbc
mkdir -p FluxBisim/assets
cd FluxBisim
hf download limxdynamics/FluxBisimAssets \
  --repo-type dataset \
  --revision 1a5b6336d7752c3b605b196ae9a4e159bef3c028 \
  --include 'environments/KitchenRoom/**' 'pick_place_fruit/plate/**' 'pick_place_fruit/banana/**' \
  --local-dir assets
```

只导入厨房资产无需安装 FluxBisim 的 ROS Noetic 或启动双臂 benchmark。
资产仓库许可为 CC-BY-NC-4.0，代码许可为 Apache-2.0；分发与使用时分别遵守。
