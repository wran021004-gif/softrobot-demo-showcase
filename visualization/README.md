# 统一 MuJoCo 展示层

覆盖 M1–M6、C1、P3、PHS-2D、Spring Button C009 和新 M7。普通展示默认使用连续重叠的分段外壳、关节护套、基座罩和末端过渡罩，隐藏腱线及内部导向件。**Presentation-only visualization**：这是外观解释层，不是新的物理外壳或已验证制造结构。

所有三维画面使用 MuJoCo 原生 `mujoco.viewer` / `mujoco.Renderer`。纯视觉几何通过 `MjvScene` / `mjv_initGeom` / `mjv_connector` 添加，不加入 MjModel，不保存 XML。允许修改的仅是显示实例的 rgba、材质、可视化参数、灯光与相机；质量/完整惯量、关节、碰撞参数、腱路、执行器、solver/integrator/timestep 及保存状态不变。原 XML 字节在读前/读后检查一致。

## 风格与真实状态

- ColorBrewer BuGn n=3：`#E5F5F9 / #99D8C9 / #2CA25F`；分类采用 Dark2，灰阶采用 ColorBrewer Greys。没有红绿唯一编码。
- 实际轨迹为实线，只显示已执行的保存状态前缀；参考轨迹为虚线。实际 Tip 是球，目标/当前参考为开放立方体；方向箭头另按世界坐标显示。
- 力箭头使用实际保存外力和真实 COM/材料点，不以 Tip 替代载荷作用点。
- 只选择原始时间戳上的保存状态，不插值 qpos/qvel。MP4 记录每帧 source index/物理时间/播放时间。慢放始终显示倍率和真实时间。
- 浅背景、弱反射、无强地面阴影。相机使用经原生 GL-camera 检查的正交投影。MuJoCo 3.13 的自由相机需要 `model.vis.global_.orthographic=1`，fovy 为正交长度；没有改物理 opt。
- P3 保留三个独立粗单元、清楚的段间边界；没有把 3 cells 伪装成验证过的连续体。PHS 保留 9 段色带及分界套。Spring 保留物理工具/按钮的真实几何和相互关系。
- 二维图由 Matplotlib 绘制，独立 PNG 600 dpi 与矢量 SVG；三维姿态原生渲染为 PNG，不把栅格图冒充矢量三维图。

## 文件职责

`mujoco_renderer.py` 负责恢复状态、原生渲染、相机/灯光和交互；`presentation_overlay.py` 负责原生 DECOR 外壳、护套和标记；`camera_presets.py`、`palettes.py` 统一风格；`video_export.py` 用 imageio/FFmpeg 编码 MP4；`adapters/` 分别读取各任务原 CSV/NPZ。

读取器同时识别指定统一成果目录的 M1–M6 等命名，以及 Showcase 的 M1_reach_hold/M2_loaded_hold 等既有命名。不会为统一接口改写历史格式。原有科学 replay.py 全部保留。

## 运行

从成果目录根执行，安装新增依赖时使用根目录 `requirements_visualization_v2.txt`，旧 requirements.txt 保留：

```powershell
python -m pip install -r requirements_visualization_v2.txt
python -B -m visualization.mujoco_renderer m2 --case positive --speed .2 --loop
python -B -m visualization.mujoco_renderer spring_button --speed 1 --loop
python -B -m visualization.mujoco_renderer m7 --case negative_y --speed .2 --loop
python -B -m visualization.video_export m2 --case positive --speed .2 --output visualization/results/videos/user_m2.mp4
```

Space 暂停、R 重播、T 切换 Technical View、1/2/3 切换相机。`--technical` 可在启动时保留原物理几何与腱路。`--root <数据根目录>` 可显式选择冻结数据位置。

## 本轮检查和导出

十类已有任务和 M7，每类只选一个代表工况，检查初态/中间态/末态三帧；共 33 个代表姿态。检查截图和物理数组一致性见 [presentation_checks.json](results/presentation_checks.json)。历史任务没有新的物理积分或重新评价。

已导出十段历史任务代表 MP4，加 M7 五工况和一段末端特写，共 16 段；索引 [video_index.json](results/video_index.json)。十类历史任务各增加一张核心科学图，[scientific_index.json](results/scientific_index.json)，原始图表及 C009 的原 24 张 PNG/SVG 全部保留。

本展示层不改变原验收身份：M3 为 development draft；M4/M6、C1 某些工况存在原有稠密指标边界；M5 是瞬态指向；PHS 为 development_pass、未注册 SELECT。详见各任务文档和历史原始结果。渲染通过不等于科学任务重新 PASS。


## M8 与 raw 导出（2026-11-12）

新增 M8 和 Figure-eight 适配器，后者只读取原四腱、两周期保存数组。M8 三候选均 FAIL，视频为失败诊断；没有成功穿越。`--raw` 在帧函数中直接返回 Renderer.render() 原数组，跳过 Pillow/HUD；imageio/FFmpeg 仅编码。M8 的同形 mesh 外壳不扩大物理边界，透明墙和 Technical 标记只修改显示层。

```powershell
python -B -m visualization.mujoco_renderer m8 --speed .2 --loop
python -B -m visualization.mujoco_renderer figure_eight --speed .2 --loop
python -B -m visualization.video_export m8 --raw --speed .2 --output M8_narrow_passage_traversal/results/videos/m8.mp4
```

M8 仅检查一个已有任务 M1 可加载，没有历史重新仿真、优化、评价或批量视频重渲染。原上文历史检查记录保持为原轮次，不代表本轮重新执行。详见 [M8](../M8_narrow_passage_traversal/README.md)。
