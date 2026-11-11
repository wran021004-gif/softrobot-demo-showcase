# M7 外部冲击主动恢复：PASS

`margin_feedback` 在无冲击、Positive Y 和 Negative Y 工况全部通过固定任务的全部逐物理步验收。正、负 Y 冻结输入对照均 FAIL。一次独立新进程闭环验证重新执行全部五工况，qpos、qvel、控制、Tip 位置/速度及外力历史最大差全部为 0（容差 1e-10）。不是保存 CSV 的播放验证。

本轮为 **Codex-assisted engineering design; not Main autonomous Study**。Main / Reporter / Provider 调用均为 0。

## 任务、结构与环境

自然语言任务及完整验收：[task.md](task.md)、[task.json](task.json)。目标世界坐标 `(0.290,-0.035,0.190) m`，总时长 1 s，0.50–0.55 s 在真实 payload COM 施加世界 Y 半正弦脉冲，峰值 ±0.04 N，脉冲外外力归零。

结构完全使用原 M2：[model.xml](model.xml) 是原 robot.xml 的按字节副本，[structure.json](structure.json) 也是原结构文件副本。两段长 160/110 mm，各 12 cells，6 主动腱，10 g 物理 payload，模量 7.2/5.4 MPa；第一段椭圆半轴 9.5/7.6 mm，第二段矩形从 13.3×9.5 渐变至 11.4×8.55 mm。原腱路半径 18 mm、短组三腱根部角 0/120/240°、长组三腱 60/180/300°，完整局部路由/截面转角/惯量以冻结 JSON/XML 为准。

物理 timestep 保持 **0.5 ms**，控制周期 **10 ms**，原 `implicitfast`、solver、接触和执行器设置均不变。没有结构搜索。初始 qpos/qvel 为原零位形，控制预张紧 0.20 N，属于新任务明示初始条件。

依赖：Python 3.11、MuJoCo 3.13.0、NumPy 2.4.6、SciPy 1.17.1；展示还使用 Matplotlib/Pillow/imageio/FFmpeg。模型、源文件及版本绑定见 [provenance.json](provenance.json)。求解不依赖生产 Main 或原 Store。

## 控制器与主动性证据

已检查现有 Core GVS NMPC 的状态反馈接口。当前展示 Python 缺少其 CasADi 依赖，因此实现最小任务专用控制器：原生模型代数平衡、完整力切线、10 ms 离散 LQR，以及 0–8 N 有界张力分配。不是开环冲击补偿，也没有重新开发通用框架。

代数参考同时平衡六腱几何行程；对腱速增加反馈代价，使用带余弦起停的接近参考和 130 ms 启动反馈包络。控制增益、目标和配置在两个方向完全相同。0.27 s 后参考固定，反馈持续工作。`Controller.command(t,qpos,qvel)` 不接收冲击发生时间、方向或外力；预测 scratch data 的外力也始终为零。

最终配置见 [effective_control.json](effective_control.json)。早期预测绳速保护与高增益分配冲突的失败试验保留；**最终禁用这项预测保护**，通过轨迹、平衡分配和反馈包络降低实际绳速。最终结论来自每个 0.5 ms 的真实 `ten_velocity`，不声称存在已认证安全投影或扰动鲁棒保证。

冲击前，五工况状态/控制历史逐项相同。冲击后，正/负主动控制相对无冲击控制的最大变化分别为 0.214/0.232 N，最大实际 Tip 冲击差分别为 1.078/1.292 mm。冻结输入基线自 0.5 s 起保持原实际控制值，仍有材料被动恢复；两者均不能通过 Final Hold。主动反馈不是把材料弹性回位换一个名称。

## 五工况评价

| 工况 | 状态 | 冲击后恢复时间 ms | Final Hold 最大误差 mm | Final Hold 最大速度 mm/s |
|---|---|---:|---:|---:|
| 无冲击，反馈 | PASS | 0 | 4.028 | 5.417 |
| Positive Y，主动反馈 | PASS | 0 | 4.032 | 5.431 |
| Negative Y，主动反馈 | PASS | 1.5 | 4.131 | 5.566 |
| Positive Y，冻结输入 | FAIL：Final Hold | 294.5 | 7.128 | 21.583 |
| Negative Y，冻结输入 | FAIL：连续恢复、Final Hold | 无有效 100 ms 区间 | 7.620 | 25.357 |

所有工况冲击前保持最大误差 **9.019 mm**、最大速度 **12.278 mm/s**。主动工况全程最大张力 **7.576 N**，最大实际腱速 **21.786 mm/s**，最大腱行程 **5.711 mm**；无碰撞、警告或控制异常。位置保持最小裕量约 0.981 mm，绳速裕量约 3.214 mm/s，不夸大为硬件鲁棒性。

数值冲量为 ±**1.273135 mN·s**；解析半正弦冲量为 ±1.273240 mN·s，相对求积误差约 0.00822%。实际施力、COM 位置和累积冲量均保存。最大“相对 0.5 s 位姿的位移”还包含正常收敛漂移；结果另外保存与无冲击控制的差，区分冲击响应和正常运动。

详细逐项检查、峰值、完整性及对照：[evaluation.json](results/evaluation.json)；独立验证：[independent_replay.json](results/independent_replay.json)。

## 搜索预算和数据

共 **7 组候选**：5 个单工况控制筛选、2 组完整五工况评价；最后仅对选定候选作 **1 次独立进程五工况闭环验证**。合计 **20 次完整工况积分 / 40,000 个物理步**，没有多种子、网格或步长收敛试验。[搜索记录](results/search_record.md) 与 [budget_usage.json](results/budget_usage.json) 保留失败和成本。

`cases/` 中每工况：

- `trajectory_timestep.csv`：2,001 行，时间、完整 48 qpos/48 qvel、Tip 世界位置/速度。
- `control_timestep.csv`：每物理步的实际控制、实际张力、几何腱长/腱速；`control_updates.csv` 是 100 个真实 10 ms 更新节点。
- `external_force_timestep.csv`：瞬时世界力、真实 payload COM、实际累积冲量；额外列保存前一步实际保持的外力和该步起点 COM，消除 pre/post-step 歧义。
- `trajectory.npz`：相同的完整原始数组；`controller_updates.json`：控制器输入、输出和分配状态；`result.json`：本工况评价。

没有插值生成新的物理状态。初始点加 2,000 个 0.5 ms 物理步；独立验证未注入保存状态或控制序列。`calibration/`、`candidates/`、`independent/` 保留各次完整归档，最终五工况另置于 `cases/`。

## 图片、视频与启动

8 张二维科学图各自保存 600 dpi PNG/矢量 SVG，另有 3 张原生 MuJoCo 2400×1800 姿态 PNG，见 [figures/index.json](results/figures/index.json)。最大偏移姿态选择 **相对无冲击对照的冲击差峰值 0.5555 s**，不是把正常后续收敛当作冲击峰值。所有图独立，没有组合面板。

[正向主动 MP4](results/videos/positive_y.mp4)、[反向主动 MP4](results/videos/negative_y.mp4)、[正向冻结输入 MP4](results/videos/positive_y_no_feedback.mp4)、[反向冻结输入 MP4](results/videos/negative_y_no_feedback.mp4)、[无冲击 MP4](results/videos/no_impulse.mp4)、[正向末端特写 MP4](results/videos/positive_y_tip_detail.mp4)。视频统一使用 MuJoCo Renderer，主视频完整显示机器人，特写另标文件；0.2 倍播放，画面显示真实物理时间及倍率。

从统一成果目录根运行：

```powershell
python -B M7_external_impact_active_recovery/replay.py --case positive_y --speed .2 --loop
python -B M7_external_impact_active_recovery/replay.py --case negative_y --speed .2 --loop
python -B -m visualization.video_export m7 --case positive_y --speed .2 --output M7_external_impact_active_recovery/results/videos/user_positive.mp4
python -B -m visualization.video_export m7 --case positive_y --speed .2 --tip-detail --output M7_external_impact_active_recovery/results/videos/user_tip_detail.mp4
```

Space 暂停，R 重播，T 切换 Technical View，1/2/3 切换相机。回放只恢复保存状态并 `mj_forward`，不积分。求解 `run.py` 保留有限预算和独立验证已消耗标记，避免把随意重跑混入本轮预算。

外壳、保护套、基座罩和显示背景全部是 **Presentation-only** 的原生 MjvScene DECOR 几何，不参与质量、惯量、碰撞或腱路。P3 的粗单元和 PHS 九段有独立展示规则。本 M7 是离散柔性链和理想张力执行器下的固定工况工程解，未验证电机、摩擦、绳弹性、未知传感延迟或硬件。
