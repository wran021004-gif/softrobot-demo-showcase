# Softrobot Benchmark · 统一 MuJoCo 展示与 M7

本轮完成九类既有 Benchmark、Spring Button C009 的统一展示，并新增 **M7 External Impact Active Recovery：PASS**。成果统一导出到 `D:\softrobot-benchmark-exports\base_only_accepted_20261028`。

全部成果属于 **Codex-assisted engineering design; not Main autonomous Study**。Main / Reporter / Provider 调用均为 0。历史任务未重新优化、积分或重新验收；原始模型/状态/科学评价冻结。M7 单独使用有限控制搜索和一次独立新进程闭环验证。

## 启动

当前 Windows 可直接使用已有展示环境：

```powershell
Set-Location 'D:\softrobot-benchmark-exports\base_only_accepted_20261028'
$benchmarkPython = 'D:\softrobot-demo-showcase\.venv\Scripts\python.exe'
& $benchmarkPython -B -m visualization.mujoco_renderer m7 --case positive_y --speed .2 --loop
& $benchmarkPython -B -m visualization.mujoco_renderer spring_button --loop
```

新环境安装新增依赖：`python -m pip install -r requirements_visualization_v2.txt`。然后从成果目录根执行下面入口；`--root <数据目录>` 可显式定位，Space 暂停、R 重播、T 切换 Technical View、1/2/3 切换相机。

| 任务 | 自然语言与结构/结果说明 | 代表回放命令 |
|---|---|---|
| M1 | [到达并保持](visualization/tasks/m1/README.md) | `python -B -m visualization.mujoco_renderer m1 --case central_high --speed .2 --loop` |
| M2 | [持续外力定位保持](visualization/tasks/m2/README.md) | `python -B -m visualization.mujoco_renderer m2 --case positive --speed .2 --loop` |
| M3 | [短时快速到达，development draft](visualization/tasks/m3/README.md) | `python -B -m visualization.mujoco_renderer m3 --speed .2 --loop` |
| M4 | [原 timed 折线跟踪](visualization/tasks/m4/README.md) | `python -B -m visualization.mujoco_renderer m4 --speed .2 --loop` |
| M5 | [瞬态位置与工具轴指向](visualization/tasks/m5/README.md) | `python -B -m visualization.mujoco_renderer m5 --speed .2 --loop` |
| M6 | [20 g 工具的位置/轴扫描](visualization/tasks/m6/README.md) | `python -B -m visualization.mujoco_renderer m6 --speed .2 --loop` |
| C1 | [四独立通道多任务](visualization/tasks/c1/README.md) | `python -B -m visualization.mujoco_renderer c1 --case M1_A --speed .2 --loop` |
| P3 | [被动渐进刚度，三个粗单元](visualization/tasks/p3/README.md) | `python -B -m visualization.mujoco_renderer p3 --case plus_x --loop` |
| PHS-2D | [九段被动水平稳定](visualization/tasks/phs2d/README.md) | `python -B -m visualization.mujoco_renderer phs2d --case s050_positive --loop` |
| Spring Button | [C009 完整可行解](visualization/tasks/spring_button/README.md) | `python -B -m visualization.mujoco_renderer spring_button --loop` |
| M7 | [未知方向冲击后的主动恢复](M7_external_impact_active_recovery/README.md) | `python -B -m visualization.mujoco_renderer m7 --case negative_y --speed .2 --loop` |

统一 MP4 导出示例：

```powershell
python -B -m visualization.video_export m2 --case positive --speed .2 --output visualization/results/videos/user_m2.mp4
python -B -m visualization.video_export m7 --case negative_y --speed .2 --output M7_external_impact_active_recovery/results/videos/user_negative.mp4
```

原有各任务科学 replay.py、历史 manifest 和全部原数据保留。Showcase 原 Figure-eight 额外 Demo 也保留旧入口 `viewer/showcase.py eight`，本轮没有重新求解或改造它。旧根 README 在历史目录保存；各任务的原 README 不覆盖，新说明放在 `visualization/tasks/`。

## 统一展示层

[visualization/README.md](visualization/README.md) 解释完整实现。三维机器人全部由 MuJoCo 原生 viewer/Renderer 绘制；二维科学图才使用 Matplotlib，视频由 imageio/FFmpeg 编码。

每个外壳跟随真实 cell/body 姿态，重叠减少缝隙，增加连接护套、基座罩和末端过渡。普通主展示隐藏 tendon 和内部导向；目标、负载、按钮、真实外部环境保留。P3 清楚保留三个离散单元，PHS 保留九段辨识。所有新增几何仅为 **Presentation-only MjvScene DECOR**，不参与质量、惯量、碰撞或控制，不写回 XML。

ColorBrewer BuGn n=3 为主色，Dark2 分类、Greys 中性色；实际/参考同时用实线/虚线、球/开放立方体区分。浅背景、柔光、原生正交相机。所有视频显示真实物理时间和倍率；只选取实际保存的时间戳状态，无状态插值，实际尾迹没有未来部分。逐帧 source index CSV 随视频保存。

## C009 来源与冻结导出

来源仓库 `wran021004-gif/softrobot-demo-showcase`，分支 `codex/button-control-autosearch-v1-2-20261111`，commit **`bd4fe89f13b286f127f1fc336a860318be109119`**。

整个 `spring_button_press/v1_2/` 原归档按源提交文件清单复制，保留控制/runner/replay 源码、完整物理状态/控制历史、原 Evaluator 和独立闭环结果、原 README 和 24 张 PNG/SVG。导出包额外提供同字节 `model.xml` 别名及必要的冻结父版本依赖，原文件不更改。

C009 与 FINAL_C009 的 `result.json` SHA-256 均为 **`9928d7f71233ee931005bfe1f8b4cc5d34cdf61fd315d91b217e65686f5c7c0f`**。原结果为 PASS，未重新优化或重算。来源、逐文件复制核对和别名绑定保存在导出审计中。

## M7 结果

保留原 M2 两段×12 cells、6 主动腱和 10 g payload，model.xml SHA 与原 M2 完全一致。1 s 任务、0.5 ms 物理步、10 ms 控制；0.50–0.55 s 在真实 COM 施加 ±0.04 N 半正弦横向力。Controller 只接收正常状态和目标，没有冲击时间/方向/力信息。

| 工况 | 评价 | 恢复时间 ms | Final Hold 最大误差 mm | 最大速度 mm/s |
|---|---|---:|---:|---:|
| 无冲击，反馈 | PASS | 0 | 4.028 | 5.417 |
| Positive Y，主动反馈 | PASS | 0 | 4.032 | 5.431 |
| Negative Y，主动反馈 | PASS | 1.5 | 4.131 | 5.566 |
| Positive Y，冻结输入 | FAIL：最终保持 | 294.5 | 7.128 | 21.583 |
| Negative Y，冻结输入 | FAIL：连续恢复和最终保持 | 无有效 100 ms 区间 | 7.620 | 25.357 |

主动工况全程最大张力 **7.576 N**、最大实际绳速 **21.786 mm/s**、最大腱行程 **5.711 mm**。预保持最大误差 9.019 mm、最大速度 12.278 mm/s，全部物理步满足原定 10 mm/20 mm/s。冲量数值为 ±1.273135 mN·s，脉冲后外力严格归零。

**7 组候选、20 次工况积分、40,000 个物理步；1 次独立新进程五工况闭环验证，完整历史最大差为 0。** 两个冻结输入对照均失败，主动作用有必要对照支持。正向主动在冲击结束前已回到容限，因此恢复时间按定义为 0；反向为 1.5 ms。幅值只有效调整一次；最终基线最大偏移仍略低于 5 mm，作为诊断如实保留，未继续增大力或放宽验收。

完整任务、结构、全部候选、逐步 CSV/NPZ、外力 COM/冲量和验证见 [M7 README](M7_external_impact_active_recovery/README.md)、[evaluation.json](M7_external_impact_active_recovery/results/evaluation.json)。最终速度预测保护禁用，实际绳速通过；不宣称经过认证的保护器或硬件鲁棒性。

## 图片、视频和必要检查

- 十类已有任务各一个代表工况，加 M7，共 **33 张初/中/末原生姿态检查**：[presentation_checks.json](visualization/results/presentation_checks.json)。零历史物理积分；XML、质量/完整惯量、碰撞、腱路、执行器、solver/integrator/timestep、恢复的 qpos/qvel/控制均保持一致。
- **16 段 MP4**：[视频索引](visualization/results/video_index.json)，包含十类已有任务、M7 五工况和末端特写。
- 十类已有任务各一张独立核心科学图：[科学图索引](visualization/results/scientific_index.json)。
- M7 的 8 张二维科学图分别提供 600 dpi PNG/矢量 SVG，加 3 张原生三维姿态 PNG：[M7 图片索引](M7_external_impact_active_recovery/results/figures/index.json)。没有拼接面板；原 C009 的 24 张图保持原字节。

历史科学边界仍在：M3 是 draft；M4/M6 及 C1 部分稠密指标曾越限；M5 是瞬态而非静止瞄准；PHS 是未注册 SELECT 的 development_pass，且 Z 可移动。展示检查通过不能替代任务验收，也不改写这些旧结果。

## 清单与版本

原 `manifest.json` 仅作历史记录，**不将其旧失败状态当作当前整包通过**。新的 [export_manifest_v2.json](export_manifest_v2.json) 对指定统一成果目录当前全部文件给出 SHA-256；只排除自身，避免自引用。`export_audit_v2.json` 区分历史清单的既有不一致、根 README 的版本更新，以及冻结数据不变的检查。

公开 Git 分支同时保留 Showcase 既有目录布局；清单绑定的是上述完整本地导出目录，不应把它套到布局不同的 Git 工作树。显式只读校验命令：

```powershell
python -B -m visualization.verify_manifest --root 'D:\softrobot-benchmark-exports\base_only_accepted_20261028'
```

本轮独立分支：`codex/external-impact-active-recovery-20261111`，基于指定 C009 commit 建立独立 worktree；不改 Spring 原分支。原始数据与模型不因展示而增加物理外壳、摩擦、电机带宽或真机验证。
