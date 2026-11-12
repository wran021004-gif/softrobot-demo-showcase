"""Write evidence-bound task documentation; no simulation."""
import shutil,json
from pathlib import Path
import numpy as np
import mujoco
from environment import H,read,write,sha
def main():
    spec=read(H/'task.json');rows=[]
    for name in ['baseline_fixed','six_augmented','compact_six']:
        p=H/'cases'/name;r=read(p/'evaluation.json');z=read(p/'source_zero_stroke.json');m=mujoco.MjModel.from_xml_path(str(p/'model.xml'));r['structure_mass_kg']=float(m.body_mass.sum());rows.append({'name':name,'actuators':m.nu,'evaluation':r,'source_zero_stroke':z})
    write(H/'results/comparison.json',rows)
    table='| 候选 | 通道数 | 最小间隙 mm | 终点误差 mm | 峰值张力 N | 峰值绳速 mm/s | 自初态行程 mm | 结论 |\n|---|---:|---:|---:|---:|---:|---:|---|\n'
    for row in rows:
        r=row['evaluation']['metrics'];table+=f"| {row['name']} | {row['actuators']} | {r['minimum_clearance_m']*1000:.3f} | {r['tip_final_error_m']*1000:.3f} | {r['max_tension_n']:.3f} | {r['max_rope_speed_m_s']*1000:.3f} | {r['max_stroke_m']*1000:.3f} | FAIL |\n"
    (H/'results/comparison.md').write_text('# 固定结构与结构候选：均为 FAIL\n\n'+table+'\n共同初态还违反原模型零位行程；这些是失败诊断，不构成有效的成功结构消融。\n',encoding='utf8')
    usage=read(H/'results/budget_usage.json');usage.update(candidate_count=3,formal_integrations=3,independent_integrations=1,steps_per_integration=12000,total_state_records=48004,geometry_pairs_per_state=108,formal_and_independent_distance_queries=48004*108,search_stopped=True,final_candidate='baseline_fixed',final_full_run='Selected baseline complete run reused; no second baseline optimization/integration',historical_physics_steps=0,halfstep_tests=0,random_seeds=0,preflight_physics_steps=0,preflight_algebraic_only=True)
    usage['algebraic_forward_evaluations_in_saved_optimization_records']={str(p.parent.relative_to(H)):read(p)['mj_forward_evaluations'] for p in H.rglob('optimization.json') if p.parent.name!='final_candidate'}
    usage['additional_preflight_note']='precheck.py inverse workspace probes were algebraic only; no mj_step. Saved node nfev does not equal total finite-difference forward calls.'
    write(H/'results/budget_usage.json',usage)
    (H/'task.md').write_text('''# M8 自然语言任务与冻结条件

用固定基座的腱驱连续体软臂，先对齐入口，再通过一个沿世界 X 的直线矩形孔道，到达出口后目标并保持。全身柔性 cell、导向件、连接件及 10 g 工具都须无接触，不能从外部绕行。固定基座不穿过孔道。

冻结环境：入口 X=205 mm，出口 X=265 mm，中心 Y=0、Z=150 mm；内部宽 32、高 24、长 60 mm，四面物理墙厚 4 mm。目标 (285,0,150) mm。基座 (0,8,150) mm，偏航 −0.03 rad。初态 Tip (195,−100,150) mm；该初态几何无碰撞，但后续原零位审计发现其不满足执行器行程，因此不是可接受的任务初态。

几何间隙每物理步 ≥1 mm；机器人与壁面 contact=0；Tip 按顺序通过两个有效开口；真实 cell 中心线与两平面的交点均在孔内，通道 slab 内的连续线段须在孔内；最终连续 0.2 s 误差 ≤8 mm、速度 ≤20 mm/s。张力 0–8 N、行程 ±15 mm、绳速 ≤25 mm/s。另检查从原模型 q=0 定义的绝对行程，禁止用预备姿态重设零位。

总时长 6 s，物理步 0.5 ms，反馈更新 10 ms，5 s 后参考保持。正式搜索之前冻结 task.json 和完整初始状态 SHA。预检中通道 Z 只从 100 调到 150 mm 一次；尺寸和阈值从未放宽。居中预备姿态不能无碰撞，因此正式冻结为侧向预备姿态。preflight_* 保存被拒绝的代数预检；其中状态没有作为物理轨迹使用。

本次是 Codex-assisted engineering design; not Main autonomous Study。未调用 Main / Reporter / Provider。所有候选 FAIL，停止搜索，不启动下一任务。
''',encoding='utf8')
    (H/'README.md').write_text('''# M8 狭窄孔道穿越：FAIL，未找到可接受解

已完成有限结构/控制设计、真实物理积分、逐物理步导出、独立进程复现及 MuJoCo 原生可视化。**没有成功穿越，也没有找到满足原行程的预备初态。** 视频为失败诊断，不能展示实际未发生的入口通过、出口通过或目标保持。

本轮是 **Codex-assisted engineering design; not Main autonomous Study**。历史任务的模型、轨迹、结果及验收身份不变；历史物理积分为 0。

## 任务与结构

自然语言及全部阈值见 [task.md](task.md)、[task.json](task.json)。目标要求全身柔性臂通过孔道，无接触到达出口后目标，不能只检查 Tip 或从侧面绕行。

本轮最优失败记录保留原 **C1 四腱结构**，4 根腱对应 4 个独立理想张力执行通道。两段柔性长度 **158.5 / 109 mm**，总长 267.5 mm；每段 12 个 cell、每 cell 两个弯曲自由度，共 48 qpos / 48 qvel。第一段椭圆半轴 9.5 / 7.6 mm，截面角 0.35 rad，Young 模量 7.2 MPa、密度 1100 kg/m³；第二段矩形全宽/全高从 13.3×9.5 渐变到 11.4×8.55 mm，角从 −0.25 到 +0.15 rad，模量 5.4 MPa、密度 1050 kg/m³。腱路半径 18 mm。完整真实截面、转角、惯量、阻尼、腱路和材料见 [structure.json](structure.json) 和 [model.xml](model.xml)。

几何核对发现：原 FamilySpace 的矩形 width/height 是全宽/全高，不能把它们再乘二；段间导向物理 box 为 **50×50 mm**，不是小截面柔性体。所有候选都保留该导向体并纳入距离评价；其大部分运动停留在入口前。这不证明导向体能够进入 32×24 mm 开口。模型仍为真实离散弯曲链，没有轴向伸缩/剪切/扭转，不宣称连续材料实验。

末端保留 C1 的 **10 g** payload、原完整 COM 和非对角惯量；不换成仅显示的假负载。局部 COM=(7,1,−2) mm；惯量对角 (6,8,9)×10⁻⁷ kg·m²，Ixy=Iyz=10⁻⁷ kg·m²，Ixz=0。末端工具物理 box 半尺寸 (9,9,8) mm；Tip 是原 payload 局部 X=14 mm 的 site。基线总运动质量约 67.32 g。

## 冻结环境与初态

孔道沿 X：入口 205 mm，出口 265 mm，内部 **60×32×24 mm（长×宽×高）**，四壁厚 4 mm；中心 (Y,Z)=(0,150) mm。固定基座 (0,8,150) mm、偏航 −0.03 rad；目标 (285,0,150) mm，位于出口后 20 mm。

预检曾因低位初态无法平衡且碰撞，把通道中心 Z 从 100 调到 150 mm 一次。居中预备姿态仍有碰撞，故在正式积分前保存侧向无碰撞静力姿态：Tip=(195,−100,150) mm，qvel=0，初始张力约 (0,6.5464,0.2821,4.7701) N。完整 qpos/qvel/张力见 [initial_state.npz](initial_state.npz)，所有结构使用相同 qpos/qvel 和实际初始 cell 位置；六腱初始张力按各模型力平衡分配。初态离入口平面 10 mm、全身最小壁间隙 43.742 mm。

**关键限制：原执行器零位行程审计在积分后补充完成。** 初态 T3 已相对源模型 q=0 缩短 **44.707 mm**，超过 ±15 mm；全程相对原零位最大位移 **46.912 mm**。所以几何无碰撞的初态并不是执行器可接受的任务初态。这些完整积分保留作失败诊断，不构成满足协议的成功 A/B 实验，不允许用初态重设执行器零点隐藏问题。冻结文件和原阈值不变，评价器增加了更严格的原零位审计。

## 求解、预算与结果

通过原 Core FamilySpace `generate_family`、Parameter Registry 对应的受支持参数域及 `resolve/compile_xml` 生成结构；没有手改 XML 的机械参数。新 M8 XML 独立添加四面真实碰撞墙和每个运动 robot geom 的显式 contact pair。基线保留原机械结构；两个 B 方案分别为增加 2 根腱的六通道结构，以及 0.8 截面比例、14 mm 腱路半径、15 / 13.18359375 MPa 模量的紧凑六通道结构。长度和离散数不变。没有电机选型或 BOM。

控制使用有界代数静力参考、全身距离软惩罚和 10 ms 原生状态反馈张力分配，更新间线性保持张力斜坡。代数参考允许存在残差，绝不作为成功物理状态；只有 `mj_step` 的完整轨迹决定验收。控制约束评价相同；软惩罚优化器没有保证碰撞、行程和绳速硬可行。参考连续化和动态跟踪未能消除入口局部不可行和分支变化，不能把失败归结为结构的数学不可能性。

'''+table+'''
最优失败记录最小有符号间隙 **−0.261261 mm**，发生于 **1.7955 s**，最近对 **payload_shape / tunnel_left**；负值为真实几何穿入。共有 **8452 个状态**存在 native robot-wall contact。Tip 未依次通过有效入口和出口；终点误差 **26.631 mm**，最后 0.2 s 最大速度约 1.61×10⁻⁶ mm/s，但位置不合格，连续有效保持为 **0 s**。最大张力 **7.508568 N**；实际绳速 **30.767145 mm/s**，自初态行程 **39.786146 mm**，后两项均超限。

3 个正式候选各完整积分一次，共 36,000 步。选定基线复用其完整积分，不额外重新优化或重跑固定结构；只对它作 **1 次独立新进程闭环复现**、再积分 12,000 步。合计 **48,000 个物理积分步 / 48,004 个真实保存状态**。每状态 27 个运动 geom×4 面墙=108 对，总计正式及复现 **5,184,432 次**逐步距离查询，代数规划查询另计。无随机搜索、半步长试验、历史积分或历史评估。

独立复现重新计算控制，不注入保存轨迹或控制序列；全部 qpos/qvel、控制、张力、Tip、cell 中心、几何距离、最近对及 contact 数组最大差 **0**，任务仍 FAIL。见 [independent_replay.json](results/independent_replay.json)。原零位补充审计只重读已有状态，不增加物理积分。

## 全身评价与数据

实际环境 MuJoCo 3.13.0 确认提供 `mj_geomDistance(model,data,g1,g2,distmax,fromto)`；采用 SI 米、秒、牛顿，返回有符号距离和世界最近点。对每个 0.5 ms 状态，独立枚举全部运动柔性 mesh、50 mm 导向 box、连接件、payload 与四面墙的 108 对；排除固定基座、floor。独立距离不依赖过滤是否允许接触，另读取引擎实际 contact。没有把 Presentation Shell 或 Tip 到中心线距离作为碰撞边界。记录全程最小距离、时间、几何名、两个世界点、当时全部 qpos/qvel 和完整时间曲线。

中心线用各真实 cell 的世界 COM 中心，首尾加入实际基座/Tip；只对相邻真实点做平面线段求交和 slab 裁剪，不制造新物理状态。检查 Tip 在有效开口内依次越过入口/出口，以及运动后的连续中心线是否处于通道内。没有连续时间 swept collision 或 timestep 间安全证明。

`results/trajectory_timestep.csv` 含 12,001 行真实时间、全部 qpos/qvel、Tip 位置/速度、24 个 cell 中心、阶段、全身间隙、最近对、接触数和中心线检查。`control_timestep.csv` 含每步指令、实际张力、腱长、真实腱速和约束状态；`control_updates.csv` 保留 600 个真实反馈更新节点。`clearance_timestep.csv` 给出每步最近几何名称、最近世界点、contact 数和 1 mm 检查。完整数组在 `trajectory.npz`，ID 映射在 `geometry_map.json`，原零位行程在 `source_zero_stroke.json`。CSV 不是展示插值。各候选、最终别名和独立复现保留完整数据。

## 原生展示与环境

Python **3.11.16**、MuJoCo **3.13.0**、NumPy **2.4.6**、SciPy **1.17.1**；版本和来源 SHA 见 [provenance.json](provenance.json)。新环境在仓库/成果根目录运行 `python -m pip install -r requirements_visualization_v2.txt`。结构再生成还需要 `SOFTROBOT_CORE` 指向 Core commit `2c83f8b7bd654328dd7648f844295c54a7c15223`；回放、评价和现有 bank 的物理复现不需要 Core。

统一回放（只恢复保存状态并 mj_forward，不积分）：

```powershell
python -B -m visualization.mujoco_renderer m8 --speed .2 --loop
python -B -m visualization.mujoco_renderer m8 --technical --speed .2 --loop
python -B M8_narrow_passage_traversal/replay.py --speed .2 --loop
```

原生视频导出：

```powershell
python -B -m visualization.video_export m8 --raw --speed .2 --output M8_narrow_passage_traversal/results/videos/m8.mp4
```

Space 暂停、R 重播、T 切换 Technical、1/2/3 换相机。Presentation 使用跟随真实 cell 的原 mesh 同形 BuGn 外壳，不放大碰撞边界；隐藏腱线与内部导向。透明四壁仅改显示，不改物理几何。Technical 显示全部物理墙、腱路、中心线、最近点紫色标记和最近墙轮廓；原生 viewer 文字给出间隙和几何对。失败姿态的真实穿入不会被显示外壳遮掩。

6 张独立二维科学图分别导出 600 dpi PNG 和 SVG；初/中/末 Presentation 和 Technical 原生截图见 [图片索引](results/figures/index.json)。三维全部用 MuJoCo；raw 分支直接将 Renderer.render() 数组交给编码器，无 PIL ImageDraw、OpenCV、Matplotlib 或文字合成。MP4 901 帧已全部解码检查，1280×960，30 fps、0.2 倍速度，逐帧 source index 另存。

[初态](results/figures/initial.png)、[中间](results/figures/middle.png)、[末态技术视图](results/figures/final_technical.png)、[失败诊断 MP4](results/videos/m8.mp4)。实际没有发生成功穿越阶段，因此不伪造五阶段成功动画。

`run.py` 有候选和独立验证预算记录，已消耗的名称/独立验证不会自动重复；本轮搜索停止。有限失败不能证明所有四腱/六腱结构均不可能，也不证明任何全局最少执行器数。完成本轮归档后不启动后续任务。
''',encoding='utf8')
    vd=H.parent/'visualization/tasks';(vd/'m8').mkdir(exist_ok=True);(vd/'m8/README.md').write_text('# M8：未找到可接受穿越解\n\n[完整任务、结构、失败指标及启动命令](../../../M8_narrow_passage_traversal/README.md)。4/6/6 通道三个候选均 FAIL；独立复现最大差 0 仅证明重现失败。初态几何无碰撞，但原零位行程不合格。\n\n```powershell\npython -B -m visualization.mujoco_renderer m8 --speed .2 --loop\npython -B -m visualization.video_export m8 --raw --speed .2 --output M8_narrow_passage_traversal/results/videos/m8.mp4\n```\n',encoding='utf8')
    (vd/'figure_eight').mkdir(exist_ok=True);(vd/'figure_eight/README.md').write_text('# Figure-eight：原四腱、两周期保存结果\n\n读取原 `figure_eight_tracking/trajectory_full.npz`，4 个独立通道、单段 270 mm、12 cells、2 g 原负载。22 s 保存结果包含两个周期；不与 M4 旧折线 Tracking 混淆。本轮未重新求解或评价。\n\n```powershell\npython -B -m visualization.mujoco_renderer figure_eight --speed .2 --loop\n```\n',encoding='utf8')
    root=H.parent/'README.md';old=root.read_text(encoding='utf8');history=H.parent/'history/README_before_M8_20261112.md'
    if not history.exists():shutil.copyfile(root,history)
    text=old.replace('# Softrobot Benchmark · 统一 MuJoCo 展示与 M7','# Softrobot Benchmark · 统一 MuJoCo 展示、M7 与 M8')
    text=text.replace('## 启动','''M8 本轮结论：**FAIL，未找到可接受穿越解**。三候选均发生碰撞/约束超限；预备姿态还违反原执行器零位行程。数据、原生视频和逐步评价见 [M8 README](M8_narrow_passage_traversal/README.md)。完成一次独立复现、无后续搜索；原任务结论不变。

统一入口新增 `figure_eight` 和 `m8`。新增 `--raw` 可直接导出 MuJoCo Renderer 原始帧，无二维 HUD；M8 视频全部采用该模式。Figure-eight 保留原四腱结构及两个周期数据，不重新计算。

## 启动''',1)
    text=text.replace('| M7 | [未知方向冲击后的主动恢复]', '| Figure-eight | [原四腱、两周期保存结果](visualization/tasks/figure_eight/README.md) | `python -B -m visualization.mujoco_renderer figure_eight --speed .2 --loop` |\n| M8 | [全身狭窄孔道穿越：FAIL](M8_narrow_passage_traversal/README.md) | `python -B -m visualization.mujoco_renderer m8 --speed .2 --loop` |\n| M7 | [未知方向冲击后的主动恢复]')
    text=text.replace('Showcase 原 Figure-eight 额外 Demo 也保留旧入口 `viewer/showcase.py eight`，本轮没有重新求解或改造它。','Showcase 原 Figure-eight 额外 Demo 同时保留旧入口 `viewer/showcase.py eight` 和新增统一 `figure_eight` 读取适配器；物理模型和保存结果不变。')
    root.write_text(text,encoding='utf8')
    vread=H.parent/'visualization/README.md';vread.write_text(vread.read_text(encoding='utf8')+'\n\n## M8 与 raw 导出（2026-11-12）\n\n新增 M8 和 Figure-eight 适配器，后者只读取原四腱、两周期保存数组。M8 三候选均 FAIL，视频为失败诊断；没有成功穿越。`--raw` 在帧函数中直接返回 Renderer.render() 原数组，跳过 Pillow/HUD；imageio/FFmpeg 仅编码。M8 的同形 mesh 外壳不扩大物理边界，透明墙和 Technical 标记只修改显示层。\n\n```powershell\npython -B -m visualization.mujoco_renderer m8 --speed .2 --loop\npython -B -m visualization.mujoco_renderer figure_eight --speed .2 --loop\npython -B -m visualization.video_export m8 --raw --speed .2 --output M8_narrow_passage_traversal/results/videos/m8.mp4\n```\n\nM8 仅检查一个已有任务 M1 可加载，没有历史重新仿真、优化、评价或批量视频重渲染。原上文历史检查记录保持为原轮次，不代表本轮重新执行。详见 [M8](../M8_narrow_passage_traversal/README.md)。\n',encoding='utf8')
if __name__=='__main__':main()
