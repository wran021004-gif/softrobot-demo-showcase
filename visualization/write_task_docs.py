"""Add task-specific presentation docs; never edit original task READMEs."""
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
TASKS={
'm1':('Reach / Hold','将末端从初始状态移动到所选目标，在原最后 60 ms 保持。','原两段各 12 cells、6 主动腱、10 g payload。','central_high / positive_high / negative_low','central_high','M1/robot.xml 或 Showcase M1_reach_hold/robot.xml；具名关节 trajectory_timestep.csv、reference.csv。','原独立目标任务；保持误差约 9.9 mm。三工况是独立初态，不是连续切换；不扩大保持时长。','position_error'),
'm2':('External-load Reach / Hold','在原固定目标附近保持，payload COM 在 0.10–0.35 s 承受所选持续世界外力。','160/110 mm 两段，各 12 cells，6 主动腱，10 g 物理负载。','zero / positive / negative','positive','M2/robot.xml 或 Showcase M2_loaded_hold/robot.xml；完整具名关节 CSV、reference.csv、外力列。','原正向保持速度裕量仅约 0.000051 mm/s；原记录没有卸载/主动恢复。M7 是独立新任务。','position_error'),
'm3':('Fast Reach / development draft','在原短时截止时间内到达目标，原最后 50 ms 检查位置与速度。','原两段各 12 cells、6 主动腱、10 g 负载。','default','default','M3/robot.xml 或 Showcase M3_fast_reach/robot.xml；trajectory_timestep.csv、reference.csv。','仍为 development draft。原最后 50 ms 通过不等于长期稳定保持或正式批准 Benchmark。','position_error'),
'm4':('Legacy Timed Polyline Tracking','按原时间跟踪世界坐标折线路径。','原两段各 12 cells、6 主动腱、10 g 负载；不与同拓扑其他模型合并。','default','default','M4/robot.xml 或 Showcase M4_legacy_tracking/robot.xml；完整状态 CSV 和原 timed reference。','原 20 ms 网格误差 11.987763 mm；逐物理步峰值 12.095371 mm，终点速度 325.362 mm/s。保留技术回放身份，不标为稳定停止跟踪。','tracking_yz'),
'm5':('Transient Position + Tool-axis Pointing','在原截止时间同时满足末端位置与工具局部 +X 指向。','原两段各 12 cells、6 主动腱、10 g 物理负载。','default','default','M5/robot.xml 或 Showcase M5_tool_axis/robot.xml；状态、工具轴及 reference CSV。','原终点速度约 457.755 mm/s；只约束位置和轴方向，不包含 roll、静止瞄准或保持要求。','tool_axis_error'),
'm6':('Loaded Position + Tool-axis Scan','携带原刚性工具，按原参考扫描位置与工具轴方向。','原两段各 12 cells、6 主动腱、20 g 真实刚性 payload，保留完整非对角惯量。','default','default','M6/robot.xml 或 Showcase M6_loaded_scan/robot.xml；状态/工具轴/reference CSV。','原网格通过；逐步轴误差峰值 10.052466° 超过原 10° 指标。无环境表面接触，不能宣传接触扫描验证。','tool_axis_error'),
'c1':('Four Independent Actuation Channels','同一四通道结构分别完成三个位置目标和一个工具轴任务。','158.5/109 mm 两段，各 12 cells，4 主动腱，10 g 负载。','M1_A / M1_B / M1_C / M5','M1_A','C1_min_tendons_multi_task/model.xml 与 cases/*/trajectory.csv；或 Showcase C1_four_tendons 原目录。','不宣称全局最少执行器。M1_B 稠密保持速度 30.396440 mm/s 超过 30；M5 终点速度 84.781 mm/s，不是静止瞄准。','tension_channels'),
'p3':('Passive Progressive Stiffness','在固定被动放绳下承受分级载荷，并观察非线性刚度和卸载回位。','3 段、每段仅 1 个粗离散 cell，2 被动腱、0 主动通道、10 g payload；固定放绳 143.1667 mm/根。','plus_x / minus_x（原附加工况文件仍保留）','plus_x','P3_passive_progressive_stiffness/cases/*/model.xml、完整 trajectory.csv；或 Showcase 已有原状态显示样本。','原开发工况 PASS，刚度比约 3.699；没有连续体收敛或真机证明。外壳按三个真实刚体姿态画出，清楚保留段间边界。','force_displacement'),
'phs2d':('Passive Horizontal Tip Stabilization','加载中段时允许结构变形，同时使 Tip 的水平 XY 位移较小。','9 段×8 cells，144 弯曲 DOF，4 被动腱、0 主动通道，10 g 负载。','s030/s040/s050/s060/s070_positive/negative，另 equilibrium','s050_positive','PHS2D_passive_head_stabilization 原 robot.xml、states_timestep.npz；或 Showcase 原状态 display_replay.npz。','原分类 development_pass、未注册 SELECT；水平稳定不等于 Z 固定。原记录无卸载过程；保留九段色带和连接边界。','tip_horizontal_xy'),
'spring_button':('Spring Button Press / C009','真实探头接触按钮，按压并保持，再跟随回零和撤离。','149/120.5 mm 两段，各 12 cells，4 主动腱、20 导向眼（16 活跃），10 g 工具，110.516 g 总结构。','FINAL_C009（完整独立闭环验证）/ C009','FINAL_C009','指定 commit bd4fe89 的 spring_button_press/v1_2；environment.xml、2001-state trajectory.npz、原 Evaluator 和独立验证。','原 C009 全部 11 项 PASS；最大绳速 14.991 mm/s、最小按钮行程 −0.245 μm。此次只复制和展示，不重新优化、积分或评价。','button_travel'),
'm7':('External Impact Active Recovery','在未知方向短脉冲后，由实时状态反馈恢复并保持目标。','原 M2 两段/12 cells/6 主动腱/10 g 负载，原 model.xml 字节不变。','no_impulse / positive_y / negative_y / positive_y_no_feedback / negative_y_no_feedback','positive_y','M7_external_impact_active_recovery/model.xml、五工况完整 0.5 ms CSV/NPZ、100 个 10 ms 更新、完整 COM 外力/冲量记录。','无冲击及双向主动恢复全部 PASS；冻结输入基线均 FAIL。7 组候选、40000 步、一次独立五工况闭环验证。','')}
def main():
    for task,(title,request,structure,cases,case,source,status,figure) in TASKS.items():
        p=ROOT/'visualization/tasks'/task;p.mkdir(parents=True,exist_ok=True)
        text=f'''# {task.upper()} · {title}

自然语言任务：{request}

机器人：{structure}

物理模型和状态来源：{source} 原 XML、物理属性及原科学结果冻结；此入口只恢复保存状态并执行 mj_forward，不做历史物理积分或重新验收。

可选工况：`{cases}`。本轮只对代表 `{case}` 检查初态/中间态/末态三张原生 MuJoCo 图像。

结果和限制：{status}

依赖统一为 Python 3.11、MuJoCo 3.13.0、NumPy 2.4.6；曲线使用 Matplotlib，MP4 使用 imageio/FFmpeg。新增依赖见根目录 requirements_visualization_v2.txt，原版本/运行配置保留在原数据目录。

从数据根目录运行：

```powershell
python -B -m visualization.mujoco_renderer {task} --case {case} --speed {'.2' if task not in ['p3','phs2d','spring_button'] else '1'} --loop
python -B -m visualization.video_export {task} --case {case} --speed {'.2' if task not in ['p3','phs2d','spring_button'] else '1'} --output visualization/results/videos/user_{task}.mp4
```

`--root <数据目录>` 可显式选择数据根。Space 暂停，R 重播，T 切换 Technical View，1/2/3 切换相机。MP4 显示真实物理时间及倍率；逐帧 source index CSV 保证只选择真实保存状态，不插值、也不提前显示未来实际轨迹。

外壳、保护套、基座罩与末端过渡仅为 **Presentation-only visualization**，使用原生 MjvScene DECOR 几何，不参与质量、完整惯量、碰撞、腱路或控制。普通展示隐藏原腱线/内部导向；目标、实际负载和任务环境保留。外观没有增加已验证的物理外壳，不改变验收身份。
'''
        if figure:text+=f'\n本轮独立核心图：[PNG](../../results/{task}/{figure}.png) / [SVG](../../results/{task}/{figure}.svg)。初态/中态/末态图也在同目录，未拼接。\n'
        else:text+='\n完整 M7 数据、性能和图表见根目录 [M7 README](../../../M7_external_impact_active_recovery/README.md)。\n'
        (p/'README.md').write_text(text,encoding='utf8')
    print('11 additive task presentation READMEs; original READMEs unchanged.')
if __name__=='__main__':main()
