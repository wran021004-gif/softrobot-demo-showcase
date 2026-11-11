# Spring Button Press v1.2：完整可行解 C009

**C009：PASS。独立闭环 FINAL_C009：PASS，全部状态和控制历史逐项完全一致。**

本轮使用 9 个固定 S4 结构控制候选、0 个结构候选、1 次最终独立完整闭环回放，共 10 次完整积分、20,000 个物理步。Provider / Main / Reporter 调用均为 0。已达到推荐的速度、按钮下限和张力裕量，停止搜索。

这是 **Codex-assisted engineering design; not Main autonomous Study**。v1.1 S4 仍为 FAIL；本轮没有更改旧结论。

## 冻结任务与验收

直接读取父目录原始文件：[task_spec.md](../task_spec.md)、[task.json](../task.json)、[environment.json](../environment.json)、[evaluation.json](../evaluation.json)。所有候选调用原始 [evaluator.py](../evaluator.py)，没有本轮 Evaluator 副本或修改版本。

保留 2 段、每段 12 cells、4 腱/4 独立理想张力通道、10 g 物理工具；原始按钮位置、探头几何、5 g 按钮、50 N/m 弹簧、0.20 N·s/m 阻尼、3 mm 最大行程、接触参数、重力和 0.20 N 初始预张紧均不变。1 ms 物理步，10 ms 反馈更新，节点之间仅做开环张力插值。没有外力、状态重置或按钮 qpos/qvel 修正。

## 最终结构：保留 S4

| 参数 | C009 |
|---|---|
| 两段长度 | 149 / 120.5 mm |
| 第一段截面 | 椭圆，半轴 Y=11.685、Z=10.640 mm，原截面角 0.35 rad |
| 第二段根部截面 | 矩形 Y×Z=17.689×15.675 mm，角 −0.25 rad |
| 第二段末端截面 | 矩形 Y×Z=15.162×14.1075 mm，角 +0.15 rad；保留原插值/渐变 |
| 两段 Young's modulus | 8 / 8 MPa |
| 两个腱路 span 半径 | 21 / 21 mm |
| T0 / T1 根部角 | 90° / 255° |
| T2 / T3 根部角 | 40° / 130° |
| 导向眼 | 20 个；16 个实际导向/锚定，4 个备用 |
| 工具质量 | 10.000 g |
| 新增导向质量 | 0.509692 g，其中运动导向 0.407754 g |
| 总结构质量 | 110.516272 g；不计固定按钮/外部安装件 |

角度定义为局部 YZ 平面 atan2(Z,Y)。四根腱实际路由为：

| 腱 | MuJoCo 空间腱的实际 site 顺序 |
|---|---|
| T0 | tendon_0_point_0 → tendon_0_point_1 → tendon_0_point_2；基座→第一段中点→段间锚点 |
| T1 | tendon_1_point_0 → tendon_1_point_1 → tendon_1_point_2；基座→第一段中点→段间锚点 |
| T2 | tendon_2_point_0 → tendon_2_point_1 → tendon_2_point_2 → tendon_2_point_3 → tendon_2_point_4；基座→第一段中点→段间导向→第二段中点→末端锚点 |
| T3 | tendon_3_point_0 → tendon_3_point_1 → tendon_3_point_2 → tendon_3_point_3 → tendon_3_point_4；基座→第一段中点→段间导向→第二段中点→末端锚点 |

完整局部附着坐标及编译后的 site 顺序见 [tendon_paths.json](results/tendon_paths.json)。模型由 v1.1 的原 FamilySpace/导向生成接口重新生成；S4 结构完全匹配，所以允许复用其代数逆解 bank，未复用旧张力轨迹。控制已找到可行解，无需扩大结构空间，也未以减重换取可行性。

## 控制策略

保留 S4 的 XYZ 反馈、按钮状态反馈、正向接触反力平衡逆解和有界张力分配。主要参数为位置增益 0.8、速度时间系数 0.06 s、XYZ 权重 [0.1,1,1]、张力变化惩罚 0.05、前馈跟踪惩罚 10、力增益 0.1、按钮深度增益 0.1、按钮速度系数 0.02 s。

启动：从原始零 qpos/qvel、0.20 N 开始，不预先放到平衡姿态。支持张力在 20 ms 启动窗口建立；节点张力变化限制 200 N/s，采用 `linear_10ms` 插值，实际每毫秒指令最大变化率 200 N/s，避免旧 quintic 的 1.875 倍节点斜率放大。原始硬约束约束的是实际绳速，不是张力变化率。

接触状态机：`FREE_APPROACH → CONTACT_ACQUIRE → CONTACT_FOLLOW → PRESS → HOLD → RETURN_FOLLOW → WITHDRAW`。真实接触发生于 0.426 s，10 ms 反馈节点于 0.430 s 检出；以实际 probe/button_cap pair、`mj_contactForce`、button_slide q/qdot 触发并锁存接触状态，使用 0.12 mm 轻压目标，避免短暂接触抖动把控制器退回自由空间。0.5–0.9 s 按原时序 Press，0.9–1.4 s Hold，保持原时序验收。

回位：1.4 s 后进行 270 ms 回位参考，反馈持续读取 q/qdot/力。实际撤离必须等回位参考结束，并同时满足 |q|<3 μm、|qdot|<0.4 mm/s、Fn<2 mN。C009 在约 1.670 s 通过门限，保留接触逆解节点以避免改变拮抗腱自应力，采用时间常数 60 ms、总距离 0.5 mm 的指数撤离。张力仍每 10 ms 反馈、1 ms 开环线性插值；没有新增反馈频率。最终真实接触于 1.675 s 消失，此后不再接触。

速度约束：以当前质量、阻尼、被动刚度、实际 tendon Jacobian 和实际速度，建立包含隐式阻尼及全部 10 个插值样本的局部仿射预测；限值设为 22 mm/s，从 t=0 启用。硬线性约束不可行时保留速度松弛惩罚（1e8），逐次记录松弛量/优化状态，不无记录丢弃速度目标。**C009 实际没有 fallback、没有正松弛、没有物理绳速超限**。预测是局部近似，最终结论始终来自每 1 ms 的真实 `ten_velocity`。

## 结果与独立验证

完整 B5/S4/C009 性能表及全部候选决策见 [comparison.md](results/comparison.md)。

| 最终指标 | C009 |
|---|---:|
| 综合状态 | PASS，全部 11 项原始检查 |
| Hold 深度最大误差 | 0.053202 mm |
| Hold 接触力最大误差 | 0.002504 N |
| Hold 最大 XZ 误差 | 0.289389 mm |
| 连续有效 Hold | 0.500 s |
| 最大张力 | 4.405626 N |
| 最大实际绳速 | 14.990759 mm/s，T3，0.018 s（负向） |
| 最小按钮行程 | −0.245102 μm，1.699 s |
| 最大几何腱行程 | 3.286465 mm |
| 按钮最大行程 | 2.053202 mm |
| 总腱长变化量 | 14.436916 mm |
| 1.8 s / 2.0 s 接触力 | 0 / 0 N |
| 1.8 s / 2.0 s 探头速度 | 1.114302 / 0.915081 mm/s |
| 1.8 s / 2.0 s 按钮残余行程 | 0.000864 μm / 约 0 |
| 非预期碰撞 / 警告 / 外力 | 全部 0 |

推荐裕量均满足：实际绳速 <22.5 mm/s，最小按钮行程 >−0.5 μm，张力 <7 N。原始硬限始终是 25 mm/s、−1 μm 和 8 N。

最终独立验证是新进程加载 **C009 归档控制器源码**，从原始零状态重新计算每个反馈指令并完整积分；没有把保存的 qpos 或控制序列注入验证。模型 XML 与源候选逐字节一致。原始 Evaluator 再次 PASS，qpos、qvel、control、实际绳速、按钮行程和接触力的最大绝对差全部为 0，见 [replay_check.json](candidates/FINAL_C009/replay_check.json)。GUI/GIF 则是保存状态的展示，不计为新的求解验证。

接触仍存在短暂的单毫秒 pair 抖动，主要在早期接触及回位，状态机持续跟随而不重置；Hold 全窗口没有失去接触。完整事件记录见 [execution_audit.json](candidates/C009/execution_audit.json)。没有宣称接触全程无抖动。

严格按题述原始 Evaluator 排序时 C007 因 qmin=0 领先 C009，但它未触发 WITHDRAW，最终仍轻触按钮。**工程交付选择 C009，因为它还完成明确要求的实际撤离。** 两者均为原始 Evaluator PASS；原始排序、工程选择及理由均保存在 [all_candidates.json](results/all_candidates.json)，未暗改排序或验收。

## 独立图片和完整动画

24 张图片每张单独输出 PNG/SVG，PNG 600 dpi。没有组合面板，四根腱另有各自独立的速度图。使用 ColorBrewer Dark2 三色加灰，并以实线/虚线/点划线/点线重复编码。见 [图片索引](results/figures/README.md)。

完整 0–2 s 回放：[整体 GIF](results/replay_overall.gif)、[接触特写 GIF](results/replay_contact.gif)。关键状态包括初始、首次接触、最大按压、Hold、Release 和最终按钮回位。展示模型仅调整无碰撞显示支架、标记和相机；求解模型没有因此改变。

## 运行命令

在 PowerShell 中：

```powershell
Set-Location 'D:\softrobot-agent\.worktrees\paper-spring-20261111'
$springPython = 'D:\softrobot-demo-showcase\.venv\Scripts\python.exe'
# 保存状态查看：不积分，支持 technical/presentation 和接触特写
& $springPython spring_button_press/v1_2/replay.py --view technical --loop
& $springPython spring_button_press/v1_2/replay.py --view presentation --close --loop
# 只重建图片和汇总，不做物理积分
& $springPython spring_button_press/v1_2/export_figures.py
& $springPython spring_button_press/v1_2/summarize.py
# 如用户之后要求再次独立验证，仍受存储预算约束；名称必须未使用
& $springPython spring_button_press/v1_2/run.py USER_VERIFY --replay C009
```

依赖当前 Showcase Python 环境中的 NumPy、SciPy、MuJoCo 3.13、Matplotlib、Pillow，以及只读 Core `D:\softrobot-agent-core`（commit `2c83f8b7bd654328dd7648f844295c54a7c15223`）。本轮基于 Showcase commit `16ce8a663aef30f1ef25d7801f4f7b948722d1db`，新分支 [codex/button-control-autosearch-v1-2-20261111](https://github.com/wran021004-gif/softrobot-demo-showcase/tree/codex/button-control-autosearch-v1-2-20261111)，仓库保持 Public。

## 归档与适用边界

每个候选保留 design、XML、effective_control、runtime_configuration、控制器和 runner 源码快照、optimization、inverse bank、完整 trajectory、result、200 次控制节点日志及执行审计。`effective_control_resolved.json` 展开源码中的默认值；`trajectory.reference_m` 是名义时序参考，事件实际参考在 `controller_trace.json`。预测描述的早期摘要继承问题由新增审计明确更正；原始源码、控制和状态历史不改写。

本结论适用于当前有限元替代的离散柔性链、MuJoCo 原生接触、理想独立张力执行器和无摩擦空间腱导向模型。20 个眼/支撑有显式质量和惯量，但不模拟绳眼摩擦、绳弹性、卷线器、电机带宽或硬件传感延迟；没有声称硬件已验证、不同扰动下鲁棒通过或已完成材料标定。
