# PHS-2D 被动头部稳定任务

机器人顶部 Base 固定于世界 z=0.5 m，在 [0,0,-9.81] m/s² 重力下向下悬垂。
四根被动腱在实验前固定放绳长度，不进行主动控制，不使用电机或主动张力输入。
在中轴指定材料位置 s 沿世界 x 轴施加 +0.10 N 或 -0.10 N 横向力。
机构允许明显弯曲，同时应保持 Tip 的世界 xy 平面位置稳定。

s 是累计柔性材料长度占 0.28 m 的比例，排除刚性板/连接件间隔；Tip 是原 tip_site，
位于 payload 的局部 [0.014,0,0] m。Base、Tip、施力点都不新增世界约束。

六个搜索条件为 s=0.30、0.50、0.70 各 ±0.10 N；四个预留独立验证条件为 s=0.40、0.60 各 ±0.10 N。
十个加载条件均从同一独立无载基准末态 qpos、零 qvel 开始，使用同一装配和长度向量。
equilibrium/ 单独保存原无力基准本身的完整 4 s 状态，其 t=0 也是原始已保存 snapshot。

## 原冻结标准（未修改）

- 受力点世界 x 位移绝对值至少 0.010 m（10 mm）。
- Tip 世界 xy 位移范数不超过 0.005 m（5 mm）。
- 上述 Tip/受力点位移比不超过 0.30。
- 全部原 monitor_s 监测点及 Tip 的世界 y 位移最大值不超过 0.002 m（2 mm）。
- 无载末态横向位置不超过 0.005 m，且原无载稳定检查通过。
- 原尾窗 0.25 s 内：Tip 及监测点速度各不超过 0.001 m/s；
  监测点漂移不超过 0.0005 m；关节速度无穷范数不超过 0.01 rad/s；
  静态广义力残差不超过 1e-05 N·m；腱长超限不超过 5e-05 m。
- 单侧腱约束方向、非负乘子、各腱 8 N 容量、无异常接触及保存姿态的腱路/实体几何检查保持原样。

原 evaluator 比较无载与受载**最后一帧**位移，并在原 10 ms 样本网格检查尾窗稳定性；
逐 timestep 数据用于观察与完整性核验，不引入更改后的连续时间验收标准。
历史评价原件见各工况 original_evaluation.json 以及 evidence/artifacts/。

## 时间和数值设置

时域 4 s，timestep=0.000125 s，共 32,000 次历史积分；原真实末时间有约 2e-12 s 浮点累积误差，予以保留。
力在第 k 步开始按 t=k·dt、a=min(t/0.5,1)、F=F_target·a²(3-2a) 平滑加载。
原 XML integrator 为 implicitfast；其他 XML options 原样保留。
原 PassiveModel 在 XML 编译后另外将 iterations=100、tolerance=1e-10，独立脚本保留这两个运行时设置。
完整原协议、solreflimit/solimplimit、有效网格和采样相位见 task.json。
原协议继承的 cells_per_segment=12 是默认值；此候选实际网格以 effective_discretization 为准：每段 8 cells。

结论身份是 **PHS 开发性独立数值验收通过**，registered_acceptance 原值为 not_registered；
尚未注册正式 TaskAdapter/SELECT，也未验证真机性能。不得将历史通过改称正式 SELECT 或硬件证明。
