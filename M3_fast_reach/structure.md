# M3 结构信息

- segment_0：长度 0.16 m；Young 模量 7200000.0 Pa；12 cells；截面站点 s=[0.0]。
- segment_1：长度 0.11 m；Young 模量 5400000.0 Pa；12 cells；截面站点 s=[0.0, 1.0]。

structure.json.design 保留原候选的段、材料、截面种类/尺寸/方向/站点、连接件、导向孔、腱路和末端定义。
站点 s 为所属柔性段归一化轴向位置；position_m 为 attachment 所在坐标系的局部偏移。
resolved_tendons 明确每个离散路径点所属 MuJoCo 刚体、局部坐标、原 attachment 和角色 start/guide/anchor。
六根腱都从 fixed_base 的安装点出发。resolved_physics 保存每个 cell 的几何、质心、质量、惯量、刚度和阻尼。
质量 kg，长度 m，角度 rad，Young 模量 Pa，惯量 kg·m²，关节刚度 N·m/rad，阻尼 N·m·s/rad。
Base 位姿是 world 中的 position_m 与 wxyz 四元数；tip_site 局部位置以实际所属刚体为准。
工具有向轴以 tip_site 局部坐标定义；无方向要求的任务也输出局部 +X 的世界方向用于显示。
joint_map.json 按原 physics.dofs 顺序映射全部 qpos/qvel 地址和 CSV 列；actuator_order 是 XML 的理想张力通道。
design 中历史 displacement actuator 元数据不用于本次成功回放，也不是硬件驱动映射。
设计、配置、离散化及 XML 身份均列在结构与各条件 provenance.json 中。

这是每段 12-cell、每 cell 两个弯曲铰链的离散化理想张力驱动模型，不是制造 CAD。六根腱是直线分段、无摩擦的理想路径；材料、惯性、重力及接触参数以 XML 和 structure.json 为准。未验证真实摩擦、制造公差、驱动动态、硬件安全或跨机器鲁棒性。控制 CSV 的单位为 N，不是已验证可直接下发真机的电机命令。本包不提供电控、通信、驱动器或电机转角映射。
