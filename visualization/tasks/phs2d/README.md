# PHS2D · Passive Horizontal Tip Stabilization

自然语言任务：加载中段时允许结构变形，同时使 Tip 的水平 XY 位移较小。

机器人：9 段×8 cells，144 弯曲 DOF，4 被动腱、0 主动通道，10 g 负载。

物理模型和状态来源：PHS2D_passive_head_stabilization 原 robot.xml、states_timestep.npz；或 Showcase 原状态 display_replay.npz。 原 XML、物理属性及原科学结果冻结；此入口只恢复保存状态并执行 mj_forward，不做历史物理积分或重新验收。

可选工况：`s030/s040/s050/s060/s070_positive/negative，另 equilibrium`。本轮只对代表 `s050_positive` 检查初态/中间态/末态三张原生 MuJoCo 图像。

结果和限制：原分类 development_pass、未注册 SELECT；水平稳定不等于 Z 固定。原记录无卸载过程；保留九段色带和连接边界。

依赖统一为 Python 3.11、MuJoCo 3.13.0、NumPy 2.4.6；曲线使用 Matplotlib，MP4 使用 imageio/FFmpeg。新增依赖见根目录 requirements_visualization_v2.txt，原版本/运行配置保留在原数据目录。

从数据根目录运行：

```powershell
python -B -m visualization.mujoco_renderer phs2d --case s050_positive --speed 1 --loop
python -B -m visualization.video_export phs2d --case s050_positive --speed 1 --output visualization/results/videos/user_phs2d.mp4
```

`--root <数据目录>` 可显式选择数据根。Space 暂停，R 重播，T 切换 Technical View，1/2/3 切换相机。MP4 显示真实物理时间及倍率；逐帧 source index CSV 保证只选择真实保存状态，不插值、也不提前显示未来实际轨迹。

外壳、保护套、基座罩与末端过渡仅为 **Presentation-only visualization**，使用原生 MjvScene DECOR 几何，不参与质量、完整惯量、碰撞、腱路或控制。普通展示隐藏原腱线/内部导向；目标、实际负载和任务环境保留。外观没有增加已验证的物理外壳，不改变验收身份。

本轮独立核心图：[PNG](../../results/phs2d/tip_horizontal_xy.png) / [SVG](../../results/phs2d/tip_horizontal_xy.svg)。初态/中态/末态图也在同目录，未拼接。
