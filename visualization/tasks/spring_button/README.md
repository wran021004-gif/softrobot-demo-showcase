# SPRING_BUTTON · Spring Button Press / C009

自然语言任务：真实探头接触按钮，按压并保持，再跟随回零和撤离。

机器人：149/120.5 mm 两段，各 12 cells，4 主动腱、20 导向眼（16 活跃），10 g 工具，110.516 g 总结构。

物理模型和状态来源：指定 commit bd4fe89 的 spring_button_press/v1_2；environment.xml、2001-state trajectory.npz、原 Evaluator 和独立验证。 原 XML、物理属性及原科学结果冻结；此入口只恢复保存状态并执行 mj_forward，不做历史物理积分或重新验收。

可选工况：`FINAL_C009（完整独立闭环验证）/ C009`。本轮只对代表 `FINAL_C009` 检查初态/中间态/末态三张原生 MuJoCo 图像。

结果和限制：原 C009 全部 11 项 PASS；最大绳速 14.991 mm/s、最小按钮行程 −0.245 μm。此次只复制和展示，不重新优化、积分或评价。

依赖统一为 Python 3.11、MuJoCo 3.13.0、NumPy 2.4.6；曲线使用 Matplotlib，MP4 使用 imageio/FFmpeg。新增依赖见根目录 requirements_visualization_v2.txt，原版本/运行配置保留在原数据目录。

从数据根目录运行：

```powershell
python -B -m visualization.mujoco_renderer spring_button --case FINAL_C009 --speed 1 --loop
python -B -m visualization.video_export spring_button --case FINAL_C009 --speed 1 --output visualization/results/videos/user_spring_button.mp4
```

`--root <数据目录>` 可显式选择数据根。Space 暂停，R 重播，T 切换 Technical View，1/2/3 切换相机。MP4 显示真实物理时间及倍率；逐帧 source index CSV 保证只选择真实保存状态，不插值、也不提前显示未来实际轨迹。

外壳、保护套、基座罩与末端过渡仅为 **Presentation-only visualization**，使用原生 MjvScene DECOR 几何，不参与质量、完整惯量、碰撞、腱路或控制。普通展示隐藏原腱线/内部导向；目标、实际负载和任务环境保留。外观没有增加已验证的物理外壳，不改变验收身份。

本轮独立核心图：[PNG](../../results/spring_button/button_travel.png) / [SVG](../../results/spring_button/button_travel.svg)。初态/中态/末态图也在同目录，未拼接。
