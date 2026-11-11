# P3 · Passive Progressive Stiffness

自然语言任务：在固定被动放绳下承受分级载荷，并观察非线性刚度和卸载回位。

机器人：3 段、每段仅 1 个粗离散 cell，2 被动腱、0 主动通道、10 g payload；固定放绳 143.1667 mm/根。

物理模型和状态来源：P3_passive_progressive_stiffness/cases/*/model.xml、完整 trajectory.csv；或 Showcase 已有原状态显示样本。 原 XML、物理属性及原科学结果冻结；此入口只恢复保存状态并执行 mj_forward，不做历史物理积分或重新验收。

可选工况：`plus_x / minus_x（原附加工况文件仍保留）`。本轮只对代表 `plus_x` 检查初态/中间态/末态三张原生 MuJoCo 图像。

结果和限制：原开发工况 PASS，刚度比约 3.699；没有连续体收敛或真机证明。外壳按三个真实刚体姿态画出，清楚保留段间边界。

依赖统一为 Python 3.11、MuJoCo 3.13.0、NumPy 2.4.6；曲线使用 Matplotlib，MP4 使用 imageio/FFmpeg。新增依赖见根目录 requirements_visualization_v2.txt，原版本/运行配置保留在原数据目录。

从数据根目录运行：

```powershell
python -B -m visualization.mujoco_renderer p3 --case plus_x --speed 1 --loop
python -B -m visualization.video_export p3 --case plus_x --speed 1 --output visualization/results/videos/user_p3.mp4
```

`--root <数据目录>` 可显式选择数据根。Space 暂停，R 重播，T 切换 Technical View，1/2/3 切换相机。MP4 显示真实物理时间及倍率；逐帧 source index CSV 保证只选择真实保存状态，不插值、也不提前显示未来实际轨迹。

外壳、保护套、基座罩与末端过渡仅为 **Presentation-only visualization**，使用原生 MjvScene DECOR 几何，不参与质量、完整惯量、碰撞、腱路或控制。普通展示隐藏原腱线/内部导向；目标、实际负载和任务环境保留。外观没有增加已验证的物理外壳，不改变验收身份。

本轮独立核心图：[PNG](../../results/p3/force_displacement.png) / [SVG](../../results/p3/force_displacement.svg)。初态/中态/末态图也在同目录，未拼接。
