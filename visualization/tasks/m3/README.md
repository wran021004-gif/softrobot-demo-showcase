# M3 · Fast Reach / development draft

自然语言任务：在原短时截止时间内到达目标，原最后 50 ms 检查位置与速度。

机器人：原两段各 12 cells、6 主动腱、10 g 负载。

物理模型和状态来源：M3/robot.xml 或 Showcase M3_fast_reach/robot.xml；trajectory_timestep.csv、reference.csv。 原 XML、物理属性及原科学结果冻结；此入口只恢复保存状态并执行 mj_forward，不做历史物理积分或重新验收。

可选工况：`default`。本轮只对代表 `default` 检查初态/中间态/末态三张原生 MuJoCo 图像。

结果和限制：仍为 development draft。原最后 50 ms 通过不等于长期稳定保持或正式批准 Benchmark。

依赖统一为 Python 3.11、MuJoCo 3.13.0、NumPy 2.4.6；曲线使用 Matplotlib，MP4 使用 imageio/FFmpeg。新增依赖见根目录 requirements_visualization_v2.txt，原版本/运行配置保留在原数据目录。

从数据根目录运行：

```powershell
python -B -m visualization.mujoco_renderer m3 --case default --speed .2 --loop
python -B -m visualization.video_export m3 --case default --speed .2 --output visualization/results/videos/user_m3.mp4
```

`--root <数据目录>` 可显式选择数据根。Space 暂停，R 重播，T 切换 Technical View，1/2/3 切换相机。MP4 显示真实物理时间及倍率；逐帧 source index CSV 保证只选择真实保存状态，不插值、也不提前显示未来实际轨迹。

外壳、保护套、基座罩与末端过渡仅为 **Presentation-only visualization**，使用原生 MjvScene DECOR 几何，不参与质量、完整惯量、碰撞、腱路或控制。普通展示隐藏原腱线/内部导向；目标、实际负载和任务环境保留。外观没有增加已验证的物理外壳，不改变验收身份。

本轮独立核心图：[PNG](../../results/m3/position_error.png) / [SVG](../../results/m3/position_error.svg)。初态/中态/末态图也在同目录，未拼接。
