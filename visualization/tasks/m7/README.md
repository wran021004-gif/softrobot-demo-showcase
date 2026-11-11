# M7 · External Impact Active Recovery

自然语言任务：在未知方向短脉冲后，由实时状态反馈恢复并保持目标。

机器人：原 M2 两段/12 cells/6 主动腱/10 g 负载，原 model.xml 字节不变。

物理模型和状态来源：M7_external_impact_active_recovery/model.xml、五工况完整 0.5 ms CSV/NPZ、100 个 10 ms 更新、完整 COM 外力/冲量记录。 原 XML、物理属性及原科学结果冻结；此入口只恢复保存状态并执行 mj_forward，不做历史物理积分或重新验收。

可选工况：`no_impulse / positive_y / negative_y / positive_y_no_feedback / negative_y_no_feedback`。本轮只对代表 `positive_y` 检查初态/中间态/末态三张原生 MuJoCo 图像。

结果和限制：无冲击及双向主动恢复全部 PASS；冻结输入基线均 FAIL。7 组候选、40000 步、一次独立五工况闭环验证。

依赖统一为 Python 3.11、MuJoCo 3.13.0、NumPy 2.4.6；曲线使用 Matplotlib，MP4 使用 imageio/FFmpeg。新增依赖见根目录 requirements_visualization_v2.txt，原版本/运行配置保留在原数据目录。

从数据根目录运行：

```powershell
python -B -m visualization.mujoco_renderer m7 --case positive_y --speed .2 --loop
python -B -m visualization.video_export m7 --case positive_y --speed .2 --output visualization/results/videos/user_m7.mp4
```

`--root <数据目录>` 可显式选择数据根。Space 暂停，R 重播，T 切换 Technical View，1/2/3 切换相机。MP4 显示真实物理时间及倍率；逐帧 source index CSV 保证只选择真实保存状态，不插值、也不提前显示未来实际轨迹。

外壳、保护套、基座罩与末端过渡仅为 **Presentation-only visualization**，使用原生 MjvScene DECOR 几何，不参与质量、完整惯量、碰撞、腱路或控制。普通展示隐藏原腱线/内部导向；目标、实际负载和任务环境保留。外观没有增加已验证的物理外壳，不改变验收身份。

完整 M7 数据、性能和图表见根目录 [M7 README](../../../M7_external_impact_active_recovery/README.md)。
