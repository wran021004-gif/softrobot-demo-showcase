# M8：未找到可接受穿越解

[完整任务、结构、失败指标及启动命令](../../../M8_narrow_passage_traversal/README.md)。4/6/6 通道三个候选均 FAIL；独立复现最大差 0 仅证明重现失败。初态几何无碰撞，但原零位行程不合格。

```powershell
python -B -m visualization.mujoco_renderer m8 --speed .2 --loop
python -B -m visualization.video_export m8 --raw --speed .2 --output M8_narrow_passage_traversal/results/videos/m8.mp4
```
