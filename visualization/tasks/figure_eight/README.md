# Figure-eight：原四腱、两周期保存结果

读取原 `figure_eight_tracking/trajectory_full.npz`，4 个独立通道、单段 270 mm、12 cells、2 g 原负载。22 s 保存结果包含两个周期；不与 M4 旧折线 Tracking 混淆。本轮未重新求解或评价。

```powershell
python -B -m visualization.mujoco_renderer figure_eight --speed .2 --loop
```
