# 执行器需求（保存轨迹实测）

行程是每次实验中腱总长的峰峰值；速度来自保存 qpos/qvel 的 MuJoCo 解析腱速度。张力为理想腱张力，不是已标定的电机指令。M1 各实验独立，表格没有把它们拼接。

## m1/central_high

| 通道 | 最大张力 N | 行程 mm | 最大速度 mm/s |
|---|---:|---:|---:|
| tendon_0_direct_tension | 1.0343 | 2.4862 | 13.5866 |
| tendon_1_direct_tension | 3.8783 | 8.4457 | 41.4104 |
| tendon_2_direct_tension | 0.5638 | 6.0792 | 28.9525 |
| tendon_3_direct_tension | 1.8001 | 10.6587 | 104.6024 |
| tendon_4_direct_tension | 0.3562 | 4.0112 | 33.0276 |
| tendon_5_direct_tension | 0.1466 | 7.8858 | 64.6234 |

## m1/positive_high

| 通道 | 最大张力 N | 行程 mm | 最大速度 mm/s |
|---|---:|---:|---:|
| tendon_0_direct_tension | 0.8638 | 1.7711 | 8.6547 |
| tendon_1_direct_tension | 1.6351 | 4.6104 | 32.4229 |
| tendon_2_direct_tension | 0.5169 | 5.0989 | 35.5020 |
| tendon_3_direct_tension | 1.7663 | 12.6165 | 66.6734 |
| tendon_4_direct_tension | 0.5762 | 5.0560 | 19.5234 |
| tendon_5_direct_tension | 0.0354 | 7.6247 | 60.8055 |

## m1/negative_low

| 通道 | 最大张力 N | 行程 mm | 最大速度 mm/s |
|---|---:|---:|---:|
| tendon_0_direct_tension | 0.3154 | 1.8840 | 8.5430 |
| tendon_1_direct_tension | 1.0860 | 3.2610 | 24.4290 |
| tendon_2_direct_tension | 1.2453 | 5.0511 | 25.1021 |
| tendon_3_direct_tension | 1.9926 | 6.4344 | 65.9176 |
| tendon_4_direct_tension | 2.0529 | 9.4146 | 28.3387 |
| tendon_5_direct_tension | 0.0200 | 11.1874 | 53.1315 |

## eight

| 通道 | 最大张力 N | 行程 mm | 最大速度 mm/s |
|---|---:|---:|---:|
| tendon_0_direct_tension | 4.2620 | 3.0649 | 1.5111 |
| tendon_1_direct_tension | 4.2620 | 3.0649 | 1.5111 |
| tendon_2_direct_tension | 5.9027 | 1.2164 | 0.7652 |
| tendon_3_direct_tension | 2.2973 | 1.2164 | 0.7653 |

P3 没有主动执行器；两根被动腱固定放绳长度均为 143.1667 mm。以上需求不含卷筒、摩擦、弹性伸长、导向件质量或电机动态；真机需据此进行驱动和材料标定。
