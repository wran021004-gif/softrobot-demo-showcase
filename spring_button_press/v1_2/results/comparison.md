# 原始 Evaluator 性能对比

| 指标 | v1.0 B5 | v1.1 S4 | v1.2 C009 |
|---|---:|---:|---:|
| 综合状态 | FAIL | FAIL | PASS |
| Hold 最大深度误差 (mm) | 0.215779 | 0.064598 | 0.053202 |
| Hold 最大接触力误差 (N) | 0.010897 | 0.002916 | 0.002504 |
| Hold 最大 XZ 误差 (mm) | 3.334333 | 0.407033 | 0.289389 |
| 连续有效 Hold (s) | 0.000 | 0.500 | 0.500 |
| 最大张力 (N) | 5.217597 | 4.591085 | 4.405626 |
| 最大实际绳速 (mm/s) | 100.413687 | 27.832060 | 14.990759 |
| 最小按钮行程 (μm) | -19.360823 | -5.531048 | -0.245102 |
| 最大腱行程 (mm) | 4.423096 | 3.180696 | 3.286465 |
| 总结构质量 (g) | 76.145972 | 110.516272 | 110.516272 |
| 1.8/2.0 s 释放端点 | PASS | PASS | PASS |

质量是同一结构的恒定物理质量，不是随时间变化的最大值。最大腱行程是实际几何腱长相对初始值的绝对变化。原始模型、轨迹、Evaluator 均未改写。

## 自适应候选记录

| 候选 | 状态 | 失败检查 | 速度峰值 mm/s | 最小 q μm | 决策 |
|---|---|---|---:|---:|---|
| C001 | PASS | - | 14.991 | -0.605663 | Faster support ramp (20 ms, 200 N/s), linear interpolation, event acquisition and return gate. PASS, but no withdrawal gate reached. |
| C002 | FAIL | release | 14.991 | -0.605663 | Relax low-force gate from 1 to 2 mN. Full 5 mm withdrawal excites endpoint speed: release FAIL. |
| C003 | FAIL | button_travel | 14.991 | -1.310795 | Try slower 290 ms return and 0.5 mm withdrawal. Return phase changes recoil unfavorably: lower travel FAIL; gate not reached. |
| C004 | FAIL | release | 14.991 | -0.846807 | Restore 270 ms return, 0.5 mm withdrawal. Lower bound passes, 1.8 s probe speed fails. |
| C005 | FAIL | button_travel | 14.991 | -1.016132 | Reduce withdrawal to 0.2 mm. Endpoint speed passes, lower travel misses by 0.0161 um. |
| C006 | PASS | - | 14.991 | -0.605663 | Keep contact inverse node through withdrawal to avoid changing antagonist self-stress. PASS; late light recontact remains. |
| C007 | PASS | - | 14.991 | 0.000000 | Implicit damping/stiffness ten-sample prediction, guard from t=0, elastic infeasibility penalty; 0.5 mm withdrawal. PASS, zero fallback and no negative q, but gate not reached. |
| C008 | FAIL | button_travel | 14.991 | -1.479633 | Gate at q<=3 um, |qdot|<=0.4 mm/s, Fn<2 mN, after return. Withdrawal occurs but later rebound reaches -1.480 um. |
| C009 | PASS | - | 14.991 | -0.245102 | Replace slow initial quintic retreat with a 0.5 mm exponential retreat (60 ms); retain continuous 10 ms tension interpolation. Full PASS, complete separation, desired speed/q/tension margins. |

严格按题述 Evaluator 排序：C007, C009, C001, C006, C005, C003, C004, C008, C002。C007 在按钮下限裕量上领先，但最终仍轻触按钮，未触发 WITHDRAW。工程交付选择 C009：它同时完成用户要求的实际撤离、零端点接触力及全部原始验收。两种排序/选择公开记录，不改变 Evaluator。

独立闭环 FINAL_C009 再次全部 PASS，qpos/qvel/control/实际绳速/按钮 q/接触力的最大差均为 0。
