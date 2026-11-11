# Control-only versus co-design

Codex-assisted engineering design; not Main autonomous Study

| Metric | Control-only A1 | Co-design B5 |
|---|---:|---:|
| Task status | FAIL | FAIL |
| Max Hold contact-force error (N) | 0.0952542 | 0.0108965 |
| Max Hold travel error (mm) | 1.90537 | 0.215779 |
| Valid continuous Hold (s) | 0 | 0 |
| Max tendon tension (N) | 3.29188 | 5.2176 |
| Max rope speed (mm/s) | 16.1216 | 100.414 |
| Max stroke from initial (mm) | 1.86804 | 4.4231 |
| Robot structure mass including tool (g) | 67.3201 | 76.146 |
| Total tendon length variation (mm) | 13.0952 | 31.5758 |
| Max Hold lateral XZ error (mm) | 1.80423 | 3.33433 |

Neither group produced a feasible candidate. B5 improves depth/force tracking but uses more mass/tension, has higher rope speed and fails lateral alignment. This is not an overall co-design success.

All candidate outcomes:

| ID | Status | Failed hard checks |
|---|---|---|
| A1 | FAIL | depth_and_force_hold_0p4s, button_travel, release |
| A2 | FAIL | depth_and_force_hold_0p4s, button_travel, speed_limit, release, no_unexpected_collision |
| A3 | FAIL | contact_near_approach_end, depth_and_force_hold_0p4s, button_travel, speed_limit, no_unexpected_collision |
| B1 | FAIL | depth_and_force_hold_0p4s, button_travel, speed_limit, release, no_unexpected_collision |
| B2 | FAIL | depth_and_force_hold_0p4s, button_travel, speed_limit, release, no_unexpected_collision |
| B3 | FAIL | depth_and_force_hold_0p4s, button_travel, speed_limit, release, no_unexpected_collision |
| B4 | FAIL | depth_and_force_hold_0p4s, button_travel, speed_limit, release |
| B5 | FAIL | depth_and_force_hold_0p4s, button_travel, speed_limit |
| B6 | FAIL | depth_and_force_hold_0p4s, button_travel, speed_limit, release, no_unexpected_collision |
