# v1.0 versus v1.1 results

Codex-assisted engineering design; not Main autonomous Study

| Metric | v1.0 B4 | v1.0 B5 | v1.1 S4 |
|---|---:|---:|---:|
| Overall status | FAIL | FAIL | FAIL |
| Max Hold depth error (mm) | 0.202993 | 0.215779 | 0.0645984 |
| Max Hold force error (N) | 0.0101222 | 0.0108965 | 0.00291583 |
| Max Hold XZ error (mm) | 4.65428 | 3.33433 | 0.407033 |
| Valid continuous Hold (s) | 0 | 0 | 0.5 |
| Max tension (N) | 3.38283 | 5.2176 | 4.59109 |
| Max actual rope speed (mm/s) | 41.9543 | 100.414 | 27.8321 |
| Max stroke (mm) | 5.60501 | 4.4231 | 3.1807 |
| Total structure mass (g) | 74.3606 | 76.146 | 110.516 |

S4 remains FAIL: 27.832 mm/s exceeds 25 mm/s, and the -0.005531 mm global / -0.003952 mm Release lower-stop excursions exceed the unchanged -0.001 mm tolerance. Its complete 0.5 s Hold and both Release endpoint checks pass.

| Candidate | Status | Failed original checks |
|---|---|---|
| C1_A | FAIL | contact_near_approach_end, depth_and_force_hold_0p4s, button_travel, speed_limit, release, no_unexpected_collision |
| C1_B | FAIL | depth_and_force_hold_0p4s, button_travel, speed_limit, release, no_unexpected_collision |
| S1 | FAIL | depth_and_force_hold_0p4s, button_travel, speed_limit, release, no_unexpected_collision |
| S2 | FAIL | button_travel, speed_limit, release |
| S3 | FAIL | button_travel, speed_limit, release |
| S4 | FAIL | button_travel, speed_limit |
