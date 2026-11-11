# M7 finite search record

| Group | Purpose | Nominal/pre-hold | Main remaining issue |
|---|---|---|---|
| 1 original_structure | Native full-state LQR + predictive guard | FAIL | Large feedback/constraint conflict; invalid calibration |
| 2 softened_feedback | Lower feedback, guard disabled | Pre-hold PASS | Startup rope 46.210 mm/s |
| 3 bounded_feedback | Re-enable conservative prediction | FAIL | Constraint allocation disrupts support; invalid calibration |
| 4 stroke_aware_feedback | Penalize static rope excursion; linear reference with smooth ramps | FAIL | Startup/stop rope and pre-hold speed |
| 5 balanced_stroke_feedback | Balance maximum rope excursion and penalize rope velocities | Pre-hold PASS | Startup rope 34.787 mm/s; valid amplitude observation 3.802 mm |
| 6 startup_shaped_feedback | Ramp feedback over 100 ms | All active cases PASS | Rope margin only 0.088 mm/s |
| 7 margin_feedback | Ramp feedback over 130 ms | All active cases PASS | Selected: rope peak 21.786 mm/s |

7 candidate groups (5 one-case screens + 2 five-case evaluations), one independent new-process five-case verification, 20 full case integrations / 40,000 physical steps. No further search, no structure change. All failures retained.

The first screen briefly wrote an amplitude suggestion from an unstable pre-hold. It was explicitly invalidated before any further integration and never used as valid calibration. The one effective adjustment is 0.03 -> 0.04 N, based on the valid pre-hold group 5 baseline. The task then remained frozen.
