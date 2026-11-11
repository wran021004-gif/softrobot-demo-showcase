# Spring Button Press - frozen task v1.0

Codex-assisted engineering design; not Main autonomous Study

Version: `spring_button_press_v1.0`. Freeze occurs after exactly two documented
assembly prechecks and before any full control-only or structure search.

Design a fixed-base, four-tendon, two-segment continuum arm that approaches a
side-mounted spring button through a real rigid rounded probe. The arm must
press approximately 2 mm, hold both travel and measured normal contact force,
withdraw, and allow the button to return by its own spring and damping.

World X follows the nominal arm axis, Y is lateral, Z vertical. Mount is
[0, 0, 0.15] m. The C1 zero-angle, zero-velocity `mj_forward` probe center is
[0.2955, 0.0210, 0.1500] m. The unpressed button-cap center is
[0.2955, 0.0305, 0.1500] m, so its front face is at Y=0.0290 m and the
3 mm-radius probe has exactly 5 mm initial free gap. These absolute coordinates
are identical in every candidate; candidates cannot reposition the button.

The physical tool is a base block, lateral bracket and tangent 3 mm sphere,
10 g total. Its uniform-density primitive COM and full COM inertia tensor are
calculated in `environment.py` and frozen in `environment.json`. The button
is a 12 mm-diameter, 3 mm-thick cap plus a 4 mm-diameter, 12 mm rod: 5 g total,
with calculated COM inertia. A native slide joint along +Y has stiffness
50 N/m, damping 0.20 N s/m and range [0, 0.003] m; q starts at 0 without
precompression. At 2 mm, spring force is 0.10 N. Contact is native convex
sphere-cylinder normal contact, condim=1, all friction coefficients zero.

The 2 s trajectory uses 1 ms physics integration and 10 ms control updates:
Approach 0-0.5, Press 0.5-0.9, Hold 0.9-1.4, Release 1.4-1.8, Settle 1.8-2.0 s.
Only the 4 tendon controls may change during integration. No applied external
forces or imposed slider state are allowed. Initial joint angles/velocities
are zero; initial tendon pretension is 0.20 N for every candidate.

Real probe-cap contact and >0.1 mm slide travel must be observed. Contact must
occur near the end of approach, defined before search as [0.35, 0.60] s. At
least 0.4 continuous seconds within [0.9, 1.4] must simultaneously have travel
1.75-2.25 mm, true contact normal force 0.08-0.12 N and XZ center error <=2 mm.
Travel may not exceed 3 mm. Tendon tension <=8 N, actual geometric tendon
length change from initial within +/-15 mm, analytical rope speed <=25 mm/s.
At both 1.8 and 2.0 s residual travel <=0.1 mm, true contact force <0.01 N,
probe speed <=10 mm/s. No unexpected penetrating contact, numerical warning
or nonfinite state is allowed. All checks must pass; endpoint position alone
cannot pass the task. Tiny negative slider soft-limit tolerance is 1 micrometre.

The native contact force is computed with `mj_contactForce` on the exact named
pair and separately compared with spring k*q and damping c*qdot. Contact is
never inferred from spring force. Self/environment contact detection is enabled.
The moving rod does not collide with its own guide housing; housing rails form
an open passage for the slider, with a fixed mounting plate behind its travel.

C1 is the fixed structure baseline; only its control is optimized (<=3
candidates). Co-design uses <=6 structures through the actual FamilySpace
generator: 2 segments, 4 tendons, 12 cells/segment, original routing topology.
Bounds and the prespecified six candidate requests are in `design_space.json`.
Density, material viscosity, common tool and environment remain fixed. Cost
prioritizes feasibility, then tension, total tendon variation and mass; no
claim is made about motor electrical energy or guaranteed co-design advantage.

The task-local assembler/evaluator bypasses no contact physics. Core's schema
has no implemented spring button; Core NativeModel rejects contacts/additional
slide state, so a small Showcase adapter calls the unchanged Family compiler,
SciPy bounded optimizer and MuJoCo directly. No generic environment framework
or Core-main change is required. Static inverse calculations are algebraic
initializers on the same assembled contact model, not accepted trajectories.

`task.json`, `environment.json`, `design_space.json`, `evaluation.json` and
`budget.json` are frozen with this file. Once committed they may not change
in response to search outcomes. An infeasible result must be reported as such.
