# Spring Button Press v1.0 - bounded engineering feasibility attempt

**Codex-assisted engineering design; not Main autonomous Study**

**Outcome: no complete feasible solution was found within the frozen budget.**
Three control-only candidates and six co-design structures were evaluated.
The best co-design candidate B5 was independently replayed once, with exactly
the same qpos history (maximum difference 0). The physical task and evaluation
thresholds were never relaxed. Search has stopped; this task creates no Main
autonomous Study success record.

## Frozen task and actual model

Read [task_spec.md](task_spec.md), [task.json](task.json),
[environment.json](environment.json), [design_space.json](design_space.json),
[evaluation.json](evaluation.json) and [budget.json](budget.json). Their freeze
commit is `1581882`; these six specification files remain unchanged thereafter.
`environment.xml` is the loadable C1 baseline with the real button assembly.

The zero-state C1 probe center is [0.2955, 0.0210, 0.1500] m. Button-cap center
is frozen at [0.2955, 0.0305, 0.1500] m for all structures, creating a 5 mm
surface gap. The cap/rod assembly is 5 g, slides along +Y, has stiffness
50 N/m, damping 0.20 N s/m and a 3 mm soft-limited maximum stroke. It starts
at zero travel and velocity without precompression. Its COM relative to cap
center is [0, 0.0023076923, 0] m and diagonal COM inertia is
[1.136612426e-7, 6.538461538e-8, 1.136612426e-7] kg m2. The cap is 12 mm
diameter by 3 mm thick; the rod is 4 mm diameter by 12 mm long. A fixed rail
housing and mounting plate surround an open slider passage.

The common 10 g tool is a physical base block, side bracket and 3 mm-radius
spherical probe. Its local COM is [0.0089136213, 0.0039243001, 0] m and COM
inertia tensor (kg m2) is:

```text
[[ 5.831808953e-7, -1.996047657e-7, 0],
 [-1.996047657e-7,  2.629155789e-7, 0],
 [ 0,              0,             7.604437300e-7]]
```

All masses, COMs and inertias are calculated from tangent/non-overlapping
uniform-density primitives. They are explicit physical inertials, not massless
markers. The same tool is installed in every structure. Structural mass tables
exclude fixed environment housing and include the physical tool.

MuJoCo 3.13 performs native sphere/cylinder convex **normal contact** with
condim=1 and zero friction. At every 1 ms state, the evaluator reads the named
button slide joint q/qdot and calls `mj_contactForce` for the exact probe-cap
pair. It does not infer contact from spring force. The full Y force balance
also checks mass times slide acceleration against projected contact force,
spring and damping. No xfrc_applied or qfrc_applied is injected. During all
integrated runs qpos/qvel evolve only through `mj_step`; only tendon controls
change. Robot self-contact and unintended environment contacts are enabled and
evaluated. Initial joint angles/velocities are zero and tension is 0.20 N.

## Design and optimization

The fixed C1 baseline preserves 158.5 + 109 mm segments, original material,
sections, four-tendon topology and 12 cells/segment. Each tendon has one
independent active channel. Structure generation uses the unchanged Core
FamilySpace generator and compiler. Supported searched variables are segment
length, Young modulus, original section dimensions and routing radius, within
the frozen ranges. Density, material viscosity, topology, discretization and
actuator limits remain fixed. Six compact requests are listed in
`design_space.json` and in each candidate directory.

The small task adapter uses SciPy bounded least squares, as the existing
Showcase engineering solver does, for three algebraic inverse-state
initializers on the assembled model, followed by bounded tension allocation
and position/velocity feedback at 10 ms. Control candidates A1-A3 progressively
test the feedforward/feedback refinements; co-design structures each receive
their own inverse initializer and one complete control evaluation. The fixed
structure receives three control evaluations; each co-design structure receives
one. This is a bounded engineering comparison, not an equal-effort statistical
ablation or an exhaustive optimum for any structure.

Several inverse initializers stop on the non-smooth contact activation boundary
with nonzero equilibrium residuals. These are not feasible solution evidence:
their costs/residuals and mj_forward counts are retained in optimization JSON,
and only complete contact rollouts determine acceptance. Two additional
algebraic seed attempts for the fixed structure are also retained; neither was
accepted as a physical trajectory. Their reaction-force terms are mathematical
balance calculations, never applied engine forces.

Core `EnvironmentSpec` implements Plane and Window, without an executed spring
button. `NativeModel` requires the original complete hinge state and explicitly
rejects contacts/constraints; its existing optimization entry cannot accept
this task unchanged. `environment.py` and `evaluator.py` are a minimal
task-local adapter; no generic environment framework, Core branch or Core-main
modification was needed. Core provenance is recorded in
[results/budget_usage.json](results/budget_usage.json).

## Results and limitations

See [the full comparison](results/comparison.md) and
[all candidate results](results/all_candidates.json). The best candidate in
each group is selected by the fewest failed hard checks, not by position alone.

| Metric | Control-only A1 | Co-design B5 |
|---|---:|---:|
| Overall | FAIL | FAIL |
| Max Hold force error | 0.09525 N | 0.01090 N |
| Max Hold travel error | 1.905 mm | 0.216 mm |
| Valid simultaneous Hold | 0 s | 0 s |
| Max tendon tension | 3.292 N | 5.218 N |
| Max rope speed | 16.12 mm/s | 100.41 mm/s |
| Max stroke from initial | 1.868 mm | 4.423 mm |
| Robot mass with tool | 67.32 g | 76.15 g |
| Total tendon length variation | 13.10 mm | 31.58 mm |
| Max Hold XZ center error | 1.804 mm | 3.334 mm |

B5 has 145 + 122.5 mm segments, 8 MPa in both segments, original sections
scaled by 1.10 and routing radius 21 mm. It physically establishes contact at
0.594 s and has Hold travel 1.7842-1.7882 mm, normal force 0.08910-0.08939 N.
The depth/force ranges individually hold throughout the 0.5 s Hold window, but
XZ center error exceeds 2 mm throughout, so the **simultaneous valid duration
is zero**. Peak rope speed 100.41 mm/s occurs at 0.110 s, exceeding 25 mm/s.
The slider briefly crosses its soft lower stop to -0.01936 mm during release,
which fails the frozen 1 micrometre negative-travel tolerance. Peak positive
travel is only 1.7882 mm, below the 3 mm upper stop. These separate causes are
preserved rather than hidden behind a position-only success claim.

At 1.8 and 2.0 s B5 is detached, contact force is zero and button travel is
effectively zero; probe speeds are 1.623 and 0.578 mm/s. It passes the release
endpoint check and has no unintended collisions. Its Hold contact-versus-
spring/damping difference is <=0.000236 N; the projected dynamic balance
residual is <=2.3e-14 N. Thus contact is real, but task feasibility is incomplete.
The fixed C1 A1 has inadequate pressing and excessive probe speed at 1.8 s.
Co-design improves force/depth for B5 but worsens mass, tension and motion cost;
**no overall advantage or fundamental impossibility is established**. Better
controllers or different candidates outside this run's budget remain untested.

Ideal-tension actuation is the existing Family backend abstraction. Stroke and
rope speed are measured from actual tendon geometry, with analytical tendon
velocity at every physical step. They are drive constraints/cost proxies, not
a spool dynamics model or electrical power/energy calculation.

Budget usage: 2 prechecks + 3 control candidates + 6 structures + 1 final
independent replay = 12 integrated backend trajectories, 10 full-task
evaluations and 20,700 physics steps. Main/Reporter/Provider calls are all zero.
There were no extra seeds, convergence experiments or robustness runs. Algebraic
mj_forward evaluations are separately counted, not concealed as free physics
trajectories. See the recorded usage JSON for exact counts.

## Independent visualization and replay

Every image is independent, following the user's assembly preference. There is
no combined result figure. [Figure index](results/figures/README.md) links each
PDF/SVG/600 dpi PNG. Native MuJoCo render PNGs are 2400 x 1800. Initial,
contact, peak press, Hold, Release and a Hold close-up are separate. Quantitative
curves include depth, true force, tension, lateral error, rope speed and force
crosscheck. ColorBrewer Dark2 (three colorblind-safe classes), neutral gray,
distinct line styles and markers are used consistently. The animation is
[a labelled B5 failure replay](results/button_press_replay.gif), not a success
demonstration. Plot captions are in [results/figure_captions.md](results/figure_captions.md).

From the isolated checkout:

```powershell
cd D:\softrobot-agent\.worktrees\paper-spring-20261111
D:\softrobot-demo-showcase\.venv\Scripts\python.exe spring_button_press\replay.py --loop
# Start with the local probe/button view:
D:\softrobot-demo-showcase\.venv\Scripts\python.exe spring_button_press\replay.py --close --loop
```

Space pauses, R restarts, 1 selects overall view and 2 selects close-up.
Replay restores saved states and calls mj_forward; integration is forbidden.
Regenerate visuals only with `spring_button_press\export_results.py`.

All nine candidates retain metrics, design requests and exact recorded controls.
Only B4/B5 retain full co-design state arrays; discarded structures retain
compact scalar traces and controls. A future user-invoked physical reproduction
can use, for example, `reproduce.py B5 --output D:\softrobot-agent\runs\button_replay`,
with the same Python executable. This is not run automatically because the
authorized bounded experiment is complete. Reproduction requires the recorded
Core commit and the unchanged original Showcase C1 inputs. No original raw
benchmark data are copied into this task.
