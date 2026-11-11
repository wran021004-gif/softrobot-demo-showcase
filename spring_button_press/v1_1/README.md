# Spring Button Press v1.1: mechanical routing and bounded engineering results

**Codex-assisted engineering design; not Main autonomous Study**

**Final outcome: FAIL.** S4 meets the complete 0.5 s Hold condition, alignment,
tension, stroke, collision and release-endpoint checks. Two original checks
remain failed: 27.832 mm/s actual rope speed versus 25 mm/s, and brief negative
slider travel beyond the unchanged -1 micrometre lower-stop tolerance. The
single independent replay reproduces every qpos value exactly. Search stopped
after the authorized 2 baseline + 4 co-design + 1 replay evaluations.

This directory is based on commit `7c2fdcce92bfc56ecf8f97dfb5ac04cc18cf4712`.
All v1.0 files, benchmark exports, button/tool physics, contact model, timing,
actuator limits and acceptance criteria are unchanged. `../evaluator.py` is
imported directly. No Core modification, Main, Reporter or Provider was used.

## Audit: what failed in v1.0

[Revision notes](revision_notes.md) and [numeric evidence](audit_v10.json)
contain the read-only audit of B4/B5, including phase-specific rope speed,
command jumps, section anisotropy and an exact tool-offset decomposition.
No v1.0 search or integration was rerun.

The bare routing comes from routing **sites without corresponding hardware**
at the base, segment midpoints and anchors. The only original visible junction
guide was a solid box despite its declared holes. v1.1 adds actual open mesh
eyes and supports instead of hiding the tendons.

B4/B5 Hold errors are dominated by X. Mean X losses (mm) are respectively
centerline shortening -1.391/-0.849, axial tool rotation -0.559/-0.315 and
rotation of the unchanged 21 mm side-probe offset -2.575/-2.125. This motivated
small length compensation and independent section Y/Z dimensions, rather than
moving the button or changing the probe. Original rotated ellipse/rectangle
anisotropy and asymmetric tendon directions couple these motions. The available
comparisons change several parameters together and do not isolate causal
contributions from length, modulus, section or angle individually.

B4's rope peak was **T2 +41.954 mm/s at 1.534 s, during Release**. B5's was
**T2 -100.414 mm/s at 0.110 s, during Approach**. B5's largest command jump
occurs at 0.111 s, after that peak, so it cannot by itself explain the preceding
peak. The startup history nevertheless shows large sampled feedback changes.
Both old candidates detached near 1.549 s and recoiled across the soft stop.

## Configuration findings and corrections

The v1.0 entry did not consume `control.json`. Its `use_x=False` default and
inverted `contact_ff` semantics were genuine configuration hazards. However,
the stored B4/B5 runs **did use XYZ feedback**: reconstructing commands at eight
saved states with use_x=True matches within 1.5e-13 N; disabling X differs by
up to 0.569/2.152 N. This is evidence against the hypothesis that these two
specific failures resulted from disabled X. B5's startup duration was omitted
from its saved JSON, and the final old code used a different startup duration.

v1.1 requires configuration as an explicit Controller argument. XYZ feedback,
bounded allocation and button-state feedback must be true; unsupported false
values are rejected. `reaction_feedforward=True` selects the load-balanced
inverse node; false selects the free inverse node. No negative/inverted switch
is used. Each candidate retains `effective_control.json`, runtime summary,
inverse bank, exact controller and runner source snapshots and the complete
control/state history. This makes the executed version reproducible despite
the bounded development iterations.

For S4 the actual XYZ weights are **[0.1, 1, 1]**, all nonzero. X is controlled
without over-weighting a poorly conditioned axial direction. The controller
reads actual slide q/qdot and the named native contact force; sensor diagnostics
match their recorded physical states exactly. The final policy combines
load-balanced feedforward, position/velocity feedback, button depth/velocity
and force feedback, bounded allocation, tension-target change penalties and
a model-based rope-speed guard.

The first candidates exposed a poor full tangent-stiffness inverse and a 10 ms
acceleration extrapolation that blocked gravity support. Later candidates
return to the original passive hinge stiffness map and use a 1 ms prediction.
The guard is bypassed before 0.15 s for gravity support and has explicit logged
infeasibility fallbacks. It does **not** guarantee the actual 25 mm/s limit:
all 1 ms analytical tendon velocities are still evaluated, and S4 fails that
check honestly.

S4 uses linear lookahead startup target knots and quintic interpolation between
10 ms control updates. Interpolation at 1 ms is open-loop; it adds no extra
feedback updates. The 80 N/s setting limits **target-knot change divided by
10 ms**, not instantaneous interpolated tension derivative; the measured
instantaneous slope is separately recorded. The original actuator constraints
remain tension, geometric stroke and actual rope speed.

An additional diagnostic-only logging defect was found: S2-S4's original
`controller_trace.json` returned a hardcoded [1,1,1] diagnostic vector even
though the allocation used configured [0.1,1,1]. Their source snapshots,
effective configs and runtime summaries demonstrate the executed weights.
Those original traces are preserved; `execution_audit.json` records the
discrepancy and correct weights. The current diagnostic writer is fixed without
changing any physical state, control or result or spending an extra run.

**Comparison limit:** C1_A/C1_B used earlier controller iterations; the final
S4 controller was not rerun on C1 because the two-baseline budget was consumed.
Thus this is a transparent bounded engineering development record, not an
equal-controller causal ablation or proof that structure alone caused gains.

## Mechanical guide design and routing

Each of five stations has four independent annular eyes supported by radial
cylindrical spokes. Hole diameter is 1.4 mm for the actual 1 mm tendon, leaving
0.2 mm nominal radial clearance; outer diameter is 4 mm, axial thickness 1 mm,
spoke radius 0.5 mm and uniform nominal guide density 1200 kg/m3. Mesh faces
form open annuli, not solid plates with hidden tendons passing through them.
The original junction 2 g inertial is retained, with its solid envelope replaced
by a compact hub. No original tendon or routing point is deleted.

| Tendon | Route |
|---|---|
| T0 | Base eye -> first-segment midpoint eye -> first-segment terminal anchor at junction |
| T1 | Base eye -> first-segment midpoint eye -> first-segment terminal anchor at junction |
| T2 | Base eye -> first-segment midpoint eye -> junction pass-through eye -> second-segment midpoint eye -> distal anchor eye |
| T3 | Base eye -> first-segment midpoint eye -> junction pass-through eye -> second-segment midpoint eye -> distal anchor eye |

There are **20 eyes, 16 active and 4 distal spares**. T0/T1 terminate at the
junction under the original topology; their distal spare holes do not imply
that those tendons continue. Moving eye/spoke assemblies have explicit mass,
COM and full COM inertia computed from annulus/cylinder volumes. The load path
is still the ideal frictionless spatial tendon through exact routing sites.
Guide meshes are deliberately non-colliding; no hole friction, Bowden dynamics
or collision-guided rope model is claimed. The bonded support/root regions
are idealized mounting interfaces, not a complete fabrication qualification.

The single assembly precheck found zero initial contacts, positive guide
inertias and <1e-16 m site/hole-center error. Postprocessing 13 selected actual
S4 poses, including peak speed, rebound, Hold, Release and maximum bend, found
<1e-16 m registration error, approximately 0.167 mm minimum finite-bore radial
clearance, 13.36 mm minimum inter-tendon clearance after tendon diameter, and
8.35 mm conservative tendon/flexible-body clearance. These are finite sampled
geometric checks, not a continuous swept or manufacturing guarantee. See
[routing_checks.json](results/routing_checks.json).

Technical and Presentation views both retain all four colored tendons. The
technical view labels T0-T3 and shows open eyes and connections. Presentation
uses neutral hardware while retaining colored tendons. Replay also reroutes
the old non-colliding display stand behind the base plane to reveal the lower
base eye; this changes only display geometry, not the fixed mount or any
physical/contact geometry used for solving.

## Frozen design space and final structure

[design_space.json](design_space.json), [controller.json](controller.json),
[budget.json](budget.json) and revision notes were committed before full
evaluation, at `495281a`. The original task/environment/evaluator remain direct
references. All four structure requests were prespecified. Later controller
implementation corrections are preserved by per-run source snapshots, not
concealed as identical controllers.

The unchanged generator supports separate Y/Z section dimensions, segment
length/modulus and span radii/phases/offsets. It exposes one radius for each
tendon span, rather than separate radii for the two portions of a long tendon.
No unsupported per-segment radius override or Core bypass was used. Although
the Family domain allows 2-15 MPa, this revision conservatively searches 4-8 MPa:
no same-density/viscosity material above 8 MPa was validated. Dimensions remain
inside original 12 mm ellipse semiaxis / 18 mm rectangle dimension limits.

S4 parameters:

- Two segments: **149 / 120.5 mm**, total 269.5 mm; **8 / 8 MPa**.
- First ellipse semiaxes **11.685 / 10.640 mm**, original section angle retained.
- Second rectangle starts at **17.689 x 15.675 mm**, tapers to
  **15.162 x 14.1075 mm**, with original section-angle profile retained.
- Short and long span radii **21 / 21 mm**, 12 cells per segment.
- Short pair uses original phase and a +15 degree bounded offset correction;
  long pair rotates -5 degrees. Root polar angles are 90, 255, 40 and 130 degrees
  for T0-T3. Original short/long span topology remains unchanged.
- Same 10 g physical tool, button world coordinates and contact parameters.

S4 mass accounting:

| Component | Mass |
|---|---:|
| Flexible structure | 95.0066 g |
| Existing rigid connector/junction | 5.0000 g |
| New guide eye/spoke assemblies | 0.50969 g |
| Of new guides, moving with robot | 0.40775 g |
| Common physical tool | 10.0000 g |
| Total structure including tool and base guides | **110.5163 g** |

The C1 guide baseline adds 0.45314 g, including 0.36251 g moving mass, for a
67.7733 g total. Guide mass is not a free visual benefit.

## Results, remaining failures and budget

The full [comparison table](results/comparison.md) includes all six candidates.
The best v1.1 candidate is S4 by the fewest failed original hard checks.

| Metric | v1.0 B4 | v1.0 B5 | v1.1 S4 |
|---|---:|---:|---:|
| Overall | FAIL | FAIL | **FAIL** |
| Max Hold depth error | 0.2030 mm | 0.2158 mm | **0.06460 mm** |
| Max Hold force error | 0.01012 N | 0.01090 N | **0.002916 N** |
| Max Hold XZ error | 4.654 mm | 3.334 mm | **0.4070 mm** |
| Valid continuous Hold | 0 s | 0 s | **0.500 s** |
| Max tension | 3.383 N | 5.218 N | 4.591 N |
| Max actual rope speed | 41.954 mm/s | 100.414 mm/s | **27.832 mm/s** |
| Max geometric stroke | 5.605 mm | 4.423 mm | 3.181 mm |
| Structure mass including tool | 74.361 g | 76.146 g | 110.516 g |

XZ error decreased approximately 91.25% versus B4 and 87.79% versus B5; rope
peak decreased 33.66% and 72.28%. These gains have a substantial mass cost and
include controller changes, so they are not an isolated structure advantage.

S4's entire Hold is 2.0315-2.0646 mm at **0.10173-0.10292 N**, with no
unexpected collision. Contact is native and independently crosschecked against
spring/damping/slider acceleration; max Hold projected dynamic balance residual
is 3.77e-15 N. No applied external forces or imposed slide states were used.

The remaining rope peak is **T3 +27.832 mm/s at 0.060 s**, during startup,
11.33% over 25 mm/s. The global negative slide excursion is **-0.005531 mm
at 0.454 s**, following an early Approach contact/bounce, versus the frozen
-0.001 mm tolerance. Release has a smaller **-0.003952 mm** excursion and also
fails that same lower-stop check. This is not hidden by the positive 3 mm upper
travel limit. At 1.8/2.0 s, contact is zero and travel is effectively zero;
probe speeds **6.218/6.984 mm/s** pass the original 10 mm/s endpoint limit.

Budget: 1 non-integrating assembly precheck, 2 C1 baselines, 4 structures and
1 independent replay, exactly **7 full evaluations / 14,000 integration steps**.
No random seeds, convergence tests, extra short integration rollouts or old
search reruns were used. Algebraic mj_forward work is separately counted in
[budget_usage.json](results/budget_usage.json). The final independent replay's
maximum qpos difference is exactly zero. No further tuning or search follows.

## Images, preview and commands

[Independent figure index](results/figures/README.md) provides separate PDF,
SVG and 600 dpi PNG files. Native pose PNGs are 2400 x 1800. No combined figure
is delivered. Curves use all 2,001 final replay states, with
[three-class ColorBrewer Dark2](https://colorbrewer2.org/#type=qualitative&scheme=Dark2&n=3)
colors plus gray and distinct line styles. See
[captions and limits](results/figure_captions.md). Preview:
[overall technical replay](results/replay_overall.gif) and
[contact presentation replay](results/replay_contact.gif), both labelled FAIL.

```powershell
cd D:\softrobot-agent\.worktrees\paper-spring-20261111
D:\softrobot-demo-showcase\.venv\Scripts\python.exe spring_button_press\v1_1\replay.py --view technical --loop
D:\softrobot-demo-showcase\.venv\Scripts\python.exe spring_button_press\v1_1\replay.py --view presentation --close --loop
```

Space pauses, R restarts, T/P switch display mode, 1 selects overall, 2 contact,
3 base guides and 4 junction guides. The viewer restores saved states and calls
mj_forward only; integration is forbidden. `export_figures.py` regenerates
images without physics. Future user-invoked reproduction (outside this run's
budget) can call `reproduce.py S4 --output D:\softrobot-agent\runs\button_v11`
or add `--closed-loop` to load its exact archived controller/runner and config.
The reproduction command has not been executed as an extra evaluation here.
