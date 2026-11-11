# Figure captions and interpretation boundaries

All lettered parts described below are independent files, not combined figures.
The group captions are provided as assembly notes; individual panels retain
short titles without embedded panel letters so they can be arranged freely.

**Figure 1. Three independent reach/hold experiments.** (a-c) Final saved poses
for central_high, positive_high and negative_low, using one camera. (d) Actual
tip paths projected into YZ; crosses denote the three fixed goals. (e) Euclidean
position error from complete timestep records; shading identifies the existing
0.54-0.60 s hold interval. Each experiment starts independently; the figure does
not represent sequential movement through three goals. It does not change the
original error thresholds or demonstrate zero-error reaching.

**Figure 2. Figure-eight tracking.** (a-c) Exact saved poses at 5, 10 and 15 s.
(d) Orange reference and green actual YZ paths. (e) Full-run position error.
(f) All four actual tendon tensions. Curves derive from 22,001 original states
at 1 ms intervals. This is one nominal MuJoCo result with ideal tension
actuation, not a hardware result, energy estimate or robustness certificate.

**Figure 3. Progressive passive stiffness.** (a-d) Exact unloaded, low-load,
high-load and recovery states of plus_x. (e) Full physical force-displacement
history with the original scored plateaus superimposed. (f) Original secant and
interval stiffness values. The original full 160,000-row record is read in
place; transient portions are retained. External forces are the original direct
load inputs, not contact forces. Recovery is supported for P3 only. The plot
does not establish material nonlinearity, continuum convergence or hardware
validation.

**Figure 4. Passive horizontal tip stabilization (PHS-2D).** (a) Complete
unloaded nine-segment structure. (b-c) Final s050 positive and negative load
states with force-point markers and direction arrows. (d-e) Actual-scale local
views covering the load point and tip; each black world-space bar is 20 mm.
Orange centerline and tip marker denote the unloaded reference; green denotes
the loaded state. (f) Tip XY displacement norm. (g) Signed tip Z change.
(h) Load-point XY displacement norm. Solid lines are s050; dashed lines are
s070; teal and purple denote positive and negative loading. Every quantitative
curve and descriptive extremum uses all 32,001 original states per case, not
the 401 display samples. Small XY motion means horizontal stabilization,
not complete three-dimensional fixation. The final s070 Z changes are about
4.31-4.71 mm; transient peaks are larger and are shown. No unloading or recovery
record exists for PHS, so recovery must not be inferred. Local views retain the
original geometry and perspective without deformation amplification.

**Supplementary Figure S1.** Actual-scale s070 positive/negative PHS local
views at the final saved state. The 20 mm world-space bars use the same
convention as Figure 4. These are supplementary context, not new physical runs.
