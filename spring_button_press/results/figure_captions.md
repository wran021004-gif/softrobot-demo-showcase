# Independent figure captions

Codex-assisted engineering design; not Main autonomous Study

All figures depict **B5, an overall failed candidate**. The six pose views are
exact saved-state renderings, not reintegrated trajectories or altered shapes.
Each is exported independently as PDF/SVG/PNG; no multipanel composition is
delivered.

- **initial:** Complete arm, fixed base and side button at 0 s, before contact.
- **contact:** First detected probe-cap contact at 0.594 s, shown locally.
- **maximum_press:** Saved state at peak positive button travel, 0.919 s.
- **hold:** Complete structure at 1.200 s.
- **hold_close:** Local 1.200 s view of actual probe-cap contact. Black marks
  the actual contact position; orange is the tool, purple the moving cap,
  gray the fixed housing, teal the arm. Tendon colors are supplemented by
  ordered T0-T3 labels in the separate tension chart.
- **release:** Exact 1.600 s state during release, with probe detached.
- **button_travel:** Actual slide joint travel, target 2 mm and required Hold
  band 1.75-2.25 mm. A matching travel alone does not establish task success.
- **contact_force:** Exact named probe-cap contact normal force from
  mj_contactForce, separately from the spring/damping estimate. Required
  Hold band 0.08-0.12 N is shown.
- **tendon_tensions:** Actual ideal-tension actuation in four independent
  channels, with the 8 N upper limit. No motor-power inference is supported.
- **lateral_error:** Actual XZ probe/cap center error; its approximately
  3.33 mm Hold value exceeds the frozen 2 mm bound.
- **rope_speed:** Analytical MuJoCo tendon velocity at every 1 ms step.
  The 100.41 mm/s transient at 0.110 s exceeds 25 mm/s and is not smoothed.
- **force_crosscheck:** Actual contact normal force, k*q and c*qdot plotted
  as distinct quantities. Their agreement in slow Hold supports force balance,
  but only the native contact pair establishes contact evidence.

Every quantitative plot uses all 2,001 saved states from the independent final
replay. Gray shading identifies the frozen 0.9-1.4 s Hold window. B5 has zero
valid simultaneous Hold duration because lateral alignment fails; force/depth
alone should not be described as a successful button-press task. No hardware,
robustness, exact optimum or Main-autonomous-Study success is demonstrated.
