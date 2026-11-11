# v1.1 independent figure captions and boundaries

Codex-assisted engineering design; not Main autonomous Study

All images depict **S4, an overall FAIL candidate**. Every pose/chart is a
separate file, not part of an exported composite. All quantitative curves use
all 2,001 final independent replay states without smoothing or invented data.

- `initial_technical`: Full initial arm, four visible colored spatial tendons,
  new guide stations, fixed mount, physical tool and fixed side button.
- `initial_presentation`: Same saved physical state with neutral guide hardware;
  tendons remain visible. Display changes do not change the model's inertials.
- `base_guides`: Four independent base exit eyes and actual tendon routing.
- `midpoint_guides`: First-segment midpoint eyes at the actual 1.2 s pose.
- `junction_guides`: T0/T1 anchors plus T2/T3 pass-through eyes at the junction.
- `distal_guides`: T2/T3 distal anchors and two explicitly unused spare eyes.
  No claim is made that the short tendons continue beyond their junction anchor.
- `contact`: First detected native probe-cap contact, 0.399 s.
- `maximum_press`: State at maximum positive slide travel, about 2.065 mm.
- `hold_technical` / `hold_presentation`: Exact 1.200 s state, showing guide
  hardware and the whole arm in the two display modes.
- `hold_close`: Actual probe-cap contact at 1.200 s. Contact position is marked
  black; the physical orange probe and purple moving cap remain geometrically
  unchanged. Contact force comes from the named native contact pair.
- `release`: Exact 1.650 s state, after the contact-following return interval.
- `button_travel`: Full slide travel and the frozen Hold acceptance band.
- `contact_force`: Actual mj_contactForce normal force and frozen force band.
- `lateral_error`: Probe/cap XZ center error and the 2 mm bound. The full 0.5 s
  Hold qualifies simultaneously with force/depth; this alone is not overall PASS.
- `tendon_tensions`: Four actual ideal-tension channels and the 8 N upper limit.
- `rope_speed`: Actual MuJoCo analytical tendon velocity at every 1 ms; the
  T3 +27.832 mm/s startup peak is retained and exceeds the frozen 25 mm/s bound.
- `lower_stop`: A Y-axis close-up around zero, restricted to -7 to +12
  micrometres to expose short negative excursions. The positive Hold travel is
  outside this zoom and remains fully shown in `button_travel`; no amplitude
  is changed. The unchanged lower tolerance is -1 micrometre.
- `probe_speed`: Actual probe speed with the original 1.8/2.0 s endpoint checks.

Open guide meshes and their mass/inertia are in the solved robot. They are
non-colliding ideal guides: this work does not model hole friction, Bowden
dynamics, contact-constrained flexible cables or hardware manufacturability.
The display stand is rerouted only for visibility of the lower base eye; no
physical button or fixed-mount parameter is changed. Technical images label
T0-T3. All curves use colorblind-friendly ColorBrewer Dark2 (three classes),
neutral gray and redundant line patterns. No hardware, robustness, global
optimum or Main-autonomous-Study success is demonstrated.
