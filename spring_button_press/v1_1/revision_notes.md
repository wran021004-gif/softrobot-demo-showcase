# Spring Button Press v1.1 revision and pre-search audit

Codex-assisted engineering design; not Main autonomous Study

Base commit: `7c2fdcce92bfc56ecf8f97dfb5ac04cc18cf4712`.
All v1.0 task/environment/evaluation files, code, candidates and results are
read-only inputs. This directory is a robot-guide/controller/design-space
revision of the same task; no button, tool, contact, timestep, window or
acceptance threshold changes are allowed. The original evaluator is imported
directly, including the 1 micrometre lower-stop tolerance.

## What the stored v1.0 data actually show

`audit_v10.py` reads the complete B4/B5 histories without integration. Its exact
numeric evidence is in `audit_v10.json`. X dominates Hold XZ error: B4 X ranges
-4.569 to -4.305 mm, versus Z -1.706 to +0.898 mm; B5 X ranges -3.298 to
-3.206 mm, versus Z -0.241 to +0.489 mm. The mean X loss decomposes exactly as:

| Component | B4 | B5 |
|---|---:|---:|
| Payload centerline shortening | -1.391 mm | -0.849 mm |
| Rotation of 14 mm axial tool offset | -0.559 mm | -0.315 mm |
| Rotation of 21 mm side-probe offset | -2.575 mm | -2.125 mm |

Thus the fixed side probe contributes most of the X loss when the terminal
frame rotates. The probe geometry may not be changed, so v1.1 explores modest
length compensation and independent Y/Z section dimensions. The ellipse and
tapered rectangle have rotated unequal principal bending stiffnesses; material
stiffness, length split, asymmetric short-tendon angles and span radius all
affect terminal orientation and force allocation. These are model mechanisms,
not causal coefficients established by extra ablation runs. v1.0 B4/B5 jointly
changed multiple parameters, so their differences cannot isolate one variable.

B4's peak is T2 +41.954 mm/s at 1.534 s (Release), while its Approach peak is
24.533 mm/s. B5's peak is T2 -100.414 mm/s at 0.110 s (Approach); its Release
peak is 37.077 mm/s. B4 has a 2.382 N command jump at 0.011 s. B5 has a
2.229 N jump at 0.111 s, immediately after its speed peak, so that single
largest jump cannot be claimed to have caused the preceding peak. The stored
10 ms startup sequence shows large feedback corrections and clipping; B5's
gain 1.5 and velocity correction time 0.075 s, abrupt control changes and lack
of a real rope-speed guard are plausible contributors. The old inverse Hold
nodes have nonzero balance residual and sit at the activation boundary; smooth
interpolation of those nodes alone cannot guarantee the desired contact load.

Both detach near 1.549 s, then cross the lower soft stop: B4 -0.01417 mm at
1.551 s and B5 -0.01936 mm at 1.556 s. This is consistent with early loss of
contact during a fast retreat, leaving an underdamped slider to recoil into its
soft stop. The frozen spring/damping/limit cannot be changed. v1.1 instead
follows the cap smoothly toward zero travel before withdrawing the probe.

## Configuration audit: confirmed defect versus rejected hypothesis

The committed v1.0 runner does **not** load the per-candidate `control.json`.
`use_x=False` is a real unsafe default and the config is not a complete
reproduction contract. However, the stored B4/B5 controls **were actually
generated with X feedback enabled**. Reconstructing commands at eight stored
states with use_x=True matches to <=1.5e-13 N; use_x=False differs by up to
0.569 N for B4 and 2.152 N for B5. No old trajectory was reintegrated to reach
this conclusion. It would be incorrect to relabel their failure as missing X
feedback.

`contact_ff=False` in the old code means reaction compensation is **enabled**;
the name is inverted relative to `reaction_feedforward=True`. Bounded allocation
is unconditional in the final source, so the saved boolean is descriptive, not
an executed switch. B5 also omits its startup duration, and the final source's
startup default differs from the version used for B5. Recorded controls remain
reproducible, but its JSON plus the final controller is insufficient by itself.

v1.1 takes a complete required configuration directly from JSON. XYZ, bounded
allocation and button-state feedback are mandatory and validated. Positive
`reaction_feedforward` semantics select the load-balanced inverse node.
Effective config, actual position weights and sensor sources are recorded for
every run. New control keeps the old inverse-plus-feedback architecture, adds
tangent stiffness, tension-change/rate limits and a native-state rope-speed
prediction. A prediction failure is logged and never treated as passing the
actual 25 mm/s constraint.

## Mechanical guide change and interface limits

The old C1 has routing sites but no physical guide at the base, either midpoint
or distal anchors. Only the junction has a visible 2 g box, and that box renders
its declared holes as solid. Spatial tendons therefore look suspended. v1.1
adds five stations with four independent open eye/spoke assemblies each, and
replaces the junction's solid envelope by a compact central hub. Each active
hole center coincides with the existing routing site; no extra tendon, path
point or topology is inserted. The 1 mm tendon requires a 1.4 mm hole to retain
0.2 mm radial clearance rather than an undersized 0.8 mm hole.

T0/T1 terminate at the first-segment junction in the original topology. Only
T2/T3 continue through the second segment. Distal four-eye stations therefore
have two active holes and two explicitly unused spare holes. Extending T0/T1
through them would violate the required topology and is not done.

Open annulus meshes have real visual holes, with cylindrical radial supports.
Their mass, COM and full inertia are computed from annulus/cylinder volumes,
including the moving parts. The original junction 2 g inertial is retained;
new eye/spoke mass is added. These geoms are non-colliding: ideal routing sites
transmit tendon loads, not collision/friction at a hole. No Bowden, friction,
pulley contact, hardware durability or manufacture validation is claimed.
Technical and Presentation views both retain all four spatial tendons.

The unchanged generator supports independent section Y/Z dimensions, Young
modulus, segment length, each span's radius and phase/offset. It does not expose
a separate radius for the first and second portions of a long tendon span;
v1.1 uses the supported short-span/long-span radii and records that limitation.
No Core internals or generator contracts are modified. The generator allows
2-15 MPa, but no corresponding unchanged-density/viscosity material has been
validated above 8 MPa; this run conservatively retains 4-8 MPa and gains
stiffness through permitted dimensions instead of asserting an unverified
material. All absolute dimensions remain inside original FamilySpace bounds.

The four structure requests, controller tuning domains and candidate budget
are committed before any full v1.1 integration. Any subsequent tuning changes
are per-candidate effective configs within those frozen domains; original task
thresholds and environment remain unchanged.
