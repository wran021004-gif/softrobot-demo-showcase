# Paper figures from frozen Showcase v2

These are presentation exports only. No controller, physical model, original
trajectory or original acceptance result has been changed. The export process
replaces `mj_step`, `mj_step1` and `mj_step2` with throwing guards and restores
saved qpos/qvel followed by `mj_forward`. Selected restored tool/load-point
positions agree with their saved values within 1e-10 m. This is a rendering check,
not a repeat of the completed benchmark evaluations.

## Deliverables (all panels are separate files)

| Group | Intended placement | Independent PDF / SVG / PNG stems |
|---|---|---|
| M1 independent reach/hold | Main | `main/fig01_m1_reach_hold_a` through `_e` |
| Figure-eight | Main | `main/fig02_figure_eight_a` through `_f` |
| Progressive passive stiffness | Main | `main/fig03_p3_stiffness_a` through `_f` |
| PHS full / close-up / separate displacement axes | Main | `main/fig04_phs2d_a` through `_h` |
| PHS s070 close-ups | Supplementary | `supplementary/figS01_phs2d_s070_a` and `_b` |

Per the user's assembly preference, **no composite figures are delivered**.
Every viewport and every chart is exported separately; suffixes map to the
caption parts. See `file_index.md` for every format, pixel size and title.
Individual PNGs use 600 dpi. Every original MuJoCo raster
under `renders/` is rendered natively at 2400 x 1800, not enlarged. PDF and SVG
retain vector curves, axes and text with embedded high-resolution renderings.
The 3D viewport is perspective: no anisotropic geometry scaling or amplified
deformation is used. The PHS 20 mm bar is a real world-space vertical segment at
the local lookat depth; perspective means its projected length is depth-dependent.

## Cameras and provenance

Full views share azimuth 115 degrees and elevation -18 degrees. M1 and eight use
lookat [0.14, 0, 0.15] m and distance 0.57 m. P3 uses [0, 0, 0.25] m / 0.70 m;
PHS uses [0, 0, 0.29] m / 0.78 m. PHS close-ups use azimuth 100 degrees,
elevation -10 degrees, midpoint of load point and tip as lookat, and distance
2.5 times their separation (minimum 0.20 m). Exact per-render parameters,
timestamps, source paths, sample counts and descriptive extrema are recorded in
`figure_provenance.json`. Appearance-only in-memory settings unify light, ground
and background; the established v2 visual sleeves are enabled throughout.

M1 uses complete timestep CSVs in this repository; eight uses all 22,001 saved
states. P3 reads all 160,000 rows of the original 0.125 ms CSV in place and uses
the original scored plateau stiffness values. PHS reads all 32,001 original
states for each of s050/s070 positive/negative. Its 401 saved display states are
used only to select exact render poses, never for curve statistics. Raw PHS
data remain at `D:\softrobot-benchmark-exports\base_only_accepted_20261028` and
are deliberately not copied. Dense plotted series use display reduction with
global extrema retained; no smoothing, amplitude changes or interpolated
physical data are introduced. Original evaluation windows remain authoritative.

## Reproduce

```powershell
D:\softrobot-demo-showcase\.venv\Scripts\python.exe paper_figures\scripts\export_figures.py
```

Requires the local raw export root above, MuJoCo, NumPy, Matplotlib and Pillow.
No new physics run is needed. See `figure_captions.md` for interpretation limits.
