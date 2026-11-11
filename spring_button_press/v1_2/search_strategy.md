# v1.2 bounded adaptive search

The task and evaluator are read directly from the frozen parent v1.0 files. Start at commit 16ce8a6, retain S4 geometry first. No providers, Main, Reporter, seeds or unrelated demos.

1. Audit saved S4 data, without integration. Diagnose startup gravity support, simultaneous tendon speeds, prediction/interpolation mismatch and both contact-loss episodes.
2. First candidate: event-latched native contact feedback, near-zero return gate, linear 10 ms interpolation and faster support ramp. Its result determines the next change; no predetermined parameter grid.
3. Improve the prediction to include implicit damping and interpolation if the existing explicit one-step guard is incompatible with the actual motion. Constrain the 10 ms interval, log every fallback/violation, and judge only actual 1 ms tendon speeds.
4. Adapt acquisition and return following based on measured q, qdot, native force and contact. If fixed S4 remains limited after useful control directions, regenerate local FamilySpace structures and their inverse banks.
5. Stop at a credible complete PASS plus independent replay, or the finite budget. Rank by PASS, failed hard checks, normalized violations, speed/lower-travel margins, Hold errors, tension, mass, variation.

Budget: at most 12 fixed-structure controls, 12 structure/control candidates, 2 independent replays; conservatively at most 24 full integrations **including** replays. Reserve before each integration. Save full state/control histories and source snapshots for every evaluation.
