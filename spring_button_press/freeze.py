"""One-time task authoring. Refuses to overwrite an existing frozen task."""
from environment import HERE,write,read,LABEL

if __name__=='__main__':
    if (HERE/'task.json').exists():raise SystemExit('Task already frozen; do not regenerate')
    version='spring_button_press_v1.0'
    common={'version':version,'label':LABEL}
    write(HERE/'task.json',{**common,'name':'Spring Button Press','duration_s':2.,
        'physics_dt_s':.001,'control_dt_s':.010,'initial_robot_qpos':'zero joint angles',
        'initial_qvel':'all zero','initial_tension_n':.2,'initial_button_q_m':0,
        'phases_s':{'Approach':[0,.5],'Press':[.5,.9],'Hold':[.9,1.4],'Release':[1.4,1.8],'Settle':[1.8,2]},
        'target_travel_m':.002,'trajectory':'quintic smoothstep position reference; bounded ideal-tension actuation',
        'environment':'environment.json','evaluation':'evaluation.json','design_space':'design_space.json'})
    write(HERE/'evaluation.json',{**common,'contact_pair':['probe','button_cap'],
        'contact_evidence':'mj_contactForce normal component AND slide q > 0.1 mm',
        'contact_time_window_s':[.35,.60],'hold_window_s':[.9,1.4],
        'required_continuous_hold_s':.4,'hold_depth_m':[.00175,.00225],
        'hold_normal_contact_force_n':[.08,.12],'lateral_xz_error_max_m':.002,
        'max_button_travel_m':.003,'button_negative_tolerance_m':1e-6,
        'actuator_tension_n':[.2,8],'actuator_stroke_from_initial_m':[-.015,.015],
        'rope_speed_limit_m_s':.025,'release_check_times_s':[1.8,2.0],
        'release_residual_travel_max_m':.0001,'release_normal_force_max_n':.01,
        'release_probe_speed_max_m_s':.010,'unexpected_penetrating_contact_allowed':False,
        'collision_check':'native self/environment contact enabled; only probe-cap is allowed',
        'force_crosscheck':'contact normal separate from k*q and c*qdot; additionally project contact force onto Y and check m*qacc balance',
        'actuation_scope':'Core ideal-tension family, actual tendon geometric stroke and analytical speed; no spool or energy model',
        'all_hard_checks_required':True})
    candidates=[
        {'id':'B1','lengths_m':[.1585,.109],'young_pa':[6.4e6,4.8e6],'section_scale':1.,'tendon_radius_m':.018},
        {'id':'B2','lengths_m':[.1585,.109],'young_pa':[8e6,6.8e6],'section_scale':.95,'tendon_radius_m':.020},
        {'id':'B3','lengths_m':[.150,.1175],'young_pa':[7.2e6,5.4e6],'section_scale':1.,'tendon_radius_m':.018},
        {'id':'B4','lengths_m':[.170,.0975],'young_pa':[8e6,6.5e6],'section_scale':1.05,'tendon_radius_m':.020},
        {'id':'B5','lengths_m':[.145,.1225],'young_pa':[8e6,8e6],'section_scale':1.1,'tendon_radius_m':.021},
        {'id':'B6','lengths_m':[.175,.0925],'young_pa':[8e6,8e6],'section_scale':.9,'tendon_radius_m':.021}]
    write(HERE/'design_space.json',{**common,'family':'C1 serial_family_space_v2; generator contract validated',
        'segments':2,'tendons':4,'independent_actuators':4,'cells_per_segment':12,
        'topology':'original C1 spans and angular offsets, fixed guides/anchor stations',
        'segment_length_bounds_m':[[.135,.180],[.090,.125]],'total_flexible_length_bounds_m':[.230,.300],
        'young_bounds_pa':[3e6,8e6],'section_scale_bounds':[.85,1.15],
        'tendon_radius_bounds_m':[.015,.021],
        'fixed':['density','material viscosity','tool geometry/mass/COM/inertia','actuator limits','button world position and physics','evaluation windows'],
        'supported_variables':['segment length','Young modulus','original section dimensions','span radius'],
        'not_searched':'topology, actuator count, section kind, density, damping, cells',
        'baseline':'original C1 geometry/material/routing with common 10 g physical tool',
        'candidate_plan':candidates,'objective_order':['all hard constraints','peak normalized tension','total tendon length variation','structure mass'],
        'cost_scope':'tendon variation and tension are drive cost proxies; not electric energy'})
    write(HERE/'budget.json',{**common,'provider_calls':0,'main_calls':0,'reporter_calls':0,
        'prechecks_max':2,'control_candidates_max':3,'structure_candidates_max':6,
        'final_independent_full_replay_max':1,'full_task_evaluations_max':12,
        'backend_physical_rollouts_max':30,'tool_calls_max':100,'recommended_walltime_minutes':60,
        'counting':'Each integrated trajectory is one backend physical rollout; every full 2 s rollout is also a full task evaluation. Algebraic mj_forward calls are reported separately, as are WP1 read-only restores.',
        'control_plan':[{'id':'A1','feedback_gain':1.},{'id':'A2','feedback_gain':.6},{'id':'A3','feedback_gain':1.5}],
        'stop_rule':'No feasibility within budget: retain best failed candidate and reasons; never relax frozen task'})
    text='''# Spring Button Press - frozen task v1.0

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
'''
    (HERE/'task_spec.md').write_text(text,encoding='utf8')
    print('Wrote frozen specifications; commit before starting full search.')
