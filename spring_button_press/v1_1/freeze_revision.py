"""One-time v1.1 revision freeze. v1.0 files are references, never rewritten."""
import numpy as np
from mechanics import HERE,read,write,LABEL

if __name__=='__main__':
    if (HERE/'design_space.json').exists():raise SystemExit('Revision already frozen')
    fixed={'task_spec':'../task_spec.md','task':'../task.json','environment':'../environment.json','evaluation':'../evaluation.json',
        'freeze_commit':'1581882cc7dd0cc0a3363e00e0b7ef16e7e916de','base_commit':'7c2fdcce92bfc56ecf8f97dfb5ac04cc18cf4712'}
    candidates=[
        {'id':'S1','seed':'B5','lengths_m':[.148,.1225],'young_pa':[8e6,8e6],'section_yz_scale':[[1.1,1.1],[1.1,1.1]],'span_radii_m':[.021,.021],'phase_rotation_rad':[0,0],'short_pair_offset_delta_rad':0},
        {'id':'S2','seed':'B4','lengths_m':[.170,.1005],'young_pa':[8e6,8e6],'section_yz_scale':[[1.1,1.15],[1.25,1.4]],'span_radii_m':[.020,.021],'phase_rotation_rad':[0,0],'short_pair_offset_delta_rad':0},
        {'id':'S3','seed':'B5','lengths_m':[.146,.1225],'young_pa':[8e6,8e6],'section_yz_scale':[[1.2,1.3],[1.3,1.55]],'span_radii_m':[.021,.021],'phase_rotation_rad':[0,0],'short_pair_offset_delta_rad':0},
        {'id':'S4','seed':'B5','lengths_m':[.149,.1205],'young_pa':[8e6,8e6],'section_yz_scale':[[1.23,1.4],[1.33,1.65]],'span_radii_m':[.021,.021],'phase_rotation_rad':[0,float(-np.pi/36)],'short_pair_offset_delta_rad':float(np.pi/12)}]
    write(HERE/'design_space.json',{'revision':'spring_button_press_design_v1.1','label':LABEL,'frozen_task_references':fixed,
        'segments':2,'tendons':4,'independent_actuators':4,'cells_per_segment':12,'same_topology':True,
        'length_bounds_m':[[.145,.170],[.0975,.1225]],'total_flexible_length_m':[.260,.280],
        'young_pa':[4e6,8e6],'family_supported_young_pa':[2e6,15e6],
        'material_limit_reason':'8-10 MPa is generator-supported, but no corresponding same-density/same-viscosity material has been validated; conservatively retain 8 MPa upper bound this round.',
        'independent_section_dimensions':'Y and Z scale per segment, preserving existing kind, stations, taper and section angles; each absolute value is validated by the original FamilySpace domain',
        'family_absolute_dimension_limits_m':{'ellipse_semi_y':[.003,.012],'ellipse_semi_z':[.003,.012],'rectangle_width_y':[.006,.018],'rectangle_height_z':[.006,.018]},
        'section_axis_scale_bounds':[[.9,1.25],[.9,1.5],[.9,1.35],[.9,1.7]],
        'span_radius_bounds_m':[.015,.021],
        'radius_interface_limit':'Generator has one radius per tendon span, not independent radii per arm segment. Short-span radius and long-span radius are independent; long tendons keep their radius through both segments. No guide override or Core bypass is used.',
        'phase_rotation_bounds_rad':[-float(np.pi/12),float(np.pi/12)],'short_offset_delta_bounds_rad':[0,float(np.pi/6)],
        'guide_standard':{'hole_diameter_m':.0014,'outer_diameter_m':.004,'thickness_m':.001,'spoke_radius_m':.0005,'density_kg_m3':1200,
            'stations':['base','segment0 midpoint','junction','segment1 midpoint','segment1 terminal'],
            'eyes_per_station':4,'routing_preserved':'T0/T1 anchor at junction; distal stations have two active eyes and two spare eyes'},
        'mass_accounting':'all 20 added eye/spoke assemblies have explicit annulus/cylinder mass, COM and full COM inertia; existing 2 g guide inertial retained',
        'guide_physics_boundary':'mass/inertia included; non-colliding open meshes; frictionless ideal tendon routing sites rather than hole collision/friction',
        'candidates':candidates,'search_order':['S1','S2','S3','S4'],'ranking':['all original hard checks','fewest failed checks if none feasible','normalized constraint violation','peak tension','tendon variation','structure mass']})
    defaults={'xyz_feedback':True,'button_state_feedback':True,'reaction_feedforward':True,'bounded_allocation':True,
        'position_gain':.4,'velocity_time_s':.03,'startup_s':.06,'tension_rate_limit_n_s':40.,'delta_tension_penalty':.15,
        'predicted_rope_speed_limit_m_s':.022,'release_contact_end_s':1.67,'withdraw_end_s':1.78,
        'force_gain':.25,'button_depth_gain':.3,'button_velocity_time_s':.02}
    write(HERE/'controller.json',{'revision':'controller_v1.1','frozen_task_references':fixed,'defaults':defaults,
        'tuning_bounds':{'position_gain':[.15,.8],'velocity_time_s':[.01,.06],'startup_s':[.04,.12],
            'tension_rate_limit_n_s':[15,80],'delta_tension_penalty':[.05,.5],'force_gain':[.1,.5],'button_depth_gain':[.1,.6],
            'release_contact_end_s':[1.60,1.70],'withdraw_end_s':[1.75,1.79]},
        'implementation':'explicit required config -> Controller; no independent use_x/contact_ff defaults; per-run effective config and runtime sensor summary',
        'reference':'quintic approach/press; controlled contact-following return then withdrawal, within unchanged phase windows',
        'velocity_limit_scope':'predictive guard is a model-based control constraint; only actual analytical ten_velocity at every 1 ms determines acceptance'})
    write(HERE/'budget.json',{'revision':'v1.1','label':LABEL,'provider_calls':0,'main_calls':0,'reporter_calls':0,
        'assembly_prechecks_max':1,'baseline_full_evaluations_max':2,'codesign_full_evaluations_max':4,
        'final_independent_replays_max':1,'total_full_evaluations_max':7,'step_size_s':.001,
        'prohibited':['v1.0 reruns','random seed experiments','step-size convergence','large grids','extra hidden integration rollouts'],
        'algebraic_scope':'inverse equilibrium and state/Jacobian calculations use mj_forward only; not extra integration trajectories'})
    print('Wrote v1.1 design/controller/budget freeze. Commit before full evaluation.')
