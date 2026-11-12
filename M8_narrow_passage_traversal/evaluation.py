"""Every-physics-state physical geometry, contact and passage evaluation."""
import numpy as np
import mujoco
from controller import geometry_ids
def snapshot(m,d,robot,walls):
    best=(float('inf'),-1,-1,np.zeros(6));f=np.zeros(6)
    for r in robot:
        for w in walls:
            distance=mujoco.mj_geomDistance(m,d,r,w,1.,f)
            if distance<best[0]:best=(float(distance),r,w,f.copy())
    contacts=sum((int(c.geom1) in robot and int(c.geom2) in walls) or (int(c.geom2) in robot and int(c.geom1) in walls) for c in d.contact)
    return best,contacts
def crossing(points,x):
    hits=[]
    for a,b in zip(points[:-1],points[1:]):
        if (a[0]-x)*(b[0]-x)<=0 and abs(b[0]-a[0])>1e-12:hits.append(a+(b-a)*(x-a[0])/(b[0]-a[0]))
    return hits
def centerline_checks(points,task):
    tun=task['tunnel'];yc,zc=tun['center_yz_m'];half=np.array([tun['width_m'],tun['height_m']])/2
    inside=lambda p:bool((abs(p[1:]-[yc,zc])<=half+1e-12).all())
    entry=crossing(points,tun['entry_x_m']);exit=crossing(points,tun['exit_x_m']);clipped=[]
    for a,b in zip(points[:-1],points[1:]):
        if abs(b[0]-a[0])<1e-12:
            if tun['entry_x_m']<=a[0]<=tun['exit_x_m']:clipped.extend([a,b])
        else:
            u=sorted([(tun['entry_x_m']-a[0])/(b[0]-a[0]),(tun['exit_x_m']-a[0])/(b[0]-a[0])]);lo=max(0,u[0]);hi=min(1,u[1])
            if lo<=hi:clipped.extend([a+lo*(b-a),a+hi*(b-a)])
    return bool(entry and all(map(inside,entry))),bool(exit and all(map(inside,exit))),bool(clipped and all(map(inside,clipped)))
def evaluate(a,task,m):
    t=a['time_s'];acc=task['acceptance'];tip=a['tip_m'];error=np.linalg.norm(tip-task['target_m'],axis=1);speed=np.linalg.norm(a['tip_velocity_m_s'],axis=1);hold=t>=t[-1]-acc['hold_s']-1e-10
    tn=task['tunnel'];opening=lambda p:bool(abs(p[1]-tn['center_yz_m'][0])<=tn['width_m']/2 and abs(p[2]-tn['center_yz_m'][1])<=tn['height_m']/2)
    crossings=[]
    for plane in [tn['entry_x_m'],tn['exit_x_m']]:
        ids=np.flatnonzero((tip[:-1,0]<plane)&(tip[1:,0]>=plane));crossings.append(next((int(k+1) for k in ids if opening(tip[k+1]) and opening(tip[k])),None))
    enter,exit=crossings;path=a['centerline_checks'];after=np.arange(len(t))>=(enter if enter is not None else len(t))
    rope=float(abs(a['tendon_velocity_m_s']).max());stroke=float(abs(a['tendon_length_m']-a['tendon_length_m'][0]).max());minimum=int(a['clearance_m'].argmin())
    checks={'initial_clearance':bool(a['clearance_m'][0]>=acc['clearance_m']),'initial_tip_before_entrance':bool(tip[0,0]<tn['entry_x_m']),'initial_no_wall_contact':bool(a['wall_contacts'][0]==0),'all_timestep_clearance':bool(a['clearance_m'].min()>=acc['clearance_m']),'zero_wall_contacts':bool(a['wall_contacts'].max()==0),'tip_entry_then_exit_through_openings':bool(enter is not None and exit is not None and enter<exit),'final_centerline_entry_opening':bool(path[-1,0]),'final_centerline_exit_opening':bool(path[-1,1]),'continuous_body_inside_tunnel_after_entry':bool(enter is not None and path[after,2].all()),'final_hold_position_all_steps':bool((error[hold]<=acc['tip_error_m']).all()),'final_hold_speed_all_steps':bool((speed[hold]<=acc['tip_speed_m_s']).all()),'tension_bounds':bool(a['tension_n'].min()>=-1e-10 and a['tension_n'].max()<=8+1e-10),'control_bounds':bool(a['control_n'].min()>=-1e-10 and a['control_n'].max()<=8+1e-10),'actual_rope_speed':rope<=acc['rope_speed_m_s'],'actual_stroke':stroke<=acc['stroke_from_initial_m'],'finite_no_warning':bool(np.isfinite(a['qpos']).all() and np.isfinite(a['qvel']).all() and a['warnings'].max()==0),'full_timestep_coverage':bool(len(t)==round(task['duration_s']/task['physics_dt_s'])+1 and np.allclose(np.diff(t),task['physics_dt_s'],atol=1e-12))}
    nominal=mujoco.MjData(m);mujoco.mj_forward(m,nominal)
    # A prepared posture must not silently reset the original actuator zero.
    checks['source_zero_reference_stroke']=bool(abs(a['tendon_length_m']-nominal.ten_length).max()<=acc['stroke_from_initial_m'])
    checks['initial_source_zero_reference_stroke']=bool(abs(a['tendon_length_m'][0]-nominal.ten_length).max()<=acc['stroke_from_initial_m'])
    longest=0.;start=None
    for k,valid in enumerate((error<=acc['tip_error_m'])&(speed<=acc['tip_speed_m_s'])):
        if valid:
            if start is None:start=k
            longest=max(longest,float(t[k]-t[start]))
        else:start=None
    return {'status':'PASS' if all(checks.values()) else 'FAIL','checks':checks,'metrics':{'minimum_clearance_m':float(a['clearance_m'][minimum]),'minimum_clearance_time_s':float(t[minimum]),'robot_geom':m.geom(int(a['nearest_robot_geom'][minimum])).name,'wall_geom':m.geom(int(a['nearest_wall_geom'][minimum])).name,'minimum_clearance_state_index':minimum,'minimum_clearance_qpos':a['qpos'][minimum].tolist(),'minimum_clearance_qvel':a['qvel'][minimum].tolist(),'minimum_clearance_fromto_m':a['nearest_fromto_m'][minimum].tolist(),'wall_contact_state_count':int((a['wall_contacts']>0).sum()),'tip_entry_time_s':None if enter is None else float(t[enter]),'tip_exit_time_s':None if exit is None else float(t[exit]),'tip_final_error_m':float(error[-1]),'hold_max_error_m':float(error[hold].max()),'hold_max_speed_m_s':float(speed[hold].max()),'longest_position_speed_hold_s':longest,'max_tension_n':float(a['tension_n'].max()),'max_rope_speed_m_s':rope,'max_stroke_m':stroke,'initial_tip_m':tip[0].tolist(),'initial_clearance_m':float(a['clearance_m'][0]),'physics_steps':len(t)-1},'geometry_pairs_per_state':len(geometry_ids(m)[0])*len(geometry_ids(m)[1]),'scope':'all moving physical geoms including guide, connector, payload; fixed base and floor excluded; no presentation shell','swept_continuous_time_claim':False}
