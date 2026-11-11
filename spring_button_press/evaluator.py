"""Full 1 ms evaluator: true contact pair force, slide state, actuator motion."""
import numpy as np
import mujoco

def contact_state(m,d):
    force=0.;unexpected=[];positions=[];pair_count=0
    for i in range(d.ncon):
        c=d.contact[i];names={m.geom(int(c.geom1)).name,m.geom(int(c.geom2)).name}
        f=np.zeros(6);mujoco.mj_contactForce(m,d,i,f)
        if names=={'probe','button_cap'}:
            pair_count+=1;force+=max(0,float(f[0]));positions.append(c.pos.copy())
        elif c.dist<=0:unexpected.append(sorted(names))
    return force,pair_count,unexpected,(np.mean(positions,axis=0) if positions else np.full(3,np.nan))

def longest_interval(time,valid):
    best=0.;start=None
    for i,v in enumerate(valid):
        if v and start is None:start=i
        if start is not None and (not v or i==len(valid)-1):
            end=i if v else i-1;best=max(best,float(time[end]-time[start]));start=None
    return best

def evaluate(a,m,robot_mass,version='spring_button_press_v1.0'):
    t=a['time_s'];q=a['button_q_m'];v=a['button_qdot_m_s'];f=a['contact_force_n']
    h=(t>=.9-1e-9)&(t<=1.4+1e-9);near=(t>=.35-1e-9)&(t<=.60+1e-9)
    valid=h&(q>=.00175)&(q<=.00225)&(f>=.08)&(f<=.12)&(a['lateral_error_m']<=.002)&(a['contact_pairs']>0)
    duration=longest_interval(t,valid)
    lengths=a['tendon_length_m'];speeds=a['tendon_speed_m_s'];stroke=lengths-lengths[0]
    # Ideal tension does not model a spool; motion limits are checked on actual
    # geometric tendon length, consistent with the existing Family backend.
    checks={'real_contact_established':bool(((a['contact_pairs']>0)&(q>.0001)).any()),
        'contact_near_approach_end':bool(((a['contact_pairs']>0)&near).any()),
        'depth_and_force_hold_0p4s':duration>=.4-1e-9,
        'button_travel':bool(q.max()<=.003+1e-9 and q.min()>=-1e-6),
        'tension_limit':bool(a['tension_n'].max()<=8+1e-9 and a['tension_n'].min()>=.2-1e-9),
        'stroke_limit':bool(np.max(abs(stroke))<=.015+1e-9),
        'speed_limit':bool(np.max(abs(speeds))<=.025+1e-9),
        'release':bool(all(abs(q[i])<=.0001 and f[i]<.01 and a['tip_speed_m_s'][i]<=.01 for i in [1800,2000])),
        'no_unexpected_collision':not bool(a['unexpected_contact_count'].max()),
        'finite_no_warning':bool(np.isfinite(a['qpos']).all() and not a['warnings'].max()),
        'no_external_force':not bool(a['external_force_max'].max())}
    metrics={'max_hold_contact_force_error_n':float(np.max(abs(f[h]-.1))),
        'max_hold_depth_error_m':float(np.max(abs(q[h]-.002))),
        'hold_valid_continuous_s':duration,'hold_valid_total_s':float(np.sum(valid)*.001),
        'hold_depth_min_max_m':[float(q[h].min()),float(q[h].max())],
        'hold_contact_force_min_max_n':[float(f[h].min()),float(f[h].max())],
        'hold_lateral_error_max_m':float(a['lateral_error_m'][h].max()),
        'button_peak_m':float(q.max()),'first_contact_s':float(t[np.flatnonzero(a['contact_pairs']>0)[0]]) if (a['contact_pairs']>0).any() else None,
        'max_tension_n':float(a['tension_n'].max()),'max_rope_speed_m_s':float(np.max(abs(speeds))),
        'max_stroke_m':float(np.max(abs(stroke))),'structure_mass_kg':float(robot_mass),
        'total_tendon_length_variation_m':float(np.abs(np.diff(lengths,axis=0)).sum()),
        'release_at_1p8_and_2p0':[{ 'time_s':float(t[i]),'button_residual_m':float(q[i]),'contact_force_n':float(f[i]),'tip_speed_m_s':float(a['tip_speed_m_s'][i])} for i in [1800,2000]],
        'hold_contact_minus_spring_damping_max_n':float(np.max(abs(f[h]-50*q[h]-.2*v[h]))),
        'hold_button_dynamic_balance_residual_max_n':float(np.max(abs(a['button_axis_contact_n'][h]-50*q[h]-.2*v[h]-.005*a['button_qacc_m_s2'][h]))),
        'unexpected_contact_steps':int(np.count_nonzero(a['unexpected_contact_count']))}
    return {'version':version,'status':'PASS' if all(checks.values()) else 'FAIL','checks':checks,'metrics':metrics,
        'actuation_scope':'4 independent ideal tension channels; geometric tendon stroke/speed verified; no motor energy model'}
