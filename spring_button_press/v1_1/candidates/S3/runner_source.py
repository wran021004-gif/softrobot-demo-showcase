"""Bounded v1.1 runs; the unmodified v1.0 evaluator is authoritative."""
import argparse,time
import numpy as np
import mujoco
from mechanics import HERE,BASE,assemble,read,write,LABEL
from controller import Controller,inverse_bank,reference
from evaluator import contact_state,evaluate

def rollout(m,origin,bank,cfg,controls=None):
    d=mujoco.MjData(m);d.ctrl[:]=.2;mujoco.mj_forward(m,d)
    controller=Controller(m,origin,bank,cfg) if controls is None else None
    sid=m.site('probe_center').id;cap=m.geom('button_cap').id;slider=m.body('button_slider').id;j=m.joint('button_slide').id;qi=m.jnt_qposadr[j];vi=m.jnt_dofadr[j]
    keys=['time_s','qpos','qvel','control_n','tension_n','tendon_length_m','tendon_speed_m_s','probe_m','reference_m','tip_speed_m_s',
        'button_q_m','button_qdot_m_s','button_qacc_m_s2','contact_force_n','contact_pairs','contact_position_m','lateral_error_m',
        'unexpected_contact_count','warnings','external_force_max','button_axis_contact_n']
    rows={k:[] for k in keys};badpairs=set();J=np.zeros((3,m.nv));log=[]
    for k in range(2001):
        if k:
            if controls is not None:d.ctrl[:]=controls[k]
            elif (k-1)%10==0:
                d.ctrl[:],info=controller.command(d);log.append({'time_s':float(d.time),**{a:np.asarray(b).tolist() for a,b in info.items()}})
            mujoco.mj_step(m,d)
        mujoco.mj_forward(m,d);f,pairs,bad,cp=contact_state(m,d);badpairs.update(tuple(x) for x in bad)
        mujoco.mj_jacSite(m,d,J,None,sid);p=d.site_xpos[sid];axisforce=0.
        for ic in range(d.ncon):
            con=d.contact[ic]
            if {m.geom(int(con.geom1)).name,m.geom(int(con.geom2)).name}=={'probe','button_cap'}:
                z=np.zeros(6);mujoco.mj_contactForce(m,d,ic,z);world=con.frame.reshape(3,3).T@z[:3];axisforce+=float(world[1])*(1 if m.geom_bodyid[con.geom2]==slider else -1)
        vals=[d.time,d.qpos.copy(),d.qvel.copy(),d.ctrl.copy(),-d.actuator_force.copy(),d.ten_length.copy(),d.ten_velocity.copy(),p.copy(),reference(d.time,origin,cfg)[0],np.linalg.norm(J@d.qvel),
            d.qpos[qi],d.qvel[vi],d.qacc[vi],f,pairs,cp,np.linalg.norm((p-d.geom_xpos[cap])[[0,2]]),len(bad),sum(w.number for w in d.warning),
            max(np.max(abs(d.xfrc_applied)),np.max(abs(d.qfrc_applied))),axisforce]
        for key,val in zip(keys,vals):rows[key].append(val)
    summary=controller.summary if controller else {'mode':'independent recorded-control replay; zero state initialization'}
    if controller:summary['rope_guard_infeasible_fallback_updates']=controller.guard_fallbacks
    return {k:np.asarray(v) for k,v in rows.items()},sorted(badpairs),summary,log

def run(name,request=None,overrides=None,replay=None):
    folder=HERE/'candidates'/name
    if (folder/'result.json').exists():raise RuntimeError('Candidate already evaluated; refusing a hidden repeat')
    usage=read(HERE/'results/usage.json') if (HERE/'results/usage.json').exists() else {'prechecks':1,'baseline_evaluations':0,'codesign_evaluations':0,'final_replays':0,'full_evaluations':0,'physics_steps':0}
    group='final_replays' if replay else 'baseline_evaluations' if request is None else 'codesign_evaluations';limit={'final_replays':1,'baseline_evaluations':2,'codesign_evaluations':4}[group]
    if usage[group]>=limit or usage['full_evaluations']>=7:raise RuntimeError('Frozen evaluation budget exhausted')
    m,mechanics=assemble(request,folder=folder);d=mujoco.MjData(m);mujoco.mj_forward(m,d);origin=d.site_xpos[m.site('probe_center').id].copy()
    cfg=read(HERE/'controller.json')['defaults'];cfg.update(overrides or {})
    cfg.setdefault('position_weights',[.1,1.,1.])
    cfg.setdefault('feedforward_tracking_penalty',10.)
    if replay:
        src=HERE/'candidates'/replay;cfg=read(src/'effective_control.json');bank=np.load(src/'inverse_bank.npz')['states'];controls=np.load(src/'trajectory.npz')['control_n'];opt={'replay_of':replay}
    else:bank,opt=inverse_bank(m,origin);controls=None
    write(folder/'effective_control.json',cfg);write(folder/'optimization.json',opt);np.savez_compressed(folder/'inverse_bank.npz',states=bank)
    (folder/'controller_source.py').write_text((HERE/'controller.py').read_text(encoding='utf8'),encoding='utf8')
    # Reserve before integration so a failed process cannot hide consumed runs.
    usage[group]+=1;usage['full_evaluations']+=1;usage['physics_steps']+=2000;write(HERE/'results/usage.json',usage)
    a,bad,summary,log=rollout(m,origin,bank,cfg,controls=controls);result=evaluate(a,m,mechanics['total_structure_mass_kg'])
    result.update(label=LABEL,revision='v1.1 design/controller, unchanged spring_button_press_v1.0 task',unexpected_pairs=bad,mechanical_mass_breakdown={k:v for k,v in mechanics.items() if 'mass' in k})
    np.savez_compressed(folder/'trajectory.npz',**a);write(folder/'result.json',result);write(folder/'runtime_configuration.json',summary);write(folder/'controller_trace.json',log)
    if replay:
        old=np.load(HERE/'candidates'/replay/'trajectory.npz');write(folder/'replay_check.json',{'source':replay,'max_qpos_difference':float(abs(a['qpos']-old['qpos']).max()),'status':result['status']})
    print(name,result['status'],[k for k,v in result['checks'].items() if not v],result['metrics'],flush=True)
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('name');p.add_argument('--structure');p.add_argument('--gain',type=float);p.add_argument('--rate',type=float);p.add_argument('--replay');args=p.parse_args()
    request=None
    if args.structure:request=next(x for x in read(HERE/'design_space.json')['candidates'] if x['id']==args.structure)
    overrides={}
    if args.gain is not None:overrides['position_gain']=args.gain
    if args.rate is not None:overrides['tension_rate_limit_n_s']=args.rate
    run(args.name,request,overrides,replay=args.replay)
