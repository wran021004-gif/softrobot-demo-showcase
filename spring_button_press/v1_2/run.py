"""Bounded adaptive v1.2 runs; unmodified v1.0 evaluator is authoritative."""
import argparse,time,shutil,json,importlib.util
import numpy as np
import mujoco
from mechanics import HERE,BASE,assemble,read,write,LABEL,S4
from controller import Controller,inverse_bank,reference
from evaluator import contact_state,evaluate

def rollout(m,origin,bank,cfg,controls=None):
    d=mujoco.MjData(m);d.ctrl[:]=.2;mujoco.mj_forward(m,d)
    controller=Controller(m,origin,bank,cfg) if controls is None else None
    sid=m.site('probe_center').id;cap=m.geom('button_cap').id;slider=m.body('button_slider').id;j=m.joint('button_slide').id;qi=m.jnt_qposadr[j];vi=m.jnt_dofadr[j]
    keys=['time_s','qpos','qvel','control_n','tension_n','tendon_length_m','tendon_speed_m_s','probe_m','reference_m','tip_speed_m_s',
        'button_q_m','button_qdot_m_s','button_qacc_m_s2','contact_force_n','contact_pairs','contact_position_m','lateral_error_m',
        'unexpected_contact_count','warnings','external_force_max','button_axis_contact_n']
    rows={k:[] for k in keys};badpairs=set();J=np.zeros((3,m.nv));log=[];ramp_start=d.ctrl.copy();ramp_target=d.ctrl.copy()
    for k in range(2001):
        if k:
            if controls is not None:d.ctrl[:]=controls[k]
            elif (k-1)%10==0:
                ramp_start=d.ctrl.copy();ramp_target,info=controller.command(d);log.append({'time_s':float(d.time),**{a:np.asarray(b).tolist() for a,b in info.items()}})
            if controls is None:
                if cfg.get('command_interpolation')=='quintic_10ms':
                    from solver import smooth
                    fraction=float(smooth((((k-1)%10)+1)/10));d.ctrl[:]=ramp_start+fraction*(ramp_target-ramp_start)
                elif cfg.get('command_interpolation')=='linear_10ms':
                    fraction=(((k-1)%10)+1)/10;d.ctrl[:]=ramp_start+fraction*(ramp_target-ramp_start)
                else:d.ctrl[:]=ramp_target
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
    if controller:
        summary['rope_guard_infeasible_fallback_updates']=controller.guard_fallbacks
        summary['command_interpolation']=cfg.get('command_interpolation','hold_10ms')
        summary['startup_shape']=cfg.get('startup_shape','quintic_current')
        summary['interpolation_scope']='Feedback updates every 10 ms; optional open-loop interpolation at physics steps; no extra feedback updates'
    return {k:np.asarray(v) for k,v in rows.items()},sorted(badpairs),summary,log

def run(name,request=None,overrides=None,replay=None):
    folder=HERE/'candidates'/name
    if (folder/'result.json').exists():raise RuntimeError('Candidate already evaluated; refusing a hidden repeat')
    usage=read(HERE/'results/budget_usage.json') if (HERE/'results/budget_usage.json').exists() else {'control_candidates':0,'structure_candidates':0,'final_replays':0,'full_evaluations':0,'physics_steps':0,'provider_calls':0,'main_calls':0,'reporter_calls':0}
    group='final_replays' if replay else 'control_candidates' if request is None else 'structure_candidates';limit={'final_replays':2,'control_candidates':12,'structure_candidates':12}[group]
    if usage[group]>=limit or usage['full_evaluations']>=24:raise RuntimeError('Evaluation budget exhausted (24 including replays)')
    m,mechanics=assemble(request,folder=folder);d=mujoco.MjData(m);mujoco.mj_forward(m,d);origin=d.site_xpos[m.site('probe_center').id].copy()
    cfg=read(BASE/'v1_1/candidates/S4/effective_control.json');cfg.update(overrides or {})
    cfg.setdefault('position_weights',[.1,1.,1.])
    cfg.setdefault('feedforward_tracking_penalty',10.)
    if replay:
        src=HERE/'candidates'/replay;cfg=read(src/'effective_control.json');bank=np.load(src/'inverse_bank.npz')['states'];controls=None;opt={'replay_of':replay,'mode':'independent closed-loop from zero state using archived controller source and matching inverse bank'}
        # Fresh process imports the archived implementation, then recomputes
        # every 10 ms feedback command; no saved physical state is injected.
        spec=importlib.util.spec_from_file_location('archived_controller',src/'controller_source.py');module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        global Controller,reference
        Controller=module.Controller;reference=module.reference
        assert (folder/'environment.xml').read_bytes()==(src/'environment.xml').read_bytes()
    elif request is None:
        bank=np.load(BASE/'v1_1/candidates/S4/inverse_bank.npz')['states'];opt=read(BASE/'v1_1/candidates/S4/optimization.json');opt['provenance']='reused matching, unchanged S4 structure algebraic bank; no trajectory reuse';controls=None
    else:bank,opt=inverse_bank(m,origin);controls=None
    write(folder/'effective_control.json',cfg);write(folder/'optimization.json',opt);np.savez_compressed(folder/'inverse_bank.npz',states=bank)
    (folder/'controller_source.py').write_text((HERE/'controller.py').read_text(encoding='utf8'),encoding='utf8')
    if replay:shutil.copyfile(src/'controller_source.py',folder/'controller_source.py')
    (folder/'runner_source.py').write_text((HERE/'run.py').read_text(encoding='utf8'),encoding='utf8')
    # Reserve before integration so a failed process cannot hide consumed runs.
    assert d.ncon==0 and m.nu==4 and m.nq==49
    write(folder/'precheck.json',{'xml_load':True,'initial_contacts':int(d.ncon),'nu':m.nu,'nq':m.nq,'physics_dt_s':float(m.opt.timestep)})
    usage[group]+=1;usage['full_evaluations']+=1;usage['physics_steps']+=2000;write(HERE/'results/budget_usage.json',usage)
    a,bad,summary,log=rollout(m,origin,bank,cfg,controls=controls);result=evaluate(a,m,mechanics['total_structure_mass_kg'])
    result.update(label=LABEL,revision='v1.2 design/controller, unchanged spring_button_press_v1.0 task',unexpected_pairs=bad,mechanical_mass_breakdown={k:v for k,v in mechanics.items() if 'mass' in k})
    peak=np.unravel_index(np.argmax(abs(a['tendon_speed_m_s'])),a['tendon_speed_m_s'].shape);low=np.argmin(a['button_q_m'])
    result['diagnostics']={'peak_rope_time_s':float(a['time_s'][peak[0]]),'peak_rope_tendon':f'T{peak[1]}','peak_rope_signed_mm_s':float(a['tendon_speed_m_s'][peak]*1000),'minimum_button_um':float(a['button_q_m'][low]*1e6),'minimum_button_time_s':float(a['time_s'][low]),'max_actual_tension_rate_n_s':float(abs(np.diff(a['control_n'],axis=0)/.001).max()),'fallback_updates':summary.get('rope_guard_infeasible_fallback_updates',0)}
    np.savez_compressed(folder/'trajectory.npz',**a);write(folder/'result.json',result);write(folder/'runtime_configuration.json',summary);write(folder/'controller_trace.json',log)
    if replay:
        old=np.load(HERE/'candidates'/replay/'trajectory.npz')
        differences={key:float(np.nanmax(abs(a[key]-old[key]))) for key in ['qpos','qvel','control_n','tendon_speed_m_s','button_q_m','contact_force_n']}
        write(folder/'replay_check.json',{'source':replay,'mode':'independent archived-controller closed-loop','initial_state':'zero qpos/qvel, 0.20 N tension, native contact','max_absolute_differences':differences,'status':result['status'],'identical':all(x==0 for x in differences.values())})
    print(name,result['status'],[k for k,v in result['checks'].items() if not v],result['diagnostics'],result['metrics'],flush=True)
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('name');p.add_argument('--structure');p.add_argument('--config');p.add_argument('--replay');args=p.parse_args()
    request=read(args.structure) if args.structure else None
    overrides=read(args.config) if args.config else {}
    run(args.name,request,overrides,replay=args.replay)
