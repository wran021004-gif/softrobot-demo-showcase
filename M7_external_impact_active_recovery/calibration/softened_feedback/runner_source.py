"""Finite M7 development, complete 0.5 ms state/control/force records."""
import argparse,csv,hashlib,json,shutil,sys,platform
from pathlib import Path
import numpy as np
import mujoco
from controller import Controller,prepare
HERE=Path(__file__).resolve().parent
SOURCE=Path(r'D:\softrobot-benchmark-exports\base_only_accepted_20261028/M2')
CASES={'no_impulse':(0,True),'positive_y':(1,True),'negative_y':(-1,True),'positive_y_no_feedback':(1,False),'negative_y_no_feedback':(-1,False)}
DEFAULT={'reach_end_s':.25,'startup_s':.02,'position_scale_m':.005,'joint_weight':.01,'velocity_weight':.3,'joint_velocity_weight':.0001,'tension_weight':.05,'tension_rate_n_s':200.,'input_change_weight':.01,'predicted_speed_limit_m_s':.023}
def read(p):return json.loads(Path(p).read_text(encoding='utf8'))
def write(p,x):
    p=Path(p);p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(x,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf8')
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def csvsave(p,columns,array):
    np.savetxt(p,np.asarray(array),delimiter=',',header=','.join(columns),comments='',fmt='%.17g')
def setup():
    if not (HERE/'model.xml').exists():shutil.copyfile(SOURCE/'robot.xml',HERE/'model.xml')
    assert sha(HERE/'model.xml')==sha(SOURCE/'robot.xml')
    if not (HERE/'structure.json').exists():shutil.copyfile(SOURCE/'structure.json',HERE/'structure.json')
    if not (HERE/'task.json').exists():
        write(HERE/'task.json',{'version':'M7_external_impact_active_recovery_v1','label':'Codex-assisted engineering design; not Main autonomous Study','target_m':[.290,-.035,.190],'duration_s':1.,'physics_dt_s':.0005,'control_dt_s':.01,'initial_qpos':'all zero, unchanged M2','initial_qvel':'all zero','initial_control_n':.2,'windows_s':{'approach':[0,.3],'pre_impact_hold':[.3,.5],'impact':[.5,.55],'recovery':[.55,.9],'final_hold':[.9,1.]},'impulse':{'onset_s':.5,'duration_s':.05,'peak_n':.03,'shape':'half sine','application':'world Y at actual payload COM; zero COM couple','calibration_frozen':False},'acceptance':{'position_error_m':.01,'tip_speed_m_s':.02,'recovery_deadline_s_after_pulse':.35,'continuous_hold_s':.1,'tension_bounds_n':[0,8],'geometric_tendon_stroke_m':.015,'actual_tendon_speed_m_s':.025,'unexpected_contacts':0,'warnings':0},'controller_observations':['qpos','qvel','fixed target','normal task clock for approach only'],'controller_forbidden_inputs':['impact onset','direction','force','future disturbance']})
    if not (HERE/'provenance.json').exists():write(HERE/'provenance.json',{'source_model':str(SOURCE/'robot.xml'),'source_model_sha256':sha(SOURCE/'robot.xml'),'model_sha256':sha(HERE/'model.xml'),'source_structure_sha256':sha(SOURCE/'structure.json'),'structure_unchanged':True,'mujoco':mujoco.__version__,'numpy':np.__version__,'python':platform.python_version(),'existing_interface_inspected':'Core GVSNMPCController command(t,geometry,q,v), unavailable CasADi in showcase environment; use small task-local native-state LQR and bounded allocation','provider_calls':0,'main_calls':0,'reporter_calls':0})
    return read(HERE/'task.json')
def pulse(t,sign,task):
    p=task['impulse'];x=(t-p['onset_s'])/p['duration_s']
    return float(sign*p['peak_n']*np.sin(np.pi*x)) if 0<x<1 else 0.
def evaluate(a,task):
    t=a['time_s'];error=np.linalg.norm(a['tip_m']-task['target_m'],axis=1);speed=np.linalg.norm(a['tip_velocity_m_s'],axis=1);valid=(error<=.01+1e-12)&(speed<=.02+1e-12)
    pre=(t>=.3-1e-12)&(t<=.5+1e-12);final=t>=.9-1e-12;post=t>=.55-1e-12;dt=task['physics_dt_s'];need=round(.1/dt);start=None;enter=None;longest=0.
    for k in np.flatnonzero(post):
        if valid[k]:
            if start is None:start=k
            longest=max(longest,float(t[k]-t[start]))
            if k-start>=need and enter is None:enter=float(t[start])
        else:start=None
    q0=a['tendon_length_m'][0];maxstroke=float(abs(a['tendon_length_m']-q0).max());maxrope=float(abs(a['tendon_velocity_m_s']).max());k0=round(.5/dt);delta=np.linalg.norm(a['tip_m']-a['tip_m'][k0],axis=1)
    checks={'pre_impact_hold_all_timesteps':bool(valid[pre].all()),'recovery_within_0p35s_with_100ms_hold':enter is not None and enter<=.9+1e-12,'final_hold_all_timesteps':bool(valid[final].all()),'tension_limits':bool(a['tension_n'].min()>=-1e-9 and a['tension_n'].max()<=8+1e-9),'stroke_limit':maxstroke<=.015+1e-9,'actual_speed_limit':maxrope<=.025+1e-9,'no_unexpected_collision':bool(a['contacts'].max()==0),'finite_no_warning':bool(np.isfinite(a['qpos']).all() and np.isfinite(a['qvel']).all() and a['warnings'].max()==0),'force_off_after_pulse':bool(np.max(abs(a['force_n'][t>=.55-1e-12]))<1e-12),'controller_normal':bool(np.isfinite(a['control_n']).all())}
    maxid=np.argmax(delta[k0:])+k0;peakv=np.unravel_index(np.argmax(abs(a['tendon_velocity_m_s'])),a['tendon_velocity_m_s'].shape)
    return {'status':'PASS' if all(checks.values()) else 'FAIL','checks':checks,'metrics':{'pre_hold_max_error_m':float(error[pre].max()),'pre_hold_max_speed_m_s':float(speed[pre].max()),'max_post_impact_displacement_m':float(delta[k0:].max()),'max_displacement_time_s':float(t[maxid]),'max_post_impact_target_error_m':float(error[k0:].max()),'peak_post_impact_tip_speed_m_s':float(speed[k0:].max()),'peak_tip_speed_entire_run_m_s':float(speed.max()),'recovery_time_after_pulse_s':None if enter is None else max(0,enter-.55),'continuous_post_pulse_valid_s':longest,'final_hold_max_error_m':float(error[final].max()),'final_hold_max_speed_m_s':float(speed[final].max()),'residual_error_m':float(error[-1]),'max_tension_n':float(a['tension_n'].max()),'max_tendon_speed_m_s':maxrope,'max_tendon_speed_time_s':float(t[peakv[0]]),'max_tendon_speed_channel':int(peakv[1]),'max_tendon_stroke_m':maxstroke,'impulse_ns':a['impulse_ns'][-1].tolist(),'control_total_variation_n':float(abs(np.diff(a['control_n'],axis=0)).sum())}}
def rollout(m,bank,cfg,task,sign,feedback):
    d=mujoco.MjData(m);d.ctrl[:]=.2;mujoco.mj_forward(m,d);ctrl=Controller(m,bank,task['target_m'],cfg);sid=m.site('tip_site').id;pid=m.body('payload').id;dt=float(m.opt.timestep);J=np.zeros((3,m.nv));rows=[];log=[];impulse=np.zeros(3);ramp_start=d.ctrl.copy();ramp_target=d.ctrl.copy();frozen=None;last_force=np.zeros(3);last_point=d.xipos[pid].copy()
    for k in range(round(task['duration_s']/dt)+1):
        t=k*dt
        if k:
            pre=(k-1)*dt
            if (k-1)%20==0:
                ramp_start=d.ctrl.copy()
                if not feedback and pre>=.5-1e-12:
                    if frozen is None:frozen=d.ctrl.copy()
                    ramp_target=frozen.copy();log.append({'time_s':pre,'feedback':False,'command_n':ramp_target.tolist(),'mode':'frozen actual pre-impact input'})
                else:
                    ramp_target=ctrl.command(pre,d.qpos.copy(),d.qvel.copy());log.append({'time_s':pre,'feedback':True,'command_n':ramp_target.tolist()})
            alpha=(((k-1)%20)+1)/20;d.ctrl[:]=ramp_start+alpha*(ramp_target-ramp_start)
            last_force=np.array([0,pulse(pre,sign,task),0]);last_point=d.xipos[pid].copy();d.xfrc_applied[:]=0;d.xfrc_applied[pid,:3]=last_force;impulse+=last_force*dt
            mujoco.mj_step(m,d)
        d.xfrc_applied[:]=0;d.xfrc_applied[pid,:3]=[0,pulse(t,sign,task),0];mujoco.mj_forward(m,d);mujoco.mj_jacSite(m,d,J,None,sid)
        rows.append((t,d.qpos.copy(),d.qvel.copy(),d.site_xpos[sid].copy(),J@d.qvel,d.ctrl.copy(),-d.actuator_force.copy(),d.ten_length.copy(),d.ten_velocity.copy(),np.array([0,pulse(t,sign,task),0]),d.xipos[pid].copy(),impulse.copy(),sum(w.number for w in d.warning),d.ncon,last_force.copy(),last_point.copy()))
    keys=['time_s','qpos','qvel','tip_m','tip_velocity_m_s','control_n','tension_n','tendon_length_m','tendon_velocity_m_s','force_n','force_point_m','impulse_ns','warnings','contacts','previous_step_force_n','previous_step_force_point_m']
    return {key:np.asarray([r[i] for r in rows]) for i,key in enumerate(keys)},log,ctrl.log
def save_case(folder,a,result,updates,trace,cfg):
    folder.mkdir(parents=True,exist_ok=False);np.savez_compressed(folder/'trajectory.npz',**a);write(folder/'result.json',result);write(folder/'effective_control.json',cfg);write(folder/'controller_updates.json',trace)
    n=a['qpos'].shape[1];nu=a['control_n'].shape[1];xyz=lambda p:[p+'_'+x for x in 'xyz']
    csvsave(folder/'trajectory_timestep.csv',['time_s']+[f'qpos_{i}' for i in range(n)]+[f'qvel_{i}' for i in range(n)]+xyz('tip_position_m')+xyz('tip_velocity_m_s'),np.c_[a['time_s'],a['qpos'],a['qvel'],a['tip_m'],a['tip_velocity_m_s']])
    csvsave(folder/'control_timestep.csv',['time_s']+[f'command_n_{i}' for i in range(nu)]+[f'tension_n_{i}' for i in range(nu)]+[f'tendon_length_m_{i}' for i in range(nu)]+[f'tendon_speed_m_s_{i}' for i in range(nu)],np.c_[a['time_s'],a['control_n'],a['tension_n'],a['tendon_length_m'],a['tendon_velocity_m_s']])
    csvsave(folder/'control_updates.csv',['time_s','feedback_enabled']+[f'target_command_n_{i}' for i in range(nu)],[[x['time_s'],int(x['feedback']),*x['command_n']] for x in updates])
    csvsave(folder/'external_force_timestep.csv',['time_s']+xyz('force_n')+xyz('payload_com_m')+xyz('cumulative_applied_impulse_ns')+xyz('previous_step_force_n')+xyz('previous_step_application_com_m'),np.c_[a['time_s'],a['force_n'],a['force_point_m'],a['impulse_ns'],a['previous_step_force_n'],a['previous_step_force_point_m']])
def run(name,overrides=None,calibration=False,independent=None):
    task=setup();usage=read(HERE/'results/budget_usage.json') if (HERE/'results/budget_usage.json').exists() else {'candidate_groups':0,'calibration_runs':0,'case_integrations':0,'physics_steps':0,'independent_process_verifications':0,'provider_calls':0,'main_calls':0,'reporter_calls':0}
    if independent:
        if usage['independent_process_verifications']>=1:raise RuntimeError('Independent verification already consumed')
        usage['independent_process_verifications']+=1;src=HERE/'candidates'/independent;cfg=read(src/'effective_control.json');bank=dict(np.load(src/'controller_bank.npz'));root=HERE/'independent'/independent
    else:
        if usage['candidate_groups']>=8:raise RuntimeError('Eight candidate groups exhausted')
        usage['candidate_groups']+=1;cfg={**DEFAULT,**(overrides or {})};root=HERE/('calibration' if calibration else 'candidates')/name
        if root.exists():raise RuntimeError('No repeat/overwrite allowed')
        m=mujoco.MjModel.from_xml_path(str(HERE/'model.xml'));bank,opt=prepare(m,task['target_m']);root.mkdir(parents=True);np.savez_compressed(root/'controller_bank.npz',**bank);write(root/'optimization.json',opt);write(root/'effective_control.json',cfg)
        shutil.copyfile(HERE/'controller.py',root/'controller_source.py');shutil.copyfile(HERE/'run.py',root/'runner_source.py')
    write(HERE/'results/budget_usage.json',usage)
    results={};names=['positive_y_no_feedback'] if calibration else CASES
    for case in names:
        sign,feedback=CASES[case];m=mujoco.MjModel.from_xml_path(str(HERE/'model.xml'));assert m.opt.timestep==.0005 and m.nu==6 and m.nq==48
        usage['case_integrations']+=1;usage['physics_steps']+=2000;usage['calibration_runs']+=int(calibration);write(HERE/'results/budget_usage.json',usage)
        a,updates,trace=rollout(m,bank,cfg,task,sign,feedback);r=evaluate(a,task);r['case']=case;r['feedback_enabled_after_impact']=feedback;r['speed_allocator_fallbacks']=sum(x['fallback'] for x in trace);save_case(root/case,a,r,updates,trace,cfg);results[case]=r
        print(name,case,r['status'],[k for k,v in r['checks'].items() if not v],r['metrics'],flush=True)
    write(root/'evaluation.json',results)
    if calibration:
        result=results['positive_y_no_feedback'];displacement=result['metrics']['max_post_impact_displacement_m'];old=task['impulse']['peak_n']
        if result['checks']['pre_impact_hold_all_timesteps'] and result['checks']['actual_speed_limit']:
            if task['impulse']['calibration_frozen']:raise RuntimeError('Cannot recalibrate frozen task')
            new=.04 if displacement<.005 else .02 if displacement>.015 else old
            task['impulse'].update(peak_n=new,calibration_frozen=True,calibration_peak_n=old,calibration_displacement_m=displacement,adjustment_count=int(new!=old),decision='One prescribed amplitude selection from valid pre-impact original-structure frozen-input baseline; no further retuning permitted')
        else:task['impulse']['invalid_calibration_note']='Failed pre-hold or whole-run actuator safety: retained as failed control screen, never a valid impact amplitude calibration.'
        write(HERE/'task.json',task)
    if independent:
        errors={}
        for case in CASES:
            old=np.load(src/case/'trajectory.npz');fresh=np.load(root/case/'trajectory.npz');errors[case]={k:float(abs(old[k]-fresh[k]).max()) for k in ['qpos','qvel','control_n','tip_m','tip_velocity_m_s','force_n']}
        report={'source_candidate':independent,'fresh_process':True,'initial_state':'original zero qpos/qvel, 0.20 N control','state_injection_from_saved_trajectory':False,'absolute_tolerance':1e-10,'maximum_differences':errors,'all_histories_identical':all(v==0 for case in errors.values() for v in case.values()),'active_case_statuses':{k:results[k]['status'] for k in ['no_impulse','positive_y','negative_y']}}
        write(HERE/'results/independent_replay.json',report)
    return results
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('name');p.add_argument('--config',type=Path);p.add_argument('--calibration',action='store_true');p.add_argument('--independent');a=p.parse_args();run(a.name,read(a.config) if a.config else None,a.calibration,a.independent)
