"""Bounded M8 rollout. qpos/qvel evolve exclusively via mj_step after t=0."""
import argparse,csv,shutil,sys
import numpy as np
import mujoco
from environment import H,read,write,assemble,sha
from controller import Controller,geometry_ids,prepare
from evaluation import snapshot,centerline_checks,evaluate
def rollout(m,bank):
    task=read(H/'task.json');d=mujoco.MjData(m);d.qpos[:]=bank['qpos'];d.qvel[:]=bank['qvel'];d.ctrl[:]=bank['control_n'];mujoco.mj_forward(m,d)
    c=Controller(m,bank);robot,walls=geometry_ids(m);cells=[i for i in range(m.nbody) if '_cell_' in m.body(i).name];sid=m.site('tip_site').id;J=np.zeros((3,m.nv));rows=[];start=d.ctrl.copy();end=start.copy();stride=round(task['control_dt_s']/m.opt.timestep)
    for k in range(round(task['duration_s']/m.opt.timestep)+1):
        if k:
            pre=(k-1)*m.opt.timestep
            if (k-1)%stride==0:start=d.ctrl.copy();end=c.command(pre,d.qpos.copy(),d.qvel.copy())
            d.ctrl[:]=start+(((k-1)%stride)+1)/stride*(end-start);mujoco.mj_step(m,d)
        mujoco.mj_forward(m,d);mujoco.mj_jacSite(m,d,J,None,sid);best,contacts=snapshot(m,d,robot,walls);centers=d.xipos[cells].copy();tip=d.site_xpos[sid].copy();checks=centerline_checks(np.vstack([d.xpos[m.body('fixed_base').id],centers,tip]),task)
        phase=0 if tip[0]<task['tunnel']['entry_x_m'] else 1 if tip[0]<task['tunnel']['exit_x_m'] else 2 if k*m.opt.timestep<task['reach_end_s'] else 3
        rows.append((k*m.opt.timestep,d.qpos.copy(),d.qvel.copy(),tip,J@d.qvel,centers,phase,d.ctrl.copy(),-d.actuator_force.copy(),d.ten_length.copy(),d.ten_velocity.copy(),best[0],best[1],best[2],best[3],contacts,d.ncon,checks,sum(w.number for w in d.warning)))
    keys=['time_s','qpos','qvel','tip_m','tip_velocity_m_s','cell_centers_m','phase','control_n','tension_n','tendon_length_m','tendon_velocity_m_s','clearance_m','nearest_robot_geom','nearest_wall_geom','nearest_fromto_m','wall_contacts','contacts','centerline_checks','warnings']
    return {key:np.asarray([r[i] for r in rows]) for i,key in enumerate(keys)},c.updates
def save(folder,a,updates,m):
    task=read(H/'task.json');np.savez_compressed(folder/'trajectory.npz',**a);result=evaluate(a,task,m);write(folder/'evaluation.json',result)
    robot,walls=geometry_ids(m);write(folder/'geometry_map.json',{'robot':{str(i):m.geom(i).name for i in robot},'walls':{str(i):m.geom(i).name for i in walls},'cells':[m.body(i).name for i in range(m.nbody) if '_cell_' in m.body(i).name],'phases':{'0':'approach/alignment','1':'inside longitudinal tunnel slab','2':'past exit','3':'target hold'}})
    n=m.nq;nu=m.nu;xyz=lambda s:[s+'_'+x for x in 'xyz']
    def csvsave(name,headers,data):np.savetxt(folder/name,data,delimiter=',',header=','.join(headers),comments='',fmt='%.17g')
    csvsave('trajectory_timestep.csv',['time_s']+[f'qpos_{i}' for i in range(n)]+[f'qvel_{i}' for i in range(n)]+xyz('tip_m')+xyz('tip_velocity_m_s')+[f'{b}_center_m_{x}' for b in [m.body(i).name for i in range(m.nbody) if '_cell_' in m.body(i).name] for x in 'xyz']+['phase','minimum_clearance_m','nearest_robot_geom_id','nearest_wall_geom_id','wall_contacts','native_contacts','centerline_entry_open','centerline_exit_open','centerline_inside'],np.c_[a['time_s'],a['qpos'],a['qvel'],a['tip_m'],a['tip_velocity_m_s'],a['cell_centers_m'].reshape(len(a['time_s']),-1),a['phase'],a['clearance_m'],a['nearest_robot_geom'],a['nearest_wall_geom'],a['wall_contacts'],a['contacts'],a['centerline_checks']])
    limit=(abs(a['tendon_velocity_m_s'])>.025)|(abs(a['tendon_length_m']-a['tendon_length_m'][0])>.015)|(a['tension_n']>8)|(a['tension_n']<0)
    csvsave('control_timestep.csv',['time_s']+[f'{s}_{i}' for s in ['command_n','actual_tension_n','length_m','velocity_m_s','constraint_violation'] for i in range(nu)],np.c_[a['time_s'],a['control_n'],a['tension_n'],a['tendon_length_m'],a['tendon_velocity_m_s'],limit])
    csvsave('control_updates.csv',['time_s']+[f'command_n_{i}' for i in range(nu)],updates)
    with (folder/'clearance_timestep.csv').open('w',newline='',encoding='utf8') as f:
        w=csv.writer(f);w.writerow(['time_s','minimum_clearance_m','robot_geom','wall_geom',*xyz('robot_point_m'),*xyz('wall_point_m'),'wall_contacts','satisfies_1mm'])
        w.writerows([float(t),float(a['clearance_m'][k]),m.geom(int(a['nearest_robot_geom'][k])).name,m.geom(int(a['nearest_wall_geom'][k])).name,*a['nearest_fromto_m'][k],int(a['wall_contacts'][k]),int(a['clearance_m'][k]>=.001)] for k,t in enumerate(a['time_s']))
    return result
def main():
    p=argparse.ArgumentParser();p.add_argument('name');p.add_argument('--six',action='store_true');p.add_argument('--compact',action='store_true');p.add_argument('--independent',action='store_true');p.add_argument('--prepare-only',action='store_true');a=p.parse_args();folder=H/'cases'/a.name
    task=read(H/'task.json');usage=read(H/'results/budget_usage.json') if (H/'results/budget_usage.json').exists() else {'candidates':[],'physics_steps':0,'independent_processes':0,'main_calls':0,'reporter_calls':0,'provider_calls':0}
    if a.independent:
        assert usage['independent_processes']==0;usage['independent_processes']=1;source=H/'cases/final_candidate';m=mujoco.MjModel.from_xml_path(str(source/'model.xml'));bank=dict(np.load(source/'controller_bank.npz'));folder=H/'independent';folder.mkdir(exist_ok=False)
    else:
        assert len(usage['candidates'])<8 and a.name not in usage['candidates']
        if (folder/'controller_bank.npz').exists():m=mujoco.MjModel.from_xml_path(str(folder/'model.xml'));bank=dict(np.load(folder/'controller_bank.npz'))
        else:
            candidate=({'six':True,'section_scale':.8,'radius_m':.014,'segments':{'0':{'young_pa':15000000},'1':{'young_pa':13183593.75}}} if a.compact else {'six':True} if a.six else None)
            m=assemble(a.name,candidate);bank=prepare(m,folder,dict(np.load(H/'initial_state.npz')))
        if a.prepare_only:return
        usage['candidates'].append(a.name)
    assert abs(m.opt.timestep-task['physics_dt_s'])<1e-12
    write(H/'results/frozen_task_sha.json',{'task_sha256':sha(H/'task.json'),'initial_state_sha256':sha(H/'initial_state.npz')}) if not (H/'results/frozen_task_sha.json').exists() else None
    frozen=read(H/'results/frozen_task_sha.json');assert sha(H/'task.json')==frozen['task_sha256'] and sha(H/'initial_state.npz')==frozen['initial_state_sha256']
    usage['physics_steps']+=round(task['duration_s']/m.opt.timestep);write(H/'results/budget_usage.json',usage)
    data,updates=rollout(m,bank);result=save(folder,data,updates,m);print(result,flush=True)
    if a.independent:
        original=np.load(source/'trajectory.npz');diff={k:float(np.max(abs(original[k].astype(float)-data[k].astype(float)))) for k in data};write(H/'results/independent_replay.json',{'status':'MATCH' if max(diff.values())<=1e-10 else 'MISMATCH','task_status':result['status'],'max_absolute_differences':diff,'new_physics_steps':round(task['duration_s']/m.opt.timestep),'state_injection_after_initialization':False,'controls_recomputed_from_state':True})
if __name__=='__main__':main()
