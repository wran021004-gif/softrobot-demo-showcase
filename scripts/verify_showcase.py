"""Minimal requested checks: model invariance, all saved poses, and actuator needs.
No new physics integration, perturbations, hash audit or benchmark execution.
"""
import sys,json
from pathlib import Path
import numpy as np
import mujoco
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'viewer'))
from showcase import Replay,read

def check(r):
    b=mujoco.MjModel.from_xml_path(str(r.base_path));d=mujoco.MjData(b);m=r.m
    fields=['body_mass','body_inertia','body_ipos','body_iquat','body_pos','body_quat','jnt_axis','jnt_pos','jnt_stiffness','jnt_range','dof_damping','actuator_gainprm','actuator_biasprm','actuator_ctrlrange','actuator_forcerange','tendon_range','tendon_limited']
    for field in fields:np.testing.assert_allclose(getattr(m,field),getattr(b,field),rtol=0,atol=1e-14,err_msg=field)
    for i in range(b.ngeom):
        name=b.geom(i).name;j=m.geom(name).id
        for field in ['geom_type','geom_size','geom_pos','geom_quat','geom_contype','geom_conaffinity','geom_friction']:
            np.testing.assert_allclose(getattr(m,field)[j],getattr(b,field)[i],rtol=0,atol=1e-14,err_msg=name+': '+field)
    for i in range(m.ngeom):
        if m.geom(i).name.startswith('visual_'):assert m.geom_contype[i]==m.geom_conaffinity[i]==0
    max_pose_error=0.;max_tendon_error=0.;lengths=[];speeds=[];tip_speed=[]
    for i in range(len(r.times)):
        r.restore(i);d.qpos[:]=r.qpos[i];d.qvel[:]=r.qvel[i];mujoco.mj_forward(b,d)
        max_tendon_error=max(max_tendon_error,float(np.max(abs(r.d.ten_length-d.ten_length))))
        if r.demo=='p3':
            body=m.body(r.binding['body_name']).id
            point=r.d.xpos[body]+r.d.xmat[body].reshape(3,3)@r.binding['local_position_m'];error=np.max(abs(point-r.point[i]))
        else:error=np.max(abs(r.d.site_xpos[r.tip_id]-r.tip[i]))
        max_pose_error=max(max_pose_error,float(error));lengths.append(r.d.ten_length.copy());speeds.append(r.d.ten_velocity.copy())
        if r.demo=='m1':
            j=np.zeros((3,m.nv));mujoco.mj_jacSite(m,r.d,j,None,r.tip_id);tip_speed.append(np.linalg.norm(j@r.d.qvel))
    assert max_pose_error<1e-10;assert max_tendon_error<1e-12
    output=dict(status='PASS',rows_checked=len(r.times),max_saved_pose_error_m=max_pose_error,max_tendon_length_difference_m=max_tendon_error,physics_steps=0,visual_physics_invariant=True)
    if r.demo!='p3':
        lengths=np.array(lengths);speeds=np.array(speeds)
        output['actuator_requirements']=dict(channel_order=[m.actuator(i).name for i in range(m.nu)],max_tension_n=r.tension.max(axis=0).tolist(),stroke_m=np.ptp(lengths,axis=0).tolist(),max_speed_m_s=np.max(abs(speeds),axis=0).tolist(),scope='Observed nominal run requirements; total tendon peak-to-peak length and analytical MuJoCo tendon velocity. No spool or motor model.')
        if r.demo=='m1':
            hold=r.times>=.54-1e-10;output['holding_window_full_timestep']=dict(start_s=.54,end_s=.60,max_error_m=float(np.linalg.norm(r.tip-r.reference,axis=1)[hold].max()),max_tip_speed_m_s=float(np.array(tip_speed)[hold].max()),scope='Diagnostic over all saved physical timesteps; original acceptance evidence retained separately')
        else:
            e=np.linalg.norm(r.tip-r.reference,axis=1);result=read(ROOT/'figure_eight_tracking'/'results.json');assert result['status']=='PASS'
            np.testing.assert_allclose(np.sqrt(np.mean(e**2)),result['metrics']['tracking_rmse_m'],atol=1e-12)
            assert (r.tension>.2).all() and (r.tension<8).all();assert np.max(abs(speeds))<.03;assert np.max(np.ptp(lengths,axis=0))<.03
            output['tracking_metrics_verified']=True
    return output

def main():
    report={}
    for demo,case in [('m1','central_high'),('m1','positive_high'),('m1','negative_low'),('eight',None),('p3','plus_x'),('p3','minus_x')]:
        key=demo+('/'+case if case else '');report[key]=check(Replay(demo,case));print(key,report[key]['status'],report[key]['rows_checked'],flush=True)
    (ROOT/'verification.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf8')
if __name__=='__main__':main()
