"""Only v2 representative poses, visual/physics invariance and PHS sample mapping.
Optional --gui runs one short passive viewer per added model. No physical steps.
"""
import argparse,json,subprocess,sys
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'viewer'))
from showcase import Replay
from saved_states import FOLDERS,load_model,material_point,forbid_integration
SOURCE=Path(r'D:\softrobot-benchmark-exports\base_only_accepted_20261028')
def check(demo,case):
    r=Replay(demo,case);b=load_model(r.base_path,deep=demo=='phs');d=mujoco.MjData(b);m=r.m
    source=SOURCE/('C1_min_tendons_multi_task' if demo=='c1' else 'PHS2D_passive_head_stabilization' if demo=='phs' else demo.upper())/('model.xml' if demo=='c1' else 'robot.xml')
    assert source.read_bytes()==r.base_path.read_bytes()
    for key in ['body_mass','body_inertia','body_ipos','body_iquat','body_pos','body_quat','jnt_type','jnt_axis','jnt_pos','jnt_stiffness','jnt_range','dof_damping','actuator_trnid','actuator_gainprm','actuator_biasprm','actuator_ctrlrange','actuator_forcerange','tendon_range','tendon_limited','tendon_stiffness','tendon_damping','wrap_type','wrap_objid','wrap_prm']:
        np.testing.assert_allclose(getattr(m,key),getattr(b,key),rtol=0,atol=1e-14,err_msg=key)
    for i in range(b.ngeom):
        j=m.geom(b.geom(i).name).id
        for key in ['geom_type','geom_size','geom_pos','geom_quat','geom_contype','geom_conaffinity','geom_friction']:
            np.testing.assert_allclose(getattr(m,key)[j],getattr(b,key)[i],rtol=0,atol=1e-14,err_msg=b.geom(i).name+': '+key)
    added=[g for g in ET.parse(r.model_path).getroot().findall('.//geom') if g.get('name','').startswith('visual_')]
    for g in added:assert all(g.get(k)==v for k,v in dict(density='0',contype='0',conaffinity='0',group='3').items())
    error=0.;tendon_error=0.;ids=[0,len(r.times)//2,len(r.times)-1]
    for i in ids:
        r.restore(i);d.qpos[:]=r.qpos[i];d.qvel[:]=r.qvel[i];mujoco.mj_forward(b,d)
        error=max(error,float(np.max(abs(r.d.site_xpos[r.tip_id]-r.tip[i]))),float(np.max(abs(r.d.site_xmat[r.tip_id].reshape(3,3)-r.rotation[i]))))
        tendon_error=max(tendon_error,float(np.max(abs(r.d.ten_length-d.ten_length))))
        if demo=='phs':np.testing.assert_allclose(material_point(m,r.d,r.geometry,r.binding),r.point[i],atol=1e-12,rtol=0)
        if r.reference_axis is not None:np.testing.assert_allclose(r.d.site_xmat[r.tip_id].reshape(3,3)@r.axis_local,r.axis[i],atol=1e-12,rtol=0)
        if demo=='m2':
            bid=m.body('payload').id;np.testing.assert_allclose(r.d.xipos[bid],r.d.xpos[bid]+r.d.xmat[bid].reshape(3,3)@m.body_ipos[bid],atol=1e-12,rtol=0);assert m.body_mass[bid]==.01
    assert error<1e-12 and tendon_error<1e-12
    if demo=='m6':
        bid=m.body('payload').id;R=np.zeros(9);mujoco.mju_quat2Mat(R,m.body_iquat[bid]);R=R.reshape(3,3)
        np.testing.assert_allclose(R@np.diag(m.body_inertia[bid])@R.T,r.goal['rigid_tool']['inertia_com_local_kg_m2'],atol=1e-14,rtol=0);assert m.body_mass[bid]==.02
    r.update_figure(ids[-1]);r.update_aux_figure(ids[-1]);r.labels(ids[-1])
    scene=mujoco.MjvScene(m,maxgeom=3000);r.decorate(scene,ids[-1]);assert scene.ngeom>0
    return dict(status='PASS',case=r.case,pose_indices=ids,max_pose_or_rotation_difference=error,max_tendon_length_difference_m=tendon_error,visual_geoms_checked=len(added),model_xml_original_bytes=True,physics_unchanged=True,physics_steps=0)
def phs_samples():
    base=ROOT/FOLDERS['phs'];case='s050_positive'
    with np.load(base/case/'display_replay.npz',allow_pickle=False) as display,np.load(SOURCE/'PHS2D_passive_head_stabilization'/case/'states_timestep.npz',allow_pickle=False) as original:
        raw=original['trajectory'];ids=display['source_index'];n=144
        for key,expected in [('time_s',raw[ids,0]),('qpos',raw[ids,2:2+n]),('qvel',raw[ids,2+n:2+2*n]),('tip_m',original['tip_position_m'][ids]),('tip_rotation',original['tip_rotation'][ids]),('force_point_m',original['force_point_m'][ids])]:np.testing.assert_array_equal(display[key],expected)
        np.testing.assert_array_equal(display['force_n'][:,0],raw[ids,1]);assert not display['force_n'][:,1:].any()
        schedule=np.loadtxt(base/case/'external_force_schedule_display.csv',delimiter=',',skiprows=1)
        np.testing.assert_array_equal(schedule[:,3:6],display['force_n']);np.testing.assert_allclose(schedule[:,2],display['time_s'],rtol=0,atol=1e-11)
        assert ids[0]==0 and ids[-1]==len(raw)-1 and len(ids)==401
        return dict(status='PASS',case=case,exact_original_states_checked=len(ids),source_rows=len(raw),interpolated_states=0,original_force_schedule_matches=True)
def main():
    cli=argparse.ArgumentParser();cli.add_argument('--gui',action='store_true');args=cli.parse_args();forbid_integration();report={}
    cases=[('m2','positive'),('m3',None),('m4',None),('m5',None),('m6',None),('c1','M5'),('phs','s050_positive')]
    for demo,case in cases:
        report[demo]=check(demo,case);print(demo,'three saved poses and physics invariance PASS',flush=True)
        if args.gui:
            command=[sys.executable,str(ROOT/'viewer'/'showcase.py'),demo,'--smoke-seconds','2','--speed','2']
            if case:command+=['--case',case]
            run=subprocess.run(command,capture_output=True,text=True,encoding='utf8',timeout=30,check=True)
            gui=json.loads(run.stdout.strip().splitlines()[-1]);assert gui['frames']>5 and gui['physics_steps']==0;report[demo]['gui']=gui
    report['phs_sampling']=phs_samples();report['phs_large_deformation']=check('phs','s070_negative')
    (ROOT/'verification_v2.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf8')
    print('PHS 401 samples exactly match original; no interpolation; all checks PASS',flush=True)
if __name__=='__main__':main()
