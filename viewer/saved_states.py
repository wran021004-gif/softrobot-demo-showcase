"""Existing M/C1 named joints and PHS frozen state conventions, without integration.
Adapted from package_io.restore, C1 replay.py and phs2d_io.py.
"""
import json,threading
from pathlib import Path
import mujoco
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
FOLDERS={'m2':'M2_loaded_hold','m3':'M3_fast_reach','m4':'M4_legacy_tracking','m5':'M5_tool_axis','m6':'M6_loaded_scan','c1':'C1_four_tendons','phs':'PHS2D_head_stabilization'}
CASES={'m1':['central_high','positive_high','negative_low'],'m2':['zero','positive','negative'],'c1':['M1_A','M1_B','M1_C','M5'],'p3':['plus_x','minus_x'],'phs':[f's{s:03d}_{sign}' for s in [30,40,50,60,70] for sign in ['positive','negative']]+['equilibrium']}
TITLES={'m2':'External-load Reach / Hold','m3':'Fast Reach / Development Draft','m4':'Legacy Timed Polyline Tracking','m5':'Transient Position + Tool-axis Pointing','m6':'Loaded Position + Tool-axis Scan','c1':'4 Independent Actuation Channels','phs':'PHS-2D / Passive Horizontal Tip Stabilization'}
def read(p):return json.loads(p.read_text(encoding='utf8'))
def csv_read(p):
    with p.open(encoding='utf8') as f:columns=f.readline().strip().split(',')
    return columns,np.loadtxt(p,delimiter=',',skiprows=1,ndmin=2)
def load_model(path,deep=False):
    if not deep:return mujoco.MjModel.from_xml_path(str(path))
    result=[];errors=[]
    def compile_model():
        try:result.append(mujoco.MjModel.from_xml_string(Path(path).read_text(encoding='utf8')))
        except BaseException as e:errors.append(e)
    previous=threading.stack_size()
    try:
        threading.stack_size(max(previous,32*1024*1024));worker=threading.Thread(target=compile_model);worker.start()
    finally:threading.stack_size(previous)
    worker.join()
    if errors:raise errors[0]
    m=result[0];m.opt.iterations=100;m.opt.tolerance=1e-10 # frozen original PHS runtime overrides
    return m

def load_active(r,demo,case):
    r.case=case or ('positive' if demo=='m2' else 'M1_A' if demo=='c1' else 'default')
    choices=CASES.get(demo,['default'])
    if r.case not in choices:raise ValueError(f'{demo} cases: '+', '.join(choices))
    base=ROOT/FOLDERS[demo];folder=base/r.case;mapping=read(base/'joint_map.json')
    c1=demo=='c1';r.configuration=read(folder/'configuration.json');r.goal=r.configuration['task']['goal']['data']
    r.task=read(folder/'task.json') if not c1 else r.configuration['task']
    r.holding_window=([.54,.60] if r.case.startswith('M1_') else None) if c1 else r.task['holding_window_s']
    cols,a=csv_read(folder/('trajectory.csv' if c1 else 'trajectory_timestep.csv'));ix={c:i for i,c in enumerate(cols)}
    r.times=a[:,ix['time_s']];n=mapping['nq'];r.qpos=np.empty((len(a),n));r.qvel=np.empty_like(r.qpos)
    for j in mapping['joints']:
        r.qpos[:,j['qpos_address']]=a[:,ix[j['qpos_column']]];r.qvel[:,j['qvel_address']]=a[:,ix[j['qvel_column']]]
    vector=lambda prefix: a[:,[ix[prefix+'_'+v] for v in 'xyz']]
    r.tip=vector('tip_position_m');r.velocity=vector('tip_velocity_m_s');r.axis=vector('tool_axis_world')
    r.rotation=np.stack([a[:,ix[f'tip_rotation_r{i}{j}']] for i in range(3) for j in range(3)],axis=1).reshape(-1,3,3)
    r.tension=a[:,[ix[c] for c in mapping['actual_tension_columns']]]
    r.control=a[:,[ix['ctrl_n__'+name] for name in mapping['actuator_order']]] if c1 else a[:,[ix[c] for c in mapping['requested_tension_columns']]]
    r.force=vector('external_force_n') if demo=='m2' else None
    if c1:
        r.reference=np.tile(r.goal['target_m'],(len(a),1))
        r.reference_axis=np.tile(r.goal['target_axis_world'],(len(a),1)) if r.case=='M5' else None
    else:
        rc,b=csv_read(folder/'reference.csv');ri={c:i for i,c in enumerate(rc)}
        r.reference=np.column_stack([np.interp(r.times,b[:,ri['time_s']],b[:,ri['reference_position_m_'+v]]) for v in 'xyz'])
        if demo in ['m5','m6']:
            r.reference_axis=np.column_stack([np.interp(r.times,b[:,ri['time_s']],b[:,ri['reference_tool_axis_world_'+v]]) for v in 'xyz'])
    r.vertices=np.asarray(r.goal.get('vertices_m',[]));r.axis_local=np.asarray(r.goal.get('axis_local',[1,0,0]))
    if r.reference_axis is not None:
        r.reference_axis/=np.linalg.norm(r.reference_axis,axis=1)[:,None]
        r.angle_error=np.degrees(np.arccos(np.clip(np.sum(r.axis*r.reference_axis,axis=1),-1,1)))
    r.position_error=np.linalg.norm(r.tip-r.reference,axis=1)
    r.model_path=base/'scene.xml';r.base_path=base/'robot.xml';r.title=TITLES[demo]+' / '+r.case
    r.origin_note='Codex-assisted existing result / not Main autonomous Study'
    r.notes={
        'm2':'Direct world force at payload COM; no unloading record',
        'm3':'Development draft; final 50 ms hold only',
        'm4':'Legacy technical replay; dense max 12.095 mm; end speed 325 mm/s',
        'm5':'Transient; no roll constraint or hold; end speed 457.755 mm/s',
        'm6':'20 g rigid payload; no surface contact; dense axis peak 10.052 deg',
        'c1':'Independent initial state; no global actuator-minimum claim'
    }[demo]

def load_phs(r,case):
    r.case=case or 's050_positive'
    if r.case not in CASES['phs']:raise ValueError('phs cases: '+', '.join(CASES['phs']))
    base=ROOT/FOLDERS['phs'];r.geometry=read(base/'display_geometry.json');r.sampling=read(base/'display_sampling.json');r.entry=r.sampling['cases'][r.case];r.binding=r.entry['material_point_binding']
    with np.load(base/r.case/'display_replay.npz',allow_pickle=False) as a:
        for attr,key in [('qpos','qpos'),('qvel','qvel'),('times','time_s'),('tip','tip_m'),('rotation','tip_rotation'),('force','force_n'),('point','force_point_m'),('tension','tension_n'),('source_index','source_index')]:setattr(r,attr,a[key])
    with np.load(base/'unloaded_baseline.npz',allow_pickle=False) as a:r.unloaded_qpos=a['qpos'];r.unloaded_qvel=a['qvel'];r.unloaded_tip=a['tip_m']
    r.model_path=base/'scene.xml';r.base_path=base/'robot.xml';r.reference=np.empty((0,3));r.title=TITLES['phs']+' / '+r.case
    r.tip_delta=r.tip-r.unloaded_tip;r.origin_note='Codex-assisted engineering design / not Main autonomous Study'
    r.notes='Horizontal XY stabilization only; Z moves; no unloading record'
def material_point(m,d,geometry,binding):
    name='fixed_base' if binding['body']<0 else geometry['parts'][binding['body']]['entity'];bid=m.body(name).id
    return d.xpos[bid]+d.xmat[bid].reshape(3,3)@binding['position_m']
def centerline(m,d,geometry):
    # Same named-cell endpoints as the original phs2d_io.centerline.
    points=[d.xpos[m.body('fixed_base').id].copy()]
    for i,p in enumerate(geometry['parts']):
        if p['dofs']:points.append(material_point(m,d,geometry,dict(body=i,position_m=[p['length_m'],0,0])))
    points.append(material_point(m,d,geometry,geometry['tip']))
    return np.asarray(points)


def forbid_integration():
    """Same read-only guard as the original PHS saved-state tools."""
    def forbidden(*args,**kwargs):raise RuntimeError('Saved-state showcase: physics integration is forbidden')
    for name in ['mj_step','mj_step1','mj_step2']:setattr(mujoco,name,forbidden)
