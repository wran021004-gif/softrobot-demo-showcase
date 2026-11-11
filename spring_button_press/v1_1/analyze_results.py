"""Stored-result analysis and necessary routing/config checks; no integration."""
import json
import numpy as np
import mujoco
from mechanics import HERE,BASE,read,write,load_model,LABEL
from extensions.tendon_family.compiler import resolve

def distance_segments(a,b,c,d):
    # Closest points of two bounded 3D line segments.
    u=b-a;v=d-c;w=a-c;aa=u@u;bb=u@v;cc=v@v;dd=u@w;ee=v@w;D=aa*cc-bb*bb
    if D<1e-20:s=0.;t=np.clip(ee/max(cc,1e-20),0,1)
    else:s=np.clip((bb*ee-cc*dd)/D,0,1);t=np.clip((aa*ee-bb*dd)/D,0,1)
    for _ in range(3):s=np.clip((bb*t-dd)/max(aa,1e-20),0,1);t=np.clip((bb*s+ee)/max(cc,1e-20),0,1)
    return float(np.linalg.norm(w+s*u-t*v))

def inspect_routes():
    folder=HERE/'candidates/S4';m=load_model(path=folder/'environment.xml');a=np.load(folder/'trajectory.npz');info=read(folder/'mechanics.json');design=read(folder/'design.json')
    physics=resolve(design,{'cells':{'segment_0':12,'segment_1':12},'model':'serial_bending_cells_v1','version':'serial_bending_discretization_v1','coordinates':'two_principal_bending_angles_per_cell'})
    selected=sorted(set([0,60,399,454,900,1200,1400,1550,1600,1750,1800,2000,int(np.argmax(np.max(abs(a['qpos'][:,:-1]),axis=1)))]))
    maxerror=0.;minbore=1.;mincross=1.;minbody=1.;mindistances=[]
    for k in selected:
        d=mujoco.MjData(m);d.qpos[:]=a['qpos'][k];d.qvel[:]=a['qvel'][k];d.ctrl[:]=a['control_n'][k];mujoco.mj_forward(m,d)
        paths=[]
        for i in range(4):paths.append([d.site_xpos[m.site(e.get('site')).id].copy() for e in __import__('xml.etree.ElementTree',fromlist=['']).parse(folder/'environment.xml').getroot().find(f'tendon/spatial[@name="tendon_{i}"]')])
        for g in info['guides']:
            if not g['routing_site']:continue
            p=d.site_xpos[m.site(g['routing_site']).id];q=d.site_xpos[m.site(g['name']+'_hole_center').id];maxerror=max(maxerror,float(np.linalg.norm(p-q)))
            i=int(g['tendon'][1:]);idx=next(j for j in range(len(paths[i])) if np.linalg.norm(paths[i][j]-p)<1e-10)
            axis=d.xmat[m.body(g['parent_body']).id].reshape(3,3)[:,0]
            for j in [idx-1,idx+1]:
                if 0<=j<len(paths[i]):
                    v=paths[i][j]-p;cosine=abs(v@axis)/np.linalg.norm(v);tan=np.sqrt(max(0,1-cosine*cosine))/max(cosine,1e-9)
                    minbore=min(minbore,.0007-.0005-.0005*tan)
        for i in range(4):
            for j in range(i+1,4):
                for p,q in zip(paths[i][:-1],paths[i][1:]):
                    for r,s in zip(paths[j][:-1],paths[j][1:]):mincross=min(mincross,distance_segments(p,q,r,s)-.001)
            for p,q in zip(paths[i][:-1],paths[i][1:]):
                for part in physics['parts']:
                    if not part['section_properties']:continue
                    bid=m.body(part['entity']).id;axis=d.xmat[bid].reshape(3,3)[:,0];start=d.xpos[bid];end=start+axis*part['length_m']
                    radius=np.linalg.norm(part['section_properties']['outer_yz_m'],axis=1).max()
                    minbody=min(minbody,distance_segments(p,q,start,end)-radius-.0005)
    return {'scope':'13 selected actual saved poses, including initial/peak speed/early contact/rebound/Hold/Release/max bend; mj_forward only',
        'pose_indices':selected,'max_active_hole_center_error_m':maxerror,'minimum_bore_radial_clearance_estimate_m':minbore,
        'minimum_different_tendon_centerline_clearance_after_diameter_m':mincross,
        'minimum_tendon_to_flexible_enclosing_axis_capsule_clearance_m':float(minbody),
        'interpretation':'finite sampled geometric checks, not a continuous swept/friction or manufacturing certificate'}

def main():
    rows=[]
    for name in ['C1_A','C1_B','S1','S2','S3','S4']:
        p=HERE/'candidates'/name;a=np.load(p/'trajectory.npz');res=read(p/'result.json');cfg=read(p/'effective_control.json');trace=read(p/'controller_trace.json')
        max_sensor_error=0.;weights=[]
        for j,item in enumerate(trace):
            k=j*10;max_sensor_error=max(max_sensor_error,abs(item['q_measured_m']-a['button_q_m'][k]),abs(item['qdot_measured_m_s']-a['button_qdot_m_s'][k]),abs(item['contact_measured_n']-a['contact_force_n'][k]));weights.append(item['position_weights'])
        i,tendon=np.unravel_index(abs(a['tendon_speed_m_s']).argmax(),a['tendon_speed_m_s'].shape);n=int(a['button_q_m'].argmin());release=a['time_s']>=1.4-1e-9
        runtime=read(p/'runtime_configuration.json');effective_weights=runtime['position_weights']
        extended={'peak_rope_time_s':float(a['time_s'][i]),'peak_rope_tendon':f'T{tendon}','peak_rope_signed_mm_s':float(a['tendon_speed_m_s'][i,tendon]*1000),
            'minimum_button_travel_mm':float(a['button_q_m'][n]*1000),'minimum_button_time_s':float(a['time_s'][n]),
            'minimum_release_travel_mm':float(a['button_q_m'][release].min()*1000),
            'actual_max_command_slope_n_s':float(abs(np.diff(a['control_n'],axis=0)).max()/.001),
            'configuration_sensor_max_error':float(max_sensor_error),'actual_position_weights':effective_weights,
            'diagnostic_trace_position_weights':weights[0],
            'trace_weights_logging_defect':not np.array_equal(weights[0],effective_weights),
            'logging_defect_scope':'S2-S4 originally returned a hardcoded ones vector in the diagnostic trace; allocation and runtime summary used the configured positive weights. Original logs/source are preserved; no physical states or controls changed. Current diagnostic writer is fixed.',
            'all_xyz_weights_positive':bool(np.min(effective_weights)>0),'effective_configuration':cfg,
            'rate_limit_scope':'target-knot change / 10 ms; quintic interpolation can have a larger instantaneous slope; this is not the original rope speed limit'}
        write(p/'execution_audit.json',extended);rows.append({'id':name,'status':res['status'],'failed_checks':[k for k,v in res['checks'].items() if not v],'metrics':res['metrics'],'execution_audit':extended})
    write(HERE/'results/all_candidates.json',{'label':LABEL,'candidates':rows,'selected':'S4','selection':'fewest failed original hard checks; remaining violations reported without relaxing criteria'})
    route=inspect_routes();write(HERE/'results/routing_checks.json',route)
    fields=[('Overall status',None,1),('Max Hold depth error (mm)','max_hold_depth_error_m',1000),('Max Hold force error (N)','max_hold_contact_force_error_n',1),
        ('Max Hold XZ error (mm)','hold_lateral_error_max_m',1000),('Valid continuous Hold (s)','hold_valid_continuous_s',1),
        ('Max tension (N)','max_tension_n',1),('Max actual rope speed (mm/s)','max_rope_speed_m_s',1000),('Max stroke (mm)','max_stroke_m',1000),('Total structure mass (g)','structure_mass_kg',1000)]
    old=[read(BASE/f'candidates/{n}/result.json') for n in ['B4','B5']];new=read(HERE/'candidates/S4/result.json');lines=['# v1.0 versus v1.1 results','',LABEL,'','| Metric | v1.0 B4 | v1.0 B5 | v1.1 S4 |','|---|---:|---:|---:|']
    for label,key,scale in fields:
        vals=[r['status'] if key is None else f'{r["metrics"][key]*scale:.6g}' for r in [*old,new]];lines.append('| '+' | '.join([label,*vals])+' |')
    lines+=['','S4 remains FAIL: 27.832 mm/s exceeds 25 mm/s, and the -0.005531 mm global / -0.003952 mm Release lower-stop excursions exceed the unchanged -0.001 mm tolerance. Its complete 0.5 s Hold and both Release endpoint checks pass.','', '| Candidate | Status | Failed original checks |','|---|---|---|']
    for row in rows:lines.append('| '+row['id']+' | '+row['status']+' | '+', '.join(row['failed_checks'])+' |')
    (HERE/'results/comparison.md').write_text('\n'.join(lines)+'\n',encoding='utf8')
    usage=read(HERE/'results/usage.json');usage.update(provider_calls=0,main_calls=0,reporter_calls=0,
        no_additional_integration=True,classification=LABEL,algebraic_forward_evaluations=sum(read(HERE/f'candidates/{n}/optimization.json')['algebraic_forward_evaluations'] for n in ['C1_A','C1_B','S1','S2','S3','S4']))
    write(HERE/'results/budget_usage.json',usage)
    print('Stored-state route check:',route);print('Budget:',usage)
if __name__=='__main__':main()
