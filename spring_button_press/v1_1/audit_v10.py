"""Read-only audit of stored v1.0 data. No integration or old search rerun."""
import sys,json
from pathlib import Path
import numpy as np
import mujoco
HERE=Path(__file__).resolve().parent;BASE=HERE.parent
sys.path.insert(0,str(BASE))
import environment as v10
from solver import Controller

def main():
    report={'method':'read existing trajectories; mj_forward only; zero integration steps','candidates':{}}
    for name in ['B4','B5']:
        p=BASE/'candidates'/name;a=np.load(p/'trajectory.npz');config=v10.read(p/'control.json')
        design=v10.read(p/'design.json');optimization=v10.read(p/'optimization.json')
        m=v10.load_model(path=p/'environment.xml');ff=np.load(p/'feedforward.npz')['knots'];h=(a['time_s']>=.9-1e-9)&(a['time_s']<=1.4+1e-9)
        error=a['probe_m'][:,[0,2]]-[.2955,.15]
        i,j=np.unravel_index(abs(a['tendon_speed_m_s']).argmax(),a['tendon_speed_m_s'].shape)
        byphase={}
        for label,start,end in [('Approach',0,.5),('Press',.5,.9),('Hold',.9,1.4),('Release',1.4,1.8),('Settle',1.8,2.0001)]:
            mask=(a['time_s']>=start-1e-9)&(a['time_s']<end-1e-9);byphase[label]=float(abs(a['tendon_speed_m_s'][mask]).max()*1000)
        config_checks={}
        for flag in [False,True]:
            c=Controller(m,np.array([.2955,.021,.15]),ff,config['feedback_gain']);c.use_x=flag;c.contact_ff=False;errs=[]
            for k in [100,300,500,800,1000,1200,1500,1700]:
                d=mujoco.MjData(m);d.qpos[:]=a['qpos'][k];d.qvel[:]=a['qvel'][k];d.ctrl[:]=a['control_n'][k];d.time=a['time_s'][k];mujoco.mj_forward(m,d)
                errs.append(float(abs(c.command(d)-a['control_n'][k+1]).max()))
            config_checks[str(flag)]=max(errs)
        decomposed=[]
        for k in np.flatnonzero(h)[::10]:
            d=mujoco.MjData(m);d.qpos[:]=a['qpos'][k];mujoco.mj_forward(m,d);bid=m.body('payload').id;R=d.xmat[bid].reshape(3,3)
            # Original nominal payload origin X = total flexible length + 14 mm.
            L=sum(c.get('length_m',0) for c in design['components'] if c['kind']=='flexible_segment')
            decomposed.append([d.xpos[bid,0]-(L+.014),.014*(R[0,0]-1),.021*R[0,1]])
        z=np.array(decomposed)*1000;du=np.diff(a['control_n'],axis=0);ii,jj=np.unravel_index(abs(du).argmax(),du.shape);q=a['button_q_m'];qmin=int(q.argmin())
        startup=[]
        for k in range(0,151,10):startup.append({'time_s':float(a['time_s'][k]),'command_n':a['control_n'][min(k+1,2000)].tolist(),'rope_speed_mm_s':(a['tendon_speed_m_s'][k]*1000).tolist()})
        sections=[]
        from extensions.tendon_family.sections import properties
        for comp in design['components']:
            if comp['kind']=='flexible_segment':
                props=properties(comp['sections'][0]['section']);sections.append({'segment':comp['id'],'young_pa':comp['physics']['young_pa'],'section':comp['sections'][0]['section'],'properties':props})
        report['candidates'][name]={'stored_control':config,'hold_x_error_min_max_mm':(np.array([error[h,0].min(),error[h,0].max()])*1000).tolist(),
            'hold_z_error_min_max_mm':(np.array([error[h,1].min(),error[h,1].max()])*1000).tolist(),
            'mean_x_error_decomposition_mm':dict(zip(['payload_centerline_shortening','axial_tool_rotation','side_probe_offset_rotation'],z.mean(axis=0).tolist())),
            'max_rope_speed':{'time_s':float(a['time_s'][i]),'tendon':f'T{j}','signed_mm_s':float(a['tendon_speed_m_s'][i,j]*1000)},
            'phase_peak_rope_speed_mm_s':byphase,'command_reconstruction_max_error_n':config_checks,
            'largest_command_jump':{'time_s':float(a['time_s'][ii+1]),'tendon':f'T{jj}','jump_n':float(du[ii,jj])},
            'negative_slider':{'time_s':float(a['time_s'][qmin]),'travel_mm':float(q[qmin]*1000),'last_contact_s':float(a['time_s'][np.flatnonzero(a['contact_pairs']>0)[-1]])},
            'startup_trace':startup,'sections':sections,'inverse_hold_initializer':optimization['knots'][-1]}
    # JSON conversion for NumPy section-property arrays.
    (HERE/'audit_v10.json').write_text(json.dumps(report,indent=2,default=lambda x:np.asarray(x).tolist())+'\n',encoding='utf8')
    print(json.dumps({k:{kk:vv for kk,vv in v.items() if kk in ['mean_x_error_decomposition_mm','max_rope_speed','phase_peak_rope_speed_mm_s','command_reconstruction_max_error_n']} for k,v in report['candidates'].items()},indent=2))
if __name__=='__main__':main()
