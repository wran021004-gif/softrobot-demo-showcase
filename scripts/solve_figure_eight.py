"""One small four-tendon design, static SciPy solve, then one full MuJoCo rollout."""
import json, sys
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
from scipy.optimize import root, least_squares
from scipy.interpolate import CubicSpline

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'figure_eight_tracking'
def write_json(path,obj): path.write_text(json.dumps(obj,indent=2,allow_nan=False)+'\n',encoding='utf8')

def build():
    # Reuse the production structural resolver and MJCF compiler directly.
    # No Store, Main, Reporter, controller runtime or Provider is started.
    import os
    core=Path(os.environ.get('SOFTROBOT_CORE',r'D:\softrobot-agent-core'))
    sys.dont_write_bytecode=True
    sys.path.insert(0,str(core))
    from extensions.tendon_family.compiler import resolve
    from extensions.tendon_family.mjcf import compile_xml
    n=12;L=.27;radius=.008;E=4e6;rho=1050.;dx=L/n
    attachment=lambda part,s=0,pos=(0,0,0):dict(part=part,s=s,position_m=list(pos),quaternion_wxyz=[1,0,0,0])
    segment=dict(id='segment_0',kind='flexible_segment',connection=attachment('fixed_base'),length_m=L,sections=[dict(s=0,section=dict(kind='circle',parameters=dict(radius_m=radius)))],physics=dict(mode='material',density_kg_m3=rho,young_pa=E,bending_viscosity_nm2_s=[.025*dx,.025*dx]))
    payload=dict(id='payload',kind='payload',connection=attachment('segment_0',1),mass_kg=.002,com_local_m=[.007,0,0],inertia_com_local_kg_m2=np.diag([1e-8,4e-8,4e-8]).tolist(),envelope_halfsize_m=[.014,.003,.003])
    design=dict(id='figure_eight_single_segment',components=[segment,payload],tendons=[],actuators=[],tip=attachment('payload',pos=(.014,0,0)))
    for j,(y,z) in enumerate([(.012,0),(-.012,0),(0,.012),(0,-.012)]):
        points=[dict(role='start',attachment=attachment('fixed_base',pos=(0,y,z)))]
        for k in range(1,n+1):
            # Guide belongs to the preceding cell at an exact-grid joint.
            station=1. if k==n else k/n-1e-12
            points.append(dict(role='anchor' if k==n else 'guide',attachment=attachment('segment_0',station,(0,y,z))))
        design['tendons'].append(dict(id=f'tendon_{j}',points=points,diameter_m=.0013,length_servo_gain_n_m=900,pretension_n=.2,force_limit_n=8))
        design['actuators'].append(dict(id=f'actuator_{j}',command_type='displacement',units='m',limits=[-.015,.015],velocity_limit=.03,transmission=[dict(tendon=f'tendon_{j}',ratio=1)]))
    write_json(OUT/'design.json',design)
    physics=resolve(design,dict(cells=dict(segment_0=n)))
    scene=dict(timestep_s=.001,gravity=[0,0,-9.81],floor_id='floor',floor_z_m=-.02,mount_position=[0,0,.17],mount_rotation=np.eye(3),control=dict(tension_execution_mode='ideal_tension'))
    compile_xml(physics,scene,None,OUT/'robot.xml')
    tree=ET.parse(OUT/'robot.xml');r=tree.getroot();body=r.find('.//body[@name="payload"]')
    body.remove(body.find('geom'))
    ET.SubElement(body,'geom',name='pointer_shaft',type='capsule',fromto='0 0 0 .014 0 0',size='.002',rgba='.3 .35 .4 1')
    ET.SubElement(body,'geom',name='pointer_head',type='sphere',pos='.014 0 0',size='.003',rgba='.1 .9 .4 1')
    for e in r.findall('.//joint'):
        if e.get('name'):e.set('limited','true');e.set('range','-.3 .3')
    for e in r.findall('.//general'):e.set('ctrlrange','.2 8')
    ET.indent(r);tree.write(OUT/'robot.xml',encoding='unicode')
    I=np.pi*radius**4/4
    write_json(OUT/'structure.json',dict(segments=1,length_m=L,cells=n,cross_section='circular',radius_m=radius,young_modulus_pa=E,density_kg_m3=rho,bending_stiffness_nm2=E*I,joint_stiffness_nm_rad=E*I/dx,joint_damping_nms_rad=.025,tendon_radius_m=.012,independent_actuators=4,payload_mass_kg=.002,payload_com_m=[.007,0,0],payload_diagonal_inertia_kgm2=[1e-8,4e-8,4e-8],production_tools=['extensions.tendon_family.compiler.resolve','extensions.tendon_family.mjcf.compile_xml'],core_git_commit=__import__('subprocess').check_output(['git','-C',str(core),'rev-parse','HEAD'],text=True).strip(),post_compile_edits='pointer geometry with unchanged explicit inertia, 0.2 N task pretension bound and +/-0.3 rad joint limits'))
    return mujoco.MjModel.from_xml_path(str(OUT/'robot.xml'))

def allocation(v): return np.array([4+v[0],4-v[0],4+v[1],4-v[1]])
def phase(t):
    t=np.asarray(t); s=np.clip(t/2,0,1); end=np.clip((t-20)/2,0,1)
    integ=lambda a:2.5*a**4-3*a**5+a**6
    return .2*np.pi*np.where(t<2,2*integ(s),np.where(t<20,t-1,19+2*(end-integ(end))))

def main():
    m=build(); d=mujoco.MjData(m); tip=m.site('tip_site').id; q=np.zeros(m.nq)
    def equilibrium(v):
        nonlocal q
        u=allocation(v)
        def balance(x):
            d.qpos[:]=x; d.qvel[:]=0; d.ctrl[:]=u; mujoco.mj_forward(m,d)
            return d.qfrc_passive+d.qfrc_actuator-d.qfrc_bias
        sol=root(balance,q,tol=1e-9)
        if np.max(np.abs(balance(sol.x)))>1e-7:
            sol=least_squares(balance,np.clip(q,-.25,.25),bounds=(-.28,.28),gtol=1e-11,ftol=1e-11,xtol=1e-11,max_nfev=120)
            if np.max(np.abs(balance(sol.x)))>1e-7:raise RuntimeError(f'static equilibrium failed at {v}: {np.max(np.abs(balance(sol.x)))}')
        q=sol.x.copy(); return d.site_xpos[tip].copy()
    # x is allowed to differ slightly from the constant reference due to beam inextensibility.
    center=np.array([.282,0,.170]); A=.020; B=.008
    angles=np.linspace(0,2*np.pi,81); vv=[]; qs=[]; residual=[]; v=np.array([0.,0.])
    for a in angles:
        ref=center+np.array([0,A*np.sin(a),B*np.sin(2*a)])
        result=least_squares(lambda x:(equilibrium(x)-ref)*1000,v,bounds=(-3.6,3.6),diff_step=1e-4,xtol=1e-9,gtol=1e-8,ftol=1e-9,max_nfev=40)
        v=result.x; pos=equilibrium(v);vv.append(v.copy());qs.append(q.copy());residual.append(float(np.linalg.norm(pos-ref)))
    vv[-1]=vv[0]; spline=CubicSpline(angles,np.array(vv),bc_type='periodic')
    np.savez_compressed(OUT/'feedforward.npz',phase_rad=angles,differential_tension_n=vv,equilibrium_qpos=qs)
    equilibrium(vv[0]); d.qpos[:]=q; d.qvel[:]=0; d.time=0; d.ctrl[:]=allocation(vv[0]);mujoco.mj_forward(m,d)
    states=[]; velocity=[]; tension=[]; lengths=[]; refs=[]; tips=[]; times=[]; contacts=0
    for k in range(22001):
        if k:
            d.ctrl[:]=allocation(spline(float(phase(d.time+.0005))%(2*np.pi)))
            mujoco.mj_step(m,d)
        mujoco.mj_forward(m,d)
        ref=center+np.array([0,A*np.sin(phase(d.time)),B*np.sin(2*phase(d.time))])
        times.append(float(d.time)); states.append(d.qpos.copy());velocity.append(d.qvel.copy()); tension.append(-d.actuator_force.copy()); lengths.append(d.ten_length.copy());refs.append(ref);tips.append(d.site_xpos[tip].copy());contacts=max(contacts,d.ncon)
    times=np.array(times); tension=np.array(tension);lengths=np.array(lengths);refs=np.array(refs); tips=np.array(tips);err=np.linalg.norm(tips-refs,axis=1)
    np.savez_compressed(OUT/'trajectory_full.npz',time_s=times,qpos=states,qvel=velocity,tension_n=tension,tendon_length_m=lengths,reference_m=refs,tip_m=tips)
    # Portable XYZ trace plus complete qpos/qvel/actuator history in NPZ, every physical timestep.
    np.savetxt(OUT/'trajectory.csv',np.column_stack([times,refs,tips,err]),delimiter=',',header='time_s,ref_x_m,ref_y_m,ref_z_m,tip_x_m,tip_y_m,tip_z_m,error_m',comments='',fmt='%.10g')
    speed=np.gradient(lengths,times,axis=0); stroke=np.ptp(lengths,axis=0)
    rmse=lambda e:float(np.sqrt(np.mean(e**2)))
    # Phase-matched comparison, including ramps, interpolates the monotone phase grid.
    ph=phase(times); mask=(ph>1e-6)&(ph<2*np.pi-1e-6); p1=ph[mask]; t1=tips[mask];t2=np.column_stack([np.interp(p1+2*np.pi,ph,tips[:,a]) for a in range(3)])
    limits=dict(tension_n=[.2,8],stroke_m=.030,speed_m_s=.030,tracking_rmse_m=.003,tracking_max_m=.006,cycle_drift_m=.002)
    sat=((tension<=.2001)|(tension>=7.9999)).mean(axis=0)
    metrics=dict(tracking_rmse_m=rmse(err),max_error_m=float(err.max()),p95_error_m=float(np.percentile(err,95)),completed_cycles=float(ph[-1]/(2*np.pi)),duration_s=float(times[-1]),physical_steps=22000,static_max_error_m=max(residual),max_tension_n=tension.max(axis=0).tolist(),min_tension_n=tension.min(axis=0).tolist(),stroke_m=stroke.tolist(),max_speed_m_s=np.max(np.abs(speed),axis=0).tolist(),tension_saturation_fraction=sat.tolist(),cycle_phase_matched_max_difference_m=float(np.linalg.norm(t1-t2,axis=1).max()),terminal_speed_m_s=float(np.linalg.norm((tips[-1]-tips[-2])/.001)),contacts=contacts,solver_warnings=[int(w.number) for w in d.warning])
    checks=dict(two_cycles=abs(metrics['completed_cycles']-2)<1e-6,tracking=metrics['tracking_rmse_m']<.003 and metrics['max_error_m']<.006,no_saturation=not sat.any(),stroke=bool((stroke<.03).all()),speed=bool((np.max(abs(speed),axis=0)<.03).all()),no_drift=metrics['cycle_phase_matched_max_difference_m']<.002,no_contacts_or_warnings=not contacts and not any(w.number for w in d.warning))
    bounds=[]
    for start,end in [(0,11),(11,22.001)]:
        xyz=tips[(times>=start)&(times<end)];bounds.append(dict(min_m=xyz.min(axis=0).tolist(),max_m=xyz.max(axis=0).tolist()))
    metrics['actual_cycle_bounds']=bounds
    checks['actual_two_lobes_each_cycle']=all(b['min_m'][1]<-.018 and b['max_m'][1]>.018 and b['min_m'][2]<.163 and b['max_m'][2]>.177 for b in bounds)
    write_json(OUT/'task.json',dict(task='figure_eight_yz',center_m=center.tolist(),amplitude_y_m=A,amplitude_z_m=B,frequency_hz=.10,cycles=2,start_ramp_s=2,stop_ramp_s=2,duration_s=22,phase='integral of quintic smoothstep speed ramps; constant 0.10 Hz in between',initial_state='prepared equilibrium at phase zero, optimized preload, zero velocity',limits=limits))
    write_json(OUT/'results.json',dict(status='PASS' if all(checks.values()) else 'FAIL',checks=checks,metrics=metrics,control='periodic cubic-spline quasistatic feedforward, 4 antagonistic ideal tension channels, no applied tip force',scope='single nominal MuJoCo run; no friction, spool or motor dynamics validation'))
    print(json.dumps(dict(checks=checks,metrics=metrics),indent=2),flush=True)
if __name__=='__main__': main()
