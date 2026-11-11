"""Small task adapter around Core compilation and SciPy bounded optimization.
No Main, Reporter, Provider, force injection or prescribed slider trajectory.
Static inverse calculations are mj_forward only; rollouts integrate native contact.
"""
import argparse, copy, time
from pathlib import Path
import numpy as np
import mujoco
from scipy.optimize import least_squares, lsq_linear
from environment import HERE, ROOT, assemble, read, write, LABEL
from evaluator import contact_state,evaluate

def smooth(x):x=np.clip(x,0,1);return x*x*x*(10+x*(-15+6*x))

def reference(t,origin):
    p=origin.copy()
    if t<.5:p[1]+=.005*smooth(t/.5)
    elif t<.9:p[1]+=.005+.002*smooth((t-.5)/.4)
    elif t<1.4:p[1]+=.007
    elif t<1.74:p[1]+=.007*(1-smooth((t-1.4)/.34))
    return p

def dense_moments(m,d):
    a=np.zeros((m.nu,m.nv));mujoco.mju_sparse2dense(a,d.actuator_moment,d.moment_rownnz,d.moment_rowadr,d.moment_colind)
    return -a.T

def feedforward(m,origin):
    """Three bounded inverse equilibria, using the assembled contact model.
    Slider state is an optimization unknown here, never prescribed in rollouts.
    """
    d=mujoco.MjData(m);sid=m.site('probe_center').id;n=m.nq
    qi=m.jnt_qposadr[m.joint('button_slide').id];x=np.r_[np.zeros(n),np.ones(4)*.8]
    lo=np.r_[np.full(n,-.35),np.full(4,.2)];hi=np.r_[np.full(n,.35),np.full(4,8.)]
    lo[qi]=0;hi[qi]=.003
    solutions=[];records=[];calls=0
    for advance in [0,.005,.007]:
        desired=origin+np.array([0,advance,0])
        if advance:x[qi]=max(1e-10,advance-.005)
        def residual(z):
            nonlocal calls
            calls+=1;d.qpos[:]=z[:n];d.qvel[:]=0;d.ctrl[:]=z[n:];d.qacc_warmstart[:]=0
            mujoco.mj_forward(m,d)
            balance=d.qfrc_passive+d.qfrc_actuator+d.qfrc_constraint-d.qfrc_bias
            # Nm for robot hinges; N for slider. High equilibrium weight.
            scale=np.full(n,300.);scale[qi]=20.
            pos=d.site_xpos[sid]-desired
            return np.r_[balance*scale,pos*np.array([30,1500,1500]),.001*(z[n:]-.2)]
        sol=least_squares(residual,np.clip(x,lo+1e-10,hi-1e-10),bounds=(lo,hi),
            xtol=1e-9,ftol=1e-9,gtol=1e-8,max_nfev=90,diff_step=1e-4)
        x=sol.x.copy();residual(x);solutions.append(x.copy())
        records.append({'advance_m':advance,'cost':float(sol.cost),'nfev':sol.nfev,
            'probe_m':d.site_xpos[sid].tolist(),'button_q_m':float(d.qpos[qi]),
            'tensions_n':x[n:].tolist(),'max_scaled_residual':float(abs(sol.fun).max())})
    return np.array(solutions),{'knots':records,'forward_evaluations':calls}

def refine_contact(m,origin,ff):
    """Resolve the nonsmooth contact boundary from a penetrating seed.
    All optimization states are scratch MjData, never an integrated run.
    """
    n=m.nq;d=mujoco.MjData(m);sid=m.site('probe_center').id
    x=ff[-1].copy();x[n-1]=.002
    d.qpos[:]=x[:n];d.ctrl[:]=x[n:];mujoco.mj_forward(m,d)
    J=np.zeros((3,m.nv));mujoco.mj_jacSite(m,d,J,None,sid)
    desired=origin+np.array([-.0007,.007,0])
    x[:n-1]+=np.linalg.pinv(J[:,:n-1])@(desired+np.array([0,.000015,0])-d.site_xpos[sid])
    # A feasible unilateral tension allocation is part of the algebraic seed.
    B=dense_moments(m,d);K=np.diag(m.jnt_stiffness)
    G=J@np.linalg.solve(K,B);C=J@np.linalg.solve(K,J.T)
    du=lsq_linear(G[[1,2]],(C@np.array([0,.1,0]))[[1,2]],bounds=(.2-x[n:],8-x[n:])).x
    x[n:]+=du
    lo=np.r_[np.full(n,-.35),np.full(4,.2)];hi=np.r_[np.full(n,.35),np.full(4,8.)];lo[n-1]=0;hi[n-1]=.003
    calls=0
    def residual(z):
        nonlocal calls
        calls+=1;d.qpos[:]=z[:n];d.qvel[:]=0;d.ctrl[:]=z[n:];d.qacc_warmstart[:]=0;mujoco.mj_forward(m,d)
        bal=d.qfrc_passive+d.qfrc_actuator+d.qfrc_constraint-d.qfrc_bias
        scale=np.full(n,1000.);scale[-1]=100.
        force,*_=contact_state(m,d)
        return np.r_[bal*scale,(d.site_xpos[sid]-desired)*1000,(d.qpos[-1]-.002)*1000,(force-.1)*20,.0001*(z[n:]-.2)]
    sol=least_squares(residual,np.clip(x,lo+1e-9,hi-1e-9),bounds=(lo,hi),diff_step=1e-4,max_nfev=150,
        xtol=1e-10,ftol=1e-10,gtol=1e-9)
    residual(sol.x);out=ff.copy();out[-1]=sol.x
    info={'cost':float(sol.cost),'nfev':sol.nfev,'forward_evaluations':calls,'probe_m':d.site_xpos[sid].tolist(),
        'button_q_m':float(d.qpos[-1]),'contact_n':contact_state(m,d)[0],'tensions_n':sol.x[n:].tolist(),
        'max_equilibrium_residual':float(np.max(abs(d.qfrc_passive+d.qfrc_actuator+d.qfrc_constraint-d.qfrc_bias)))}
    return out,info

class Controller:
    def __init__(self,m,origin,ff,gain=1.):
        self.m=m;self.origin=origin;self.ff=ff;self.gain=gain
        self.sid=m.site('probe_center').id;self.last=np.ones(m.nu)*.2
        self.J=np.zeros((3,m.nv))
        # Quasistatic stiffness maps provide task-space tension feedback.
        self.maps=[];self.compliance=[]
        for x in ff:
            d=mujoco.MjData(m);d.qpos[:]=x[:m.nq];d.ctrl[:]=x[m.nq:];mujoco.mj_forward(m,d)
            mujoco.mj_jacSite(m,d,self.J,None,self.sid)
            B=dense_moments(m,d);K=np.diag(m.jnt_stiffness[:m.nv])
            # Model has one dof per joint; slider stiffness is included.
            self.maps.append(self.J@np.linalg.solve(K+np.eye(m.nv)*1e-9,B))
            self.compliance.append(self.J@np.linalg.solve(K+np.eye(m.nv)*1e-9,self.J.T))

    def command(self,d):
        t=float(d.time);p=reference(t,self.origin);advance=p[1]-self.origin[1]
        s=np.clip(advance/.005,0,1) if advance<=.005 else np.clip((advance-.005)/.002,0,1)
        k=0 if advance<=.005 else 1
        u=(1-s)*self.ff[k,self.m.nq:]+s*self.ff[k+1,self.m.nq:]
        G=(1-s)*self.maps[k]+s*self.maps[k+1]
        C=(1-s)*self.compliance[k]+s*self.compliance[k+1]
        mujoco.mj_jacSite(self.m,d,self.J,None,self.sid);v=self.J@d.qvel
        # Position and velocity feedback; 3x4 bounded tension allocation.
        error=p-d.site_xpos[self.sid]
        desired_reaction=0 if getattr(self,'contact_ff',False) else 50*max(advance-.005,0)
        correction=self.gain*(error-.075*v)+C@np.array([0,desired_reaction,0])
        weights=np.array([1.,1.,1.]) if getattr(self,'use_x',False) else np.array([0.,1.,1.])
        allocation=lsq_linear(np.vstack([G*weights[:,None],np.eye(4)*.00008]),
            np.r_[correction*weights,np.zeros(4)],bounds=(.2-u,8.-u),tol=1e-10)
        u=np.clip(u+allocation.x,.2,8.)
        if t<.015:u=.2+smooth(t/.015)*(u-.2)
        self.last=u
        return u

def rollout(m,origin,ff,gain=1.,duration=2.,contact_ff=False,use_x=False,controls=None):
    d=mujoco.MjData(m);d.ctrl[:]=.2;mujoco.mj_forward(m,d)
    c=Controller(m,origin,ff,gain) if controls is None else None
    if c is not None:c.contact_ff=contact_ff;c.use_x=use_x
    sid=m.site('probe_center').id
    j=m.joint('button_slide').id;qi=m.jnt_qposadr[j];vi=m.jnt_dofadr[j]
    cap=m.geom('button_cap').id;capbody=m.body('button_slider').id
    records={k:[] for k in ['time_s','qpos','qvel','control_n','tension_n','tendon_length_m','tendon_speed_m_s',
        'probe_m','reference_m','tip_speed_m_s','button_q_m','button_qdot_m_s','button_qacc_m_s2',
        'contact_force_n','contact_pairs','contact_position_m','lateral_error_m','unexpected_contact_count',
        'warnings','external_force_max','button_axis_contact_n']}
    unwanted=set();J=np.zeros((3,m.nv))
    for k in range(round(duration/.001)+1):
        if k:
            # Only controls are changed during the trajectory; all qpos/qvel
            # evolution, including slider travel, comes from mj_step.
            if controls is not None:d.ctrl[:]=controls[k]
            elif (k-1)%10==0:d.ctrl[:]=c.command(d)
            mujoco.mj_step(m,d)
        mujoco.mj_forward(m,d)
        force,pairs,bad,cp=contact_state(m,d);unwanted.update(tuple(x) for x in bad)
        mujoco.mj_jacSite(m,d,J,None,sid);v=J@d.qvel;pos=d.site_xpos[sid]
        axisforce=0.
        for ic in range(d.ncon):
            con=d.contact[ic]
            if {m.geom(int(con.geom1)).name,m.geom(int(con.geom2)).name}=={'probe','button_cap'}:
                f=np.zeros(6);mujoco.mj_contactForce(m,d,ic,f)
                world=con.frame.reshape(3,3).T@f[:3]
                axisforce+=float(world[1])*(1 if m.geom_bodyid[con.geom2]==capbody else -1)
        vals=[d.time,d.qpos.copy(),d.qvel.copy(),d.ctrl.copy(),-d.actuator_force.copy(),d.ten_length.copy(),d.ten_velocity.copy(),
            pos.copy(),reference(d.time,origin),np.linalg.norm(v),d.qpos[qi],d.qvel[vi],d.qacc[vi],
            force,pairs,cp,np.linalg.norm((pos-d.geom_xpos[cap])[[0,2]]),len(bad),
            sum(w.number for w in d.warning),max(np.max(abs(d.xfrc_applied)),np.max(abs(d.qfrc_applied))),axisforce]
        for key,val in zip(records,vals):records[key].append(val)
    return {k:np.asarray(v) for k,v in records.items()},sorted(unwanted)

def save_run(folder,m,a,bad,info):
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=True)
    robot_mass=sum(m.body_mass[i] for i in range(1,m.nbody) if m.body(i).name not in ['button_slider','button_housing'])
    result=evaluate(a,m,robot_mass);result.update(label=LABEL,unexpected_pairs=bad,optimization=info)
    np.savez_compressed(folder/'trajectory.npz',**a)
    keys=['time_s','button_q_m','button_qdot_m_s','contact_force_n','lateral_error_m','tip_speed_m_s']
    np.savetxt(folder/'trace.csv',np.column_stack([a[k] for k in keys]),delimiter=',',header=','.join(keys),comments='')
    write(folder/'result.json',result);return result

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--precheck',action='store_true');args=ap.parse_args()
    if args.precheck and (HERE/'prechecks.json').exists():raise SystemExit('The two prechecks are already consumed; do not repeat them.')
    env=read(HERE/'environment.json');origin=np.array(env['reference_probe_center_m'])
    m,_,_=assemble()
    ff,info=feedforward(m,origin);np.savez_compressed(HERE/'precheck_feedforward.npz',knots=ff)
    write(HERE/'precheck_static.json',info)
    if args.precheck:
        # Precheck 1: robot and button have no imposed motion; verify free button.
        d=mujoco.MjData(m);d.ctrl[:]=.2;mujoco.mj_forward(m,d);maxq=0
        for _ in range(100):mujoco.mj_step(m,d);maxq=max(maxq,abs(float(d.qpos[-1])))
        # Precheck 2: physical C1 contact approach, same environment/controller.
        a,bad=rollout(m,origin,ff,duration=.6)
        write(HERE/'prechecks.json',{'count':2,'free_button_max_q_m':maxq,
            'probe_contact_detected':bool(a['contact_pairs'].max()),'max_button_q_m':float(a['button_q_m'].max()),
            'unexpected_pairs':bad,'physics_steps':700,'thresholds_adjusted':False})
        np.savez_compressed(HERE/'precheck_contact.npz',**a)
        print(read(HERE/'prechecks.json'),flush=True)

if __name__=='__main__':main()
