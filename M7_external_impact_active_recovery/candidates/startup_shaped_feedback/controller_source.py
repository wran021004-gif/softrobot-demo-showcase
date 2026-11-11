"""Task-local native-state feedback. No disturbance input or impact clock.

The existing Core GVS NMPC interface was inspected; its CasADi dependency is
absent from the showcase runtime. This small controller uses the same ideal
tension convention, native MuJoCo state and SciPy bounded allocation instead.
"""
import numpy as np
import mujoco
from scipy.optimize import least_squares,minimize,Bounds,LinearConstraint
from scipy.linalg import expm,solve_discrete_are

def moments(m,d):
    z=np.zeros((m.nu,m.nv));mujoco.mju_sparse2dense(z,d.actuator_moment,d.moment_rownnz,d.moment_rowadr,d.moment_colind)
    return -z.T

def prepare(m,target,cfg=None):
    """Algebraic equilibrium and linear feedback only; zero integration."""
    d=mujoco.MjData(m);sid=m.site('tip_site').id;n=m.nv
    z=np.r_[np.zeros(n),np.full(m.nu,1.)];lo=np.r_[np.full(n,-.4),np.full(m.nu,.001)];hi=np.r_[np.full(n,.4),np.full(m.nu,7.5)]
    rows=[];records=[];cfg=cfg or {}
    mujoco.mj_forward(m,d);origin=d.site_xpos[sid].copy();initial_lengths=d.ten_length.copy()
    for point in [origin,np.asarray(target)]:
        def residual(x):
            d.qpos[:]=x[:n];d.qvel[:]=0;d.ctrl[:]=x[n:];mujoco.mj_forward(m,d)
            stroke=cfg.get('bank_stroke_weight',0.)*(d.ten_length-initial_lengths)
            stroke=np.sign(stroke)*abs(stroke)**cfg.get('bank_stroke_power',1)
            return np.r_[(d.qfrc_passive+d.qfrc_actuator-d.qfrc_bias)*1000,(d.site_xpos[sid]-point)*1500,cfg.get('bank_tension_weight',.0005)*x[n:],stroke]
        sol=least_squares(residual,z,bounds=(lo,hi),max_nfev=120,ftol=1e-11,xtol=1e-11,gtol=1e-9);z=sol.x;residual(z)
        rows.append(z.copy());records.append({'point_m':point.tolist(),'tip_m':d.site_xpos[sid].tolist(),'cost':float(sol.cost),'nfev':sol.nfev,'tension_n':z[n:].tolist(),'balance_max_nm':float(abs(sol.fun[:n]).max()/1000)})
    # Full native tangent at the target, including gravity and tendon geometry.
    eq=rows[-1];d.qpos[:]=eq[:n];d.ctrl[:]=eq[n:];mujoco.mj_forward(m,d)
    J=np.zeros((3,n));mujoco.mj_jacSite(m,d,J,None,sid);B=moments(m,d);M=np.empty((n,n));mujoco.mj_fullM(m,d,M)
    K=np.zeros((n,n));h=1e-5
    def force(q):
        d.qpos[:]=q;mujoco.mj_forward(m,d);return d.qfrc_passive+d.qfrc_actuator-d.qfrc_bias
    for i in range(n):
        delta=np.zeros(n);delta[i]=h;K[:,i]=-(force(eq[:n]+delta)-force(eq[:n]-delta))/(2*h)
    D=np.diag(m.dof_damping)
    A=np.block([[np.zeros((n,n)),np.eye(n)],[-np.linalg.solve(M,K),-np.linalg.solve(M,D)]])
    U=np.vstack([np.zeros((n,m.nu)),np.linalg.solve(M,B)])
    aug=np.zeros((2*n+m.nu,2*n+m.nu));aug[:2*n,:2*n]=A;aug[:2*n,2*n:]=U;disc=expm(aug*.01)
    d.qpos[:]=eq[:n];mujoco.mj_forward(m,d);T=np.zeros((m.ntendon,n));mujoco.mju_sparse2dense(T,d.ten_J,m.ten_J_rownnz,m.ten_J_rowadr,m.ten_J_colind)
    return {'equilibria':np.array(rows),'A':disc[:2*n,:2*n],'B':disc[:2*n,2*n:],'J':J,'T':T,'origin':origin}, {'equilibrium_nodes':records,'method':'native algebraic balance plus full force tangent and exact linear discretization; no physics rollout'}

class Controller:
    def __init__(self,m,bank,target,cfg):
        self.m=m;self.bank=bank;self.target=np.asarray(target);self.cfg=dict(cfg);self.d=mujoco.MjData(m);n=m.nv
        Q=np.zeros((2*n,2*n));Q[:n,:n]=bank['J'].T@bank['J']/cfg['position_scale_m']**2+np.eye(n)*cfg['joint_weight']
        Q[n:,n:]=cfg['velocity_weight']*bank['J'].T@bank['J']/.02**2+np.eye(n)*cfg['joint_velocity_weight']
        if cfg.get('rope_velocity_weight',0.):Q[n:,n:]+=cfg['rope_velocity_weight']*bank['T'].T@bank['T']/.025**2
        R=np.eye(m.nu)*cfg['tension_weight'];P=solve_discrete_are(bank['A'],bank['B'],Q,R)
        self.L=np.linalg.solve(R+bank['B'].T@P@bank['B'],bank['B'].T@P@bank['A'])
        self.last=np.full(m.nu,.2);self.fallbacks=0;self.log=[]

    def command(self,t,qpos,qvel):
        # Deliberately accepts only clock, normal state and fixed target. The
        # task runner owns the disturbance; it never passes force/direction/onset.
        m=self.m;cfg=self.cfg;d=self.d;d.qpos[:]=qpos;d.qvel[:]=qvel;d.ctrl[:]=self.last
        d.xfrc_applied[:]=0;d.qfrc_applied[:]=0;mujoco.mj_forward(m,d)
        n=m.nv;s=float(np.clip(t/cfg['reach_end_s'],0,1));blend=s*s*s*(10+s*(-15+6*s));ds=30*s*s*(1-s)**2/cfg['reach_end_s'] if 0<s<1 else 0.
        if cfg.get('reference_profile')=='cosine_ramp_linear':
            endtime=cfg['reach_end_s'];r=cfg.get('reference_ramp_s',.04);vmax=1/(endtime-r)
            if t<r:
                blend=vmax*(t/2-r*np.sin(np.pi*t/r)/(2*np.pi));ds=vmax*(1-np.cos(np.pi*t/r))/2
            elif t<endtime-r:blend=vmax*(t-r/2);ds=vmax
            elif t<endtime:
                x=endtime-t;blend=1-vmax*(x/2-r*np.sin(np.pi*x/r)/(2*np.pi));ds=vmax*(1-np.cos(np.pi*x/r))/2
            else:blend=1.;ds=0.
        start,end=self.bank['equilibria'];qref=(1-blend)*start[:n]+blend*end[:n];vref=ds*(end[:n]-start[:n]);uff=(1-blend)*start[n:]+blend*end[n:]
        feedback_ramp=float(np.clip((t+.01)/cfg.get('startup_feedback_s',.01),0,1))
        desired=uff-feedback_ramp*self.L@np.r_[qpos-qref,qvel-vref]
        ramp=np.clip((t+.01)/cfg['startup_s'],0,1);desired=.2+ramp*(desired-.2)
        lower=np.maximum(0,self.last-cfg['tension_rate_n_s']*.01);upper=np.minimum(8,self.last+cfg['tension_rate_n_s']*.01)
        B=moments(m,d);M=np.empty((n,n));mujoco.mj_fullM(m,d,M);h=m.opt.timestep;D=np.diag(m.dof_damping);K=np.diag(m.jnt_stiffness)
        W=np.linalg.inv(M+h*D+h*h*K);F=M@d.qacc+D@d.qvel
        T=np.zeros((m.ntendon,n));mujoco.mju_sparse2dense(T,d.ten_J,m.ten_J_rownnz,m.ten_J_rowadr,m.ten_J_colind)
        V=np.zeros((n,m.nu+1));V[:,0]=qvel;X=np.zeros_like(V);ps=[];bs=[]
        steps=round(.01/h)
        for i in range(1,steps+1):
            alpha=i/steps;UU=np.c_[-alpha*self.last,alpha*np.eye(m.nu)]
            rhs=M@V+h*(B@UU-K@X);rhs[:,0]+=h*F;V=W@rhs;X+=h*V;z=T@V;bs.extend(z[:,0]);ps.extend(z[:,1:])
        C=np.array(ps);base=np.array(bs);limit=cfg['predicted_speed_limit_m_s'];penalty=cfg['input_change_weight']
        def fun(u):return .5*np.sum((u-desired)**2)+.5*penalty*np.sum((u-self.last)**2)
        def jac(u):return u-desired+penalty*(u-self.last)
        guard=cfg.get('speed_guard',True)
        sol=minimize(fun,np.clip(desired,lower,upper),jac=jac,bounds=Bounds(lower,upper),constraints=[LinearConstraint(C,-limit-base,limit-base)] if guard else [],method='SLSQP',options={'ftol':1e-9,'maxiter':60})
        fallback=not sol.success or (guard and max(abs(base+C@sol.x))>limit+1e-6)
        if fallback:
            self.fallbacks+=1
            def soft(u):
                r=np.maximum(abs(base+C@u)-limit,0);return fun(u)+5e7*(r@r)
            def sj(u):
                z=base+C@u;return jac(u)+1e8*C.T@(np.maximum(abs(z)-limit,0)*np.sign(z))
            sol=minimize(soft,np.clip(sol.x,lower,upper),jac=sj,bounds=Bounds(lower,upper),method='SLSQP',options={'ftol':1e-9,'maxiter':100})
        self.last=np.clip(sol.x,lower,upper)
        info={'time_s':float(t),'command_n':self.last.tolist(),'unbounded_feedback_command_n':desired.tolist(),'measured_qpos':qpos.tolist(),'measured_qvel':qvel.tolist(),'reference_qpos':qref.tolist(),'speed_prediction_max_m_s':float(max(abs(base+C@self.last))),'predicted_speed_slack_m_s':float(max(0,max(abs(base+C@self.last))-limit)),'fallback':bool(fallback),'allocation_success':bool(sol.success)}
        self.log.append(info);return self.last.copy()
