"""Event contact management and bounded tension allocation, 10 ms feedback."""
import copy
import numpy as np
import mujoco
from scipy.optimize import least_squares,minimize,LinearConstraint,Bounds
from mechanics import HERE,BASE,read,write
from solver import dense_moments,smooth
from evaluator import contact_state

def quintic(t,start,end):
    s=np.clip((t-start)/(end-start),0,1)
    return float(smooth(s)),float(30*s*s*(1-s)**2/(end-start)) if 0<s<1 else 0.

def reference(t,origin,cfg):
    contact=np.array([.2955,.026,.15]);hold=contact+[0,.002,0]
    if t<.5:
        s,ds=quintic(t,0,cfg.get('approach_end_s',.5));p=origin+s*(contact-origin);v=ds*(contact-origin);q=qd=0.
    elif t<.9:
        s,ds=quintic(t,.5,.9);p=contact+s*(hold-contact);v=ds*(hold-contact);q=.002*s;qd=.002*ds
    elif t<1.4:p=hold.copy();v=np.zeros(3);q=.002;qd=0.
    elif t<cfg['release_contact_end_s']:
        s,ds=quintic(t,1.4,cfg['release_contact_end_s']);q=.002*(1-s);qd=-.002*ds;p=contact+[0,q,0];v=np.array([0,qd,0])
    else:
        s,ds=quintic(t,cfg['release_contact_end_s'],cfg['withdraw_end_s']);p=contact+s*(origin-contact);v=ds*(origin-contact);q=qd=0.
    return p,v,q,qd

def inverse_bank(m,origin):
    """Algebraic load-balanced states. No applied force or physics integration.
    The exact same native-contact model is used for the subsequent acceptance.
    """
    d=mujoco.MjData(m);n=m.nq-1;sid=m.site('probe_center').id;J=np.zeros((3,m.nv))
    z=np.r_[np.zeros(n),[2.8,.2,.5,.3]];lo=np.r_[np.full(n,-.35),np.full(4,.2)];hi=np.r_[np.full(n,.35),np.full(4,8.)]
    states=[];records=[];calls=0
    for desired,force,q in [(origin,0.,0.),(np.array([.2955,.026,.15]),0.,0.),(np.array([.2955,.028,.15]),0.,.002),(np.array([.2955,.028,.15]),.1,.002)]:
        def residual(x):
            nonlocal calls
            calls+=1;d.qpos[:-1]=x[:n];d.qpos[-1]=q;d.qvel[:]=0;d.ctrl[:]=x[n:];d.qacc_warmstart[:]=0;mujoco.mj_forward(m,d)
            mujoco.mj_jacSite(m,d,J,None,sid)
            balance=d.qfrc_passive+d.qfrc_actuator-d.qfrc_bias-J.T@np.array([0,force,0])
            return np.r_[balance[:-1]*1000,(d.site_xpos[sid]-desired)*np.array([600,2000,2000]),.0005*x[n:]]
        sol=least_squares(residual,np.clip(z,lo+1e-9,hi-1e-9),bounds=(lo,hi),max_nfev=90,
            ftol=1e-10,xtol=1e-10,gtol=1e-9)
        z=sol.x.copy();residual(z);states.append(np.r_[z[:n],q,z[n:]])
        records.append({'desired_probe_m':desired.tolist(),'reaction_balance_n':force,'cost':float(sol.cost),
            'nfev':sol.nfev,'probe_m':d.site_xpos[sid].tolist(),'tensions_n':z[n:].tolist(),
            'max_robot_equilibrium_residual_nm':float(abs(sol.fun[:n]).max()/1000)})
    return np.array(states),{'algebraic_forward_evaluations':calls,'nodes':records,
        'scope':'analytical contact-reaction balance initializer only; not an accepted contact trajectory'}

class Controller:
    def __init__(self,m,origin,bank,config):
        required=['xyz_feedback','button_state_feedback','reaction_feedforward','bounded_allocation',
            'position_gain','velocity_time_s','startup_s','tension_rate_limit_n_s','delta_tension_penalty',
            'predicted_rope_speed_limit_m_s','release_contact_end_s','withdraw_end_s','force_gain','button_depth_gain','button_velocity_time_s']
        if any(k not in config for k in required):raise ValueError('Incomplete effective control configuration')
        if type(config['reaction_feedforward']) is not bool:raise ValueError('reaction_feedforward must be a JSON boolean')
        if not all(config[k] is True for k in ['xyz_feedback','button_state_feedback','bounded_allocation']):raise ValueError('Task requires XYZ/state feedback and bounded allocation')
        self.m=m;self.origin=origin;self.bank=bank;self.cfg=copy.deepcopy(config);self.sid=m.site('probe_center').id
        self.last=np.full(4,.2);self.maps=[];self.compliance=[];self.guard_fallbacks=0;self.max_config_error=0
        self.weights=np.asarray(config['position_weights'],dtype=float)
        if self.weights.shape!=(3,) or np.any(self.weights<=0):raise ValueError('All XYZ feedback weights must be positive')
        self.scratch=mujoco.MjData(m)
        self.contact_latched=False;self.contact_time=None;self.withdraw_time=None;self.state='FREE_APPROACH'
        n=m.nv-1
        for row in bank:
            d=mujoco.MjData(m);d.qpos[:]=row[:m.nq];d.ctrl[:]=row[m.nq:];mujoco.mj_forward(m,d)
            J=np.zeros((3,m.nv));mujoco.mj_jacSite(m,d,J,None,self.sid);B=dense_moments(m,d)
            # Retain the original passive stiffness approximation. The first
            # candidates showed that the poorly conditioned tangent inverse
            # over-corrected axial shortening and disrupted gravity support.
            K=np.diag(m.jnt_stiffness[:n])+np.eye(n)*1e-9
            self.maps.append(J[:,:n]@np.linalg.solve(K,B[:n]));self.compliance.append(J[:,:n]@np.linalg.solve(K,J[:,:n].T))
        self.summary={'xyz_feedback':True,'position_weights':self.weights.tolist(),'button_state_feedback':True,
            'button_sensors':['button_slide qpos','button_slide qvel','mj_contactForce(probe,button_cap)'],
            'reaction_feedforward':config['reaction_feedforward'],'reaction_semantics':'true selects load-balanced inverse node; false selects free inverse node; no inverted contact_ff switch',
            'bounded_allocation':True,'configuration_source':'per-candidate effective_control.json, required argument; no silent rollout defaults',
            'all_config_fields_consumed':required,'control_dt_s':.01,'physics_dt_s':.001,
            'rope_prediction_horizon_s':.001,
            'stiffness_map':'original passive hinge stiffness; avoids ill-conditioned full tangent inverse',
            'startup_rope_guard_bypass_until_s':config.get('guard_start_s',.15),
            'rope_guard':'one-physics-step acceleration prediction from actual ten_J/ten_velocity, with explicit infeasibility fallback; acceptance uses actual 1 ms speed'}

    def command(self,d):
        m=self.m;cfg=self.cfg;t=float(d.time);p,vref,qref,qdref=reference(t,self.origin,cfg)
        # Smooth blend of inverse nodes, selected by explicit configuration.
        if t<.5 or t>=cfg['release_contact_end_s']:
            s=np.clip((p[1]-self.origin[1])/.005,0,1);k0,k1=0,1
        else:s=np.clip(qref/.002,0,1);k0,k1=1,(3 if cfg['reaction_feedforward'] else 2)
        uff=(1-s)*self.bank[k0,m.nq:]+s*self.bank[k1,m.nq:]
        G=(1-s)*self.maps[k0]+s*self.maps[k1];C=(1-s)*self.compliance[k0]+s*self.compliance[k1]
        J=np.zeros((3,m.nv));mujoco.mj_jacSite(m,d,J,None,self.sid);v=J@d.qvel
        force,pairs,_,_=contact_state(m,d);q=float(d.qpos[m.jnt_qposadr[m.joint('button_slide').id]]);qd=float(d.qvel[m.jnt_dofadr[m.joint('button_slide').id]])
        if pairs>0 and (force>cfg.get('contact_on_force_n',.0001) or q>2e-6):
            if not self.contact_latched:self.contact_time=t
            self.contact_latched=True
        if cfg.get('event_contact',True):
            if t<.5 and self.contact_latched:
                self.state='CONTACT_ACQUIRE' if t-self.contact_time<.03 else 'CONTACT_FOLLOW'
                qref=cfg.get('acquire_depth_m',.00012);qdref=0.
                p=np.array([.2955,.026+qref,.15]);vref=np.zeros(3)
            elif t<.5:self.state='FREE_APPROACH'
            elif t<.9:self.state='PRESS'
            elif t<1.4:self.state='HOLD'
            else:
                if self.withdraw_time is None and t>=cfg['release_contact_end_s'] and abs(q)<cfg.get('release_gate_q_m',1e-6) and abs(qd)<cfg.get('release_gate_v_m_s',.0001) and force<cfg.get('release_gate_force_n',.001):self.withdraw_time=t
                if self.withdraw_time is None:
                    self.state='RETURN_FOLLOW'
                    if t>=cfg['release_contact_end_s']:qref=0.;qdref=0.;p=np.array([.2955,.026,.15]);vref=np.zeros(3)
                    p[1]+=cfg.get('return_position_bias_m',0.)
                else:
                    self.state='WITHDRAW';s,ds=quintic(t,self.withdraw_time,cfg['withdraw_end_s'] if cfg['withdraw_end_s']>self.withdraw_time+.02 else self.withdraw_time+.12)
                    contact=np.array([.2955,.026,.15]);target=contact+np.array([0,-cfg.get('withdraw_distance_m',.005),0])
                    p=contact+s*(target-contact);vref=ds*(target-contact);qref=qdref=0.
            # Recompute inverse interpolation after event reference changes.
            if self.state in ['CONTACT_ACQUIRE','CONTACT_FOLLOW','PRESS','HOLD','RETURN_FOLLOW']:
                s=np.clip(qref/.002,0,1);k0,k1=1,(3 if cfg['reaction_feedforward'] else 2)
            else:s=np.clip((p[1]-self.origin[1])/.005,0,1);k0,k1=0,1
            uff=(1-s)*self.bank[k0,m.nq:]+s*self.bank[k1,m.nq:]
            G=(1-s)*self.maps[k0]+s*self.maps[k1];C=(1-s)*self.compliance[k0]+s*self.compliance[k1]
        correction=cfg['position_gain']*(p-d.site_xpos[self.sid]+cfg['velocity_time_s']*(vref-v))
        active=(self.contact_latched and self.withdraw_time is None) if cfg.get('event_contact',True) else (.5-1e-9<=t<cfg['release_contact_end_s'])
        if active:
            desired_force=max(0,50*qref+.2*qdref)
            correction+=C@np.array([0,cfg['force_gain']*(desired_force-force),0])
            correction[1]+=cfg['button_depth_gain']*(qref-q+cfg['button_velocity_time_s']*(qdref-qd))
        if cfg.get('startup_shape')=='linear_lookahead':blend=float(np.clip((t+.01)/cfg['startup_s'],0,1))
        else:blend=float(smooth(t/cfg['startup_s']))
        uff=.2+blend*(uff-.2);correction*=blend
        A=G*self.weights[:,None]*1000;b=correction*self.weights*1000
        lower=np.maximum(.2,self.last-cfg['tension_rate_limit_n_s']*.01);upper=np.minimum(8.,self.last+cfg['tension_rate_limit_n_s']*.01)
        lam=cfg['delta_tension_penalty']
        tracking=cfg['feedforward_tracking_penalty']
        H=A.T@A+np.eye(4)*(lam+tracking);g=-(A.T@(A@uff+b)+lam*self.last+tracking*uff)
        def fun(u):return .5*u@H@u+g@u
        def jac(u):return H@u+g
        # Actual tendon state and native mass-response prediction. No rollout.
        B=dense_moments(m,d);M=np.empty((m.nv,m.nv));mujoco.mj_fullM(m,d,M);R=np.linalg.solve(M,B)
        T=np.zeros((m.ntendon,m.nv));mujoco.mju_sparse2dense(T,d.ten_J,m.ten_J_rownnz,m.ten_J_rowadr,m.ten_J_colind)
        scratch=self.scratch;mujoco.mj_copyData(scratch,m,d);h=1e-5;scratch.qpos[:]+=h*d.qvel;mujoco.mj_forward(m,scratch)
        jdotv=(scratch.ten_velocity-d.ten_velocity)/h
        # A 10 ms acceleration extrapolation spans fast flexible modes and can
        # spuriously block gravity support. Predict one native 1 ms step only.
        P=.001*T@R;base=d.ten_velocity+.001*(T@d.qacc+jdotv)-P@d.ctrl
        vmax=cfg['predicted_rope_speed_limit_m_s'];constraint=LinearConstraint(P,-vmax-base,vmax-base)
        enabled=t>=cfg.get('guard_start_s',.15)
        sol=minimize(fun,np.clip(self.last,lower,upper),jac=jac,bounds=Bounds(lower,upper),constraints=[constraint] if enabled else [],method='SLSQP',options={'maxiter':35,'ftol':1e-9})
        fallback=False
        if not sol.success or np.max(abs(base+P@sol.x))>vmax+1e-6:
            self.guard_fallbacks+=1
            fallback=True
            sol=minimize(fun,np.clip(self.last,lower,upper),jac=jac,bounds=Bounds(lower,upper),method='SLSQP',options={'maxiter':30,'ftol':1e-9})
        self.last=np.clip(sol.x,lower,upper)
        return self.last.copy(),{'reference_probe_m':p,'qref_m':qref,'q_measured_m':q,'qdot_measured_m_s':qd,'contact_measured_n':force,
            'position_weights':self.weights.copy(),'predicted_rope_speed_m_s':base+P@self.last,'state':self.state,'actual_contact_pairs':pairs,'speed_guard_enabled':enabled,'speed_guard_fallback':fallback,'predicted_speed_violation_m_s':float(max(0,np.max(abs(base+P@self.last))-vmax))}
