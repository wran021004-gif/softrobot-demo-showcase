"""Bounded algebraic reference design and 10 ms native-state tension feedback.
Reference interpolation is a control command, never exported as physical state.
"""
import numpy as np
import mujoco
from scipy.optimize import least_squares,lsq_linear
from scipy.interpolate import CubicSpline
from environment import H,read,write
def moments(m,d):
    a=np.zeros((m.nu,m.nv));mujoco.mju_sparse2dense(a,d.actuator_moment,d.moment_rownnz,d.moment_rowadr,d.moment_colind);return -a.T
def geometry_ids(m):
    walls=[i for i in range(m.ngeom) if m.geom(i).name.startswith('tunnel_')]
    robot=[i for i in range(m.ngeom) if m.geom_bodyid[i]>0 and m.body(m.geom_bodyid[i]).name!='fixed_base']
    return robot,walls
def distances(m,d,robot,walls):return np.array([mujoco.mj_geomDistance(m,d,i,j,1.,None) for i in robot for j in walls])
def prepare(m,folder,initial=None):
    task=read(H/'task.json');n=m.nq;d=mujoco.MjData(m);robot,walls=geometry_ids(m);sid=m.site('tip_site').id;calls=0
    # Begin with a relaxed posture on the same continuation branch, then retract.
    z=np.r_[np.zeros(n),np.ones(m.nu)]
    lo=np.r_[np.full(n,-.6),np.zeros(m.nu)];hi=np.r_[np.full(n,.6),np.full(m.nu,7.99)]
    records=[]
    def solve(point,z,obstacle=True,maxeval=100):
        nonlocal calls
        def f(x):
            nonlocal calls
            calls+=1;d.qpos[:]=x[:n];d.qvel[:]=0;d.ctrl[:]=x[n:];mujoco.mj_forward(m,d)
            clearance=np.minimum(distances(m,d,robot,walls)-.002,0) if obstacle else np.zeros(len(robot)*len(walls))
            return np.r_[(d.qfrc_passive+d.qfrc_actuator-d.qfrc_bias)*3000,(d.site_xpos[sid]-point)*4000,clearance*8000,.002*x[n:]]
        sol=least_squares(f,z,bounds=(lo,hi),max_nfev=maxeval,ftol=1e-9,xtol=1e-9,gtol=1e-7);f(sol.x)
        record={'requested_tip_m':list(point),'tip_m':d.site_xpos[sid].tolist(),'cost':float(sol.cost),'nfev':sol.nfev,'balance_max_nm':float(abs(sol.fun[:n]).max()/3000),'min_clearance_m':float(distances(m,d,robot,walls).min()),'tension_n':sol.x[n:].tolist()}
        records.append(record);print(record,flush=True);return sol.x
    if initial is None:
        z=solve([.282,0,task['target_m'][2]],z,False,80)
        z=solve(task['initial_tip_requested_m'],z,False,150)
        if distances(m,d,robot,walls).min()<.001:raise RuntimeError('Initial preparation is not collision-free')
        initial={'qpos':z[:n].copy(),'qvel':np.zeros(n),'control_n':z[n:].copy()}
        np.savez_compressed(H/'initial_state.npz',**initial)
    else:
        d.qpos[:]=initial['qpos'];d.qvel[:]=0;mujoco.mj_forward(m,d);B=moments(m,d)
        u=lsq_linear(B,d.qfrc_bias-d.qfrc_passive,bounds=(0,8),tol=1e-12).x;z=np.r_[initial['qpos'],u]
        initial={**initial,'control_n':u};np.savez_compressed(folder/'initial_state.npz',**initial)
    points=[[.195,y,.15] for y in np.linspace(-.1,0,6)]+[[x,0,.15] for x in np.linspace(.195,.285,13)[1:]]
    rows=[z.copy()]
    for point in points[1:]:z=solve(point,z);rows.append(z.copy())
    bank={'equilibria':np.array(rows),'progress':np.linspace(0,1,len(rows)),**initial}
    np.savez_compressed(folder/'controller_bank.npz',**bank);write(folder/'optimization.json',{'mj_forward_evaluations':calls,'physics_steps':0,'nodes':records,'purpose':'algebraic references only; success decided by integrated trajectory'})
    return bank
class Controller:
    def __init__(self,m,bank):
        self.m=m;self.d=mujoco.MjData(m);self.bank=bank;self.task=read(H/'task.json');self.ref=CubicSpline(bank['progress'],bank['equilibria'],axis=0);self.updates=[]
    def command(self,t,q,v):
        m=self.m;d=self.d;n=m.nq;d.qpos[:]=q;d.qvel[:]=v;mujoco.mj_forward(m,d)
        s=np.clip(t/self.task['reach_end_s'],0,1);b=s**3*(10-15*s+6*s*s);db=30*s*s*(1-s)**2/self.task['reach_end_s'] if 0<s<1 else 0.
        ref=self.ref(b);qr=ref[:n];vr=self.ref(b,1)[:n]*db;uff=ref[n:]
        M=np.empty((n,n));mujoco.mj_fullM(m,d,M);B=moments(m,d);acc=900*(qr-q)+60*(vr-v)
        force=d.qfrc_bias-d.qfrc_passive+M@acc
        weights=1/np.maximum(m.jnt_stiffness,.1)
        A=np.vstack([B*weights[:,None],np.eye(m.nu)*.0002]);rhs=np.r_[force*weights,uff*.0002]
        u=lsq_linear(A,rhs,bounds=(0,8),tol=1e-10,max_iter=60).x
        self.updates.append([t,*u]);return u
