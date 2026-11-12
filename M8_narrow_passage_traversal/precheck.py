"""Read-only-source algebraic preflight. No mj_step and no success claim."""
import os
os.environ.setdefault('OPENBLAS_NUM_THREADS','1')
from pathlib import Path
import json,sys
import numpy as np
import mujoco
from scipy.optimize import least_squares
H=Path(__file__).resolve().parent
def dump(p,x):p.write_text(json.dumps(x,indent=2)+'\n',encoding='utf8')
def main():
    records=[]
    for folder in ['C1_four_tendons','M2_loaded_hold','figure_eight_tracking']:
        m=mujoco.MjModel.from_xml_path(str(H.parent/folder/'robot.xml'));d=mujoco.MjData(m);mujoco.mj_forward(m,d)
        geoms=[{'name':m.geom(i).name,'type':int(m.geom_type[i]),'size_m':m.geom_size[i].tolist(),'position_m':d.geom_xpos[i].tolist(),'contype':int(m.geom_contype[i]),'conaffinity':int(m.geom_conaffinity[i])} for i in range(m.ngeom)]
        records.append({'source':folder,'nu':m.nu,'dt_s':m.opt.timestep,'zero_tip_m':d.site('tip_site').xpos.tolist(),'geoms':geoms})
    dump(H/'precheck_geometry.json',{'mujoco':mujoco.__version__,'distance_api':mujoco.mj_geomDistance.__doc__,'sources':records})
    m=mujoco.MjModel.from_xml_path(str(H.parent/'C1_four_tendons/robot.xml'));d=mujoco.MjData(m);n=m.nq;sid=m.site('tip_site').id
    z=np.r_[np.zeros(n),np.ones(m.nu)]
    out=[]
    for point in [[.285,0,.15],[.205,0,.15],[.24,0,.15]]:
        def f(x):
            d.qpos[:]=x[:n];d.qvel[:]=0;d.ctrl[:]=x[n:];mujoco.mj_forward(m,d)
            return np.r_[(d.qfrc_passive+d.qfrc_actuator-d.qfrc_bias)*1000,(d.site_xpos[sid]-point)*1000,.001*x[n:]]
        sol=least_squares(f,z,bounds=(np.r_[np.full(n,-.5),np.full(m.nu,.001)],np.r_[np.full(n,.5),np.full(m.nu,7.99)]),max_nfev=130,ftol=1e-10,xtol=1e-10,gtol=1e-8)
        z=sol.x;f(z);r={'point_m':point,'tip_m':d.site_xpos[sid].tolist(),'u_n':z[n:].tolist(),'balance_max_nm':float(abs(sol.fun[:n]).max()/1000),'cost':sol.cost,'nfev':sol.nfev,'guide_m':d.body('guide_1').xpos.tolist()};out.append(r);print(r,flush=True)
    dump(H/'precheck_equilibria_axial.json',out)
if __name__=='__main__':main()
