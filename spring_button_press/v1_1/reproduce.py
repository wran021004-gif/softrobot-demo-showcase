"""Optional user-invoked reproduction after the bounded run; not auto executed."""
import argparse,importlib.util,sys
from pathlib import Path
import numpy as np
import mujoco
from mechanics import HERE,BASE,read,write,load_model
from evaluator import evaluate

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('candidate',choices=['C1_A','C1_B','S1','S2','S3','S4']);ap.add_argument('--output',type=Path,required=True);ap.add_argument('--closed-loop',action='store_true');args=ap.parse_args()
    if args.output.resolve().is_relative_to(BASE.resolve()):raise SystemExit('Output must be outside all frozen task directories')
    folder=HERE/'candidates'/args.candidate
    def module(name,path):
        s=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(s);sys.modules[name]=m;s.loader.exec_module(m);return m
    c=module('controller',folder/'controller_source.py');runner=module('recorded_runner',folder/'runner_source.py')
    m=load_model(path=folder/'environment.xml');d=mujoco.MjData(m);mujoco.mj_forward(m,d);origin=d.site_xpos[m.site('probe_center').id].copy()
    bank=np.load(folder/'inverse_bank.npz')['states'];cfg=read(folder/'effective_control.json');old=np.load(folder/'trajectory.npz')
    a,bad,summary,log=runner.rollout(m,origin,bank,cfg,controls=None if args.closed_loop else old['control_n'])
    result=evaluate(a,m,read(folder/'mechanics.json')['total_structure_mass_kg']);args.output.mkdir(parents=True,exist_ok=True)
    np.savez_compressed(args.output/'trajectory.npz',**a);write(args.output/'result.json',result);write(args.output/'reproduction_check.json',{'source':args.candidate,'closed_loop':args.closed_loop,'max_qpos_difference':float(abs(a['qpos']-old['qpos']).max()),'runtime':summary})
    print(result['status'])
