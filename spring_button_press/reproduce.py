"""Optional user-invoked independent native-contact replay of recorded controls.
Not automatically run: the bounded study is complete. Output outside originals.
"""
import argparse
from pathlib import Path
import numpy as np
from environment import HERE,read,assemble
from solver import rollout,save_run

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('candidate',choices=['A1','A2','A3','B1','B2','B3','B4','B5','B6']);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    folder=HERE/('baseline' if args.candidate.startswith('A') else 'candidates')/args.candidate
    if args.output.resolve().is_relative_to(HERE.resolve()):raise SystemExit('Choose an output directory outside the frozen task directory.')
    candidate=None if args.candidate.startswith('A') else read(folder/'design_request.json')
    m,_,_=assemble(candidate);origin=np.array(read(HERE/'environment.json')['reference_probe_center_m'])
    controls=np.load(folder/'recorded_controls.npz')['control_n'];a,bad=rollout(m,origin,None,controls=controls)
    r=save_run(args.output,m,a,bad,{'source':args.candidate,'method':'native-contact replay of recorded controls'})
    print(r['status'],r['checks'])
