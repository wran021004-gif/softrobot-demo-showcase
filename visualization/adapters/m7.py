import numpy as np
from pathlib import Path
from .base import SavedCase,read
def load_case(root,task,case):
    b=Path(root)/'M7_external_impact_active_recovery';case=case or 'positive_y';p=b/'cases'/case
    if not p.is_dir():p=b/'candidates/margin_feedback'/case
    a=np.load(p/'trajectory.npz');spec=read(b/'task.json')
    return SavedCase(task,case,b/'model.xml',a['time_s'],a['qpos'],a['qvel'],tip=a['tip_m'],reference=np.tile(spec['target_m'],(len(a['time_s']),1)),control=a['control_n'],tension=a['tension_n'],force=a['force_n'],force_point=a['force_point_m'],metadata={'source_status':read(p/'result.json')['status']+' / '+('frozen input after 0.5 s' if 'no_feedback' in case else 'state feedback, same controller in both directions'),'state_format':'complete 0.5 ms timestep NPZ/CSV'},sources=[b/'model.xml',p/'trajectory.npz',p/'result.json'])
