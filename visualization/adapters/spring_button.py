import numpy as np
from pathlib import Path
from .base import SavedCase,read
def load_case(root,task,case):
    b=Path(root)/'spring_button_press/v1_2/candidates';case=case or 'FINAL_C009';p=b/case;a=np.load(p/'trajectory.npz');trace=read(p/'controller_trace.json');nodes=np.array([x['time_s'] for x in trace]);ids=np.maximum(0,np.searchsorted(nodes,a['time_s'],side='right')-1);reference=np.array([x['reference_probe_m'] for x in trace])[ids]
    return SavedCase(task,case,p/'environment.xml',a['time_s'],a['qpos'],a['qvel'],tip=a['probe_m'],reference=reference,control=a['control_n'],tension=a['tension_n'],metadata={'source_status':'C009 original evaluator PASS; independent closed-loop exact match','state_format':'complete 2001-state NPZ','reference_kind':'actual archived 10 ms feedback-node reference, held between updates; original physical states unmodified','probe_button_force_n':a['contact_force_n'],'button_q_m':a['button_q_m']},sources=[p/'environment.xml',p/'trajectory.npz',p/'result.json',p/'controller_trace.json'])
