import json
from pathlib import Path
from dataclasses import dataclass,field
import numpy as np
def read(p):return json.loads(Path(p).read_text(encoding='utf8'))
def folder(root,*choices):
    for name in choices:
        if (Path(root)/name).is_dir():return Path(root)/name
    raise FileNotFoundError(f'No dataset {choices} under {root}')
def csv_array(p):
    with Path(p).open(encoding='utf8') as f:cols=f.readline().strip().split(',')
    return cols,np.loadtxt(p,delimiter=',',skiprows=1,ndmin=2)
def vector(a,ix,prefix):
    keys=[prefix+'_'+v for v in 'xyz']
    return a[:,[ix[k] for k in keys]] if all(k in ix for k in keys) else None
def mapped_states(a,cols,mapping):
    ix={c:i for i,c in enumerate(cols)};n=mapping['nq'];q=np.empty((len(a),n));v=np.empty_like(q)
    for j in mapping['joints']:q[:,j['qpos_address']]=a[:,ix[j['qpos_column']]];v[:,j['qvel_address']]=a[:,ix[j['qvel_column']]]
    return q,v
@dataclass
class SavedCase:
    task:str
    case:str
    model_path:Path
    time:np.ndarray
    qpos:np.ndarray
    qvel:np.ndarray
    tip:np.ndarray|None=None
    reference:np.ndarray|None=None
    control:np.ndarray|None=None
    tension:np.ndarray|None=None
    force:np.ndarray|None=None
    force_point:np.ndarray|None=None
    reference_axis:np.ndarray|None=None
    actual_axis:np.ndarray|None=None
    stage:np.ndarray|None=None
    metadata:dict=field(default_factory=dict)
    sources:list=field(default_factory=list)
    def phase(self,i):
        t=float(self.time[i])
        if self.task in ['m7','spring_button']:
            windows=[(.3,'Approach'),(.5,'Pre-impact hold'),(.55,'Impact pulse'),(.9,'Frozen-input response' if 'no_feedback' in self.case else 'Active recovery'),(float('inf'),'Final hold evaluation')] if self.task=='m7' else [(.5,'Approach'),(.9,'Press'),(1.4,'Hold'),(1.8,'Release'),(float('inf'),'Settle')]
            return next(name for end,name in windows if t<end-1e-10)
        if self.task=='p3':return self.metadata['stage_names'][int(self.stage[i])]
        if self.task=='phs2d':return 'Unloaded baseline' if self.case=='equilibrium' else 'Load ramp / hold; no unloading record'
        window=self.metadata.get('hold_window')
        return 'Hold window' if window and window[0]-1e-10<=t<=window[1]+1e-10 else 'Saved task motion'
