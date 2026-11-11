import numpy as np
from .base import SavedCase,folder,read,csv_array,vector
def load_case(root,task,case):
    b=folder(root,'P3_passive_progressive_stiffness','P3_progressive_stiffness');case=case or 'plus_x';summary=read(b/'summary.json')
    if (b/'cases').is_dir():
        p=b/'cases'/case;cols,a=csv_array(p/'trajectory.csv');xml=p/'model.xml';source=p/'trajectory.csv'
    else:
        p=b/case;source=p/'display_replay.npz';z=np.load(source);cols=z['columns'].tolist();a=z['rows'];xml=p/'robot.xml'
    ix={c:i for i,c in enumerate(cols)};q=a[:,[ix[f'qpos_{j}'] for j in range(6)]];v=a[:,[ix[f'qvel_{j}'] for j in range(6)]]
    return SavedCase(task,case,xml,a[:,ix['time_s']],q,v,force=np.column_stack([a[:,ix[f'force_{x}_n']] for x in 'xyz']),force_point=np.column_stack([a[:,ix[f'point_{x}_m']] for x in 'xyz']),tension=a[:,[ix[f'tension_n_{j}'] for j in range(2)]],stage=a[:,ix['stage']].astype(int),metadata={'stage_names':summary['stage_names'],'source_status':'Three coarse discrete cells; passive fixed payout; no continuous-body validation','state_format':'full physical CSV' if 'cases' in p.parts else 'original-state display samples'},sources=[xml,source,b/'summary.json'])
