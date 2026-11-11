import numpy as np
from .base import SavedCase,folder,read
def load_case(root,task,case):
    b=folder(root,'PHS2D_passive_head_stabilization','PHS2D_head_stabilization');case=case or 's050_positive';p=b/case
    if (p/'states_timestep.npz').exists():
        source=p/'states_timestep.npz';z=np.load(source);a=z['trajectory'];ix={c:i for i,c in enumerate(z['columns'].tolist())};q=a[:,[ix[f'qpos_{j}'] for j in range(144)]];v=a[:,[ix[f'qvel_{j}'] for j in range(144)]];t=a[:,ix['time_s']];tip=z['tip_position_m'];fp=z['force_point_m'];f=np.column_stack([a[:,ix['force_n']],np.zeros((len(a),2))]);u=a[:,[ix[f'tension_{j}'] for j in range(4)]]
    else:
        source=p/'display_replay.npz';z=np.load(source);q=z['qpos'];v=z['qvel'];t=z['time_s'];tip=z['tip_m'];fp=z['force_point_m'];f=z['force_n'];u=z['tension_n']
    return SavedCase(task,case,b/'robot.xml',t,q,v,tip=tip,tension=u,force=f,force_point=fp,metadata={'source_status':'development_pass; not registered SELECT; nine segments; horizontal stabilization only','state_format':'full original NPZ' if 'trajectory' in z.files else 'original-state display samples'},sources=[b/'robot.xml',source])
