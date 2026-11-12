import numpy as np
from pathlib import Path
from .base import SavedCase,read
def load_case(root,task,case):
    h=Path(root)/'M8_narrow_passage_traversal';name=case or 'final_candidate';p=h/'cases'/name;a=np.load(p/'trajectory.npz');spec=read(h/'task.json');evaluation=read(p/'evaluation.json');mapping=read(p/'geometry_map.json')
    return SavedCase(task,name,p/'model.xml',a['time_s'],a['qpos'],a['qvel'],tip=a['tip_m'],reference=np.tile(spec['target_m'],(len(a['time_s']),1)),control=a['control_n'],tension=a['tension_n'],stage=a['phase'],metadata={'source_status':evaluation['status']+' / bounded failed traversal; not a success animation','tunnel':spec['tunnel'],'clearance_m':a['clearance_m'],'nearest_fromto_m':a['nearest_fromto_m'],'nearest_robot_geom':a['nearest_robot_geom'],'nearest_wall_geom':a['nearest_wall_geom'],'cell_centers_m':a['cell_centers_m'],'geometry_map':mapping,'state_format':'complete 0.5 ms physical states'},sources=[p/'model.xml',p/'trajectory.npz',p/'evaluation.json'])
