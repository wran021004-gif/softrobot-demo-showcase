"""Read the existing four-tendon, two-cycle result without simulation."""
import numpy as np
from .base import SavedCase,folder,read
def load_case(root,task,case):
    p=folder(root,'figure_eight_tracking');a=np.load(p/'trajectory_full.npz')
    return SavedCase(task,'saved_two_cycles',p/'robot.xml',a['time_s'],a['qpos'],a['qvel'],tip=a['tip_m'],reference=a['reference_m'],control=a['tension_n'],tension=a['tension_n'],metadata={'source_status':read(p/'results.json')['status']+' / original two-cycle result','state_format':'saved full timestep NPZ; original four tendons'},sources=[p/'robot.xml',p/'trajectory_full.npz',p/'results.json'])
