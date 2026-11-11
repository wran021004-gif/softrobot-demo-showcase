import numpy as np
from .base import SavedCase,folder,read,csv_array,mapped_states,vector
FOLDERS={'m1':'M1_reach_hold','m2':'M2_loaded_hold','m3':'M3_fast_reach','m4':'M4_legacy_tracking','m5':'M5_tool_axis','m6':'M6_loaded_scan'}
DEFAULTS={'m1':'central_high','m2':'positive','m3':'default','m4':'default','m5':'default','m6':'default'}
def load_case(root,task,case):
    base=folder(root,task.upper(),FOLDERS[task]);case=case or DEFAULTS[task];p=base/case;mapping=read(base/'joint_map.json');cols,a=csv_array(p/'trajectory_timestep.csv');ix={v:k for k,v in enumerate(cols)};q,v=mapped_states(a,cols,mapping);t=a[:,ix['time_s']]
    rc,ref=csv_array(p/'reference.csv');ri={v:k for k,v in enumerate(rc)};reference=np.column_stack([np.interp(t,ref[:,ri['time_s']],ref[:,ri['reference_position_m_'+x]]) for x in 'xyz'])
    axes=np.column_stack([np.interp(t,ref[:,ri['time_s']],ref[:,ri['reference_tool_axis_world_'+x]]) for x in 'xyz']) if task in ['m5','m6'] and 'reference_tool_axis_world_x' in ri else None
    xml=base/'robot.xml';tj=read(p/'task.json');r=SavedCase(task,case,xml,t,q,v,tip=vector(a,ix,'tip_position_m'),reference=reference,control=a[:,[ix[c] for c in mapping['requested_tension_columns']]],tension=a[:,[ix[c] for c in mapping['actual_tension_columns']]],force=vector(a,ix,'external_force_n'),reference_axis=axes,actual_axis=vector(a,ix,'tool_axis_world'),metadata={'hold_window':tj.get('holding_window_s'),'source_status':'Frozen historical result; original dense limitations retained','state_format':'named-joint trajectory_timestep.csv'},sources=[xml,p/'trajectory_timestep.csv',p/'reference.csv',p/'task.json'])
    return r
