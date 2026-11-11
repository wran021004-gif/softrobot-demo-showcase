"""Saved-data audit only: no physical integration."""
import json
import numpy as np
from mechanics import HERE,BASE,read,write
def main():
    src=BASE/'v1_1/candidates/S4';a=np.load(src/'trajectory.npz');t=a['time_s'];v=a['tendon_speed_m_s'];q=a['button_q_m'];logs=read(src/'controller_trace.json')
    records=[]
    for i in range(4):
        k=np.argmax(abs(v[t<.15,i]));records.append({'tendon':f'T{i}','time_s':float(t[k]),'signed_speed_mm_s':float(v[k,i]*1000)})
    predicted=[]
    for row in logs[:15]:
        k=int(round(row['time_s']*1000));predicted.append({'time_s':row['time_s'],'prediction_mm_s':(np.array(row['predicted_rope_speed_m_s'])*1000).tolist(),'actual_next_ms_mm_s':(v[k+1]*1000).tolist(),'actual_next_knot_mm_s':(v[k+10]*1000).tolist()})
    events=[]
    pairs=a['contact_pairs']>0
    for k in np.flatnonzero(pairs[1:]!=pairs[:-1])+1:events.append({'time_s':float(t[k]),'contact':bool(pairs[k]),'q_um':float(q[k]*1e6),'qdot_mm_s':float(a['button_qdot_m_s'][k]*1000),'force_n':float(a['contact_force_n'][k])})
    u=a['control_n'];result={'source':'unchanged v1.1 S4 saved trajectory/source/audit','physical_steps':0,'startup_tendon_peaks':records,'startup_predictions':predicted,'contact_transitions':events,'equilibrium_tension_n':read(src/'optimization.json')['nodes'][0]['tensions_n'],'initial_tension_n':u[0].tolist(),'max_instantaneous_tension_rate_n_s':float(abs(np.diff(u,axis=0)/.001).max()),'quintic_max_slope_multiplier':1.875,'analysis':[
      'The original guard is omitted before 0.15 s, including the T3 peak at 0.060 s. Later infeasibility reruns an unconstrained speed allocation; this is logged but cannot enforce physical speed.',
      'Initial qpos/qvel are zero and tensions 0.20 N, far from the 4.22 N T0 algebraic gravity equilibrium. The physical initial condition is retained. Startup must establish support before sag/recovery excites flexible modes.',
      'The one-step predictor uses the final target tension immediately whereas the runner interpolates it over 10 ms. It linearizes current acceleration/contact and cannot predict the whole knot interval or nonlinear flexible response.',
      'Quintic node interpolation peaks at 1.875 times average knot slope; saved 1 ms commands reach 146.048 N/s despite the 80 N/s knot limit.',
      'Native contact force is measured during Approach but old button feedback is gated until 0.5 s. Loss of contact leaves an underdamped 5 g, 50 N/m, 0.2 Ns/m slider to recoil below zero.',
      'Return loses native contact before zero; endpoint settling alone does not satisfy the global lower travel bound. Event acquisition and a near-zero low-speed release gate are required.'
    ]}
    write(HERE/'results/s4_failure_audit.json',result)
    print(json.dumps(result,indent=2))
if __name__=='__main__':main()
