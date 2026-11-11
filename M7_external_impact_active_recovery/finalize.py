"""Finalize frozen M7 records and contrasts, with zero new physics steps."""
import hashlib,json,shutil
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent
def read(p):return json.loads(Path(p).read_text(encoding='utf8'))
def write(p,x):Path(p).write_text(json.dumps(x,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf8')
def main():
    src=HERE/'candidates/margin_feedback';cases=read(src/'evaluation.json');task=read(HERE/'task.json');audit={};arrays={}
    for name,result in cases.items():
        target=HERE/'cases'/name
        if not target.exists():shutil.copytree(src/name,target)
        arrays[name]=np.load(src/name/'trajectory.npz')
        a=arrays[name];t=a['time_s'];force=a['force_n'][:,1];integral=float(np.trapezoid(force,t));analytic=(0 if name=='no_impulse' else (1 if name.startswith('positive') else -1)*2*.04*.05/np.pi)
        assert len(t)==2001 and np.allclose(np.diff(t),.0005,atol=1e-14,rtol=0)
        assert np.max(abs(a['force_n'][t>=.55-1e-12]))==0
        assert abs(integral-a['impulse_ns'][-1,1])<1e-15
        audit[name]={'numerical_impulse_ns':integral,'analytic_half_sine_impulse_ns':analytic,'relative_quadrature_error':0 if analytic==0 else abs(integral-analytic)/abs(analytic),'force_off_after_0p55s':True,'full_state_rows':2001,'control_update_rows':100,'physical_dt_s':.0005,'control_dt_s':.01,'native_contact_steps':int(np.count_nonzero(a['contacts'])),'warning_count':int(a['warnings'].max())}
    no=arrays['no_impulse'];contrast={}
    for sign in ['positive_y','negative_y']:
        a=arrays[sign];b=arrays[sign+'_no_feedback'];post=a['time_s']>=.5-1e-12;delta=np.linalg.norm(a['tip_m']-no['tip_m'],axis=1);k=int(np.argmax(delta))
        assert np.array_equal(a['qpos'][:1001],no['qpos'][:1001]) and np.array_equal(a['control_n'][:1001],no['control_n'][:1001])
        assert np.array_equal(a['qpos'][:1001],b['qpos'][:1001])
        contrast[sign]={'active_status':cases[sign]['status'],'frozen_input_status':cases[sign+'_no_feedback']['status'],'pre_impact_histories_identical':True,'max_feedback_command_difference_from_no_impulse_n':float(abs(a['control_n'][post]-no['control_n'][post]).max()),'max_active_vs_frozen_command_difference_n':float(abs(a['control_n'][post]-b['control_n'][post]).max()),'max_impact_induced_tip_difference_from_no_impulse_m':float(delta[k]),'impact_induced_peak_time_s':float(a['time_s'][k]),'active_peak_post_impact_speed_m_s':cases[sign]['metrics']['peak_post_impact_tip_speed_m_s'],'frozen_peak_post_impact_speed_m_s':cases[sign+'_no_feedback']['metrics']['peak_post_impact_tip_speed_m_s'],'controller_inputs':'normal clock, qpos, qvel only; virtual predictor has zero xfrc/qfrc and no force input','same_parameters_both_signs':True}
    result={'status':'PASS' if all(cases[k]['status']=='PASS' for k in ['no_impulse','positive_y','negative_y']) else 'FAIL','selected_candidate':'margin_feedback','frozen_task':'task.json','model_sha256':hashlib.sha256((HERE/'model.xml').read_bytes()).hexdigest(),'cases':cases,'force_audit':audit,'active_recovery_contrasts':contrast,'budget':read(HERE/'results/budget_usage.json'),'independent_validation':read(HERE/'results/independent_replay.json'),'limitations':['Peak baseline displacement remains below 5 mm after the one permitted increase to 0.04 N; no additional amplitude adjustment or threshold relaxation.','Positive active case is already within tolerances at pulse end, so post-pulse recovery time is 0; negative case enters at 0.5515 s.','Post-impact displacement from the 0.5 s pose includes normal settling drift. The separate no-impulse difference isolates the impulse contribution.','Frozen-input baselines include passive elasticity and fixed actuation, not zero-tension free release.','Predictive speed constraints failed in early screens and are disabled in the final controller; full physical-step speed is nevertheless below 25 mm/s. Final feasibility is empirical for these fixed cases, not a certified guard.']}
    write(HERE/'results/evaluation.json',result)
    if not (HERE/'controller_bank.npz').exists():shutil.copyfile(src/'controller_bank.npz',HERE/'controller_bank.npz')
    if not (HERE/'effective_control.json').exists():shutil.copyfile(src/'effective_control.json',HERE/'effective_control.json')
    lines=['# M7 finite search record','','| Group | Purpose | Nominal/pre-hold | Main remaining issue |','|---|---|---|---|',
      '| 1 original_structure | Native full-state LQR + predictive guard | FAIL | Large feedback/constraint conflict; invalid calibration |',
      '| 2 softened_feedback | Lower feedback, guard disabled | Pre-hold PASS | Startup rope 46.210 mm/s |',
      '| 3 bounded_feedback | Re-enable conservative prediction | FAIL | Constraint allocation disrupts support; invalid calibration |',
      '| 4 stroke_aware_feedback | Penalize static rope excursion; linear reference with smooth ramps | FAIL | Startup/stop rope and pre-hold speed |',
      '| 5 balanced_stroke_feedback | Balance maximum rope excursion and penalize rope velocities | Pre-hold PASS | Startup rope 34.787 mm/s; valid amplitude observation 3.802 mm |',
      '| 6 startup_shaped_feedback | Ramp feedback over 100 ms | All active cases PASS | Rope margin only 0.088 mm/s |',
      '| 7 margin_feedback | Ramp feedback over 130 ms | All active cases PASS | Selected: rope peak 21.786 mm/s |',
      '', '7 candidate groups (5 one-case screens + 2 five-case evaluations), one independent new-process five-case verification, 20 full case integrations / 40,000 physical steps. No further search, no structure change. All failures retained.',
      '', 'The first screen briefly wrote an amplitude suggestion from an unstable pre-hold. It was explicitly invalidated before any further integration and never used as valid calibration. The one effective adjustment is 0.03 -> 0.04 N, based on the valid pre-hold group 5 baseline. The task then remained frozen.']
    (HERE/'results/search_record.md').write_text('\n'.join(lines)+'\n',encoding='utf8')
    print('M7 selected records copied, force integration and feedback contrasts audited; no physics integration.')
if __name__=='__main__':main()
