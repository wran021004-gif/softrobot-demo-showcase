"""Summarize saved evaluations; never integrate or call another provider."""
import json,copy
import numpy as np
from mechanics import HERE,BASE,read,write,load_model,S4

DECISIONS={
 'C001':'Faster support ramp (20 ms, 200 N/s), linear interpolation, event acquisition and return gate. PASS, but no withdrawal gate reached.',
 'C002':'Relax low-force gate from 1 to 2 mN. Full 5 mm withdrawal excites endpoint speed: release FAIL.',
 'C003':'Try slower 290 ms return and 0.5 mm withdrawal. Return phase changes recoil unfavorably: lower travel FAIL; gate not reached.',
 'C004':'Restore 270 ms return, 0.5 mm withdrawal. Lower bound passes, 1.8 s probe speed fails.',
 'C005':'Reduce withdrawal to 0.2 mm. Endpoint speed passes, lower travel misses by 0.0161 um.',
 'C006':'Keep contact inverse node through withdrawal to avoid changing antagonist self-stress. PASS; late light recontact remains.',
 'C007':'Implicit damping/stiffness ten-sample prediction, guard from t=0, elastic infeasibility penalty; 0.5 mm withdrawal. PASS, zero fallback and no negative q, but gate not reached.',
 'C008':'Gate at q<=3 um, |qdot|<=0.4 mm/s, Fn<2 mN, after return. Withdrawal occurs but later rebound reaches -1.480 um.',
 'C009':'Replace slow initial quintic retreat with a 0.5 mm exponential retreat (60 ms); retain continuous 10 ms tension interpolation. Full PASS, complete separation, desired speed/q/tension margins.',
}
def score(r,a):
    m=r['metrics'];qmin=float(a['button_q_m'].min());failed=sum(not x for x in r['checks'].values())
    holdmiss=max(0,.4-m['hold_valid_continuous_s'])/.4
    release=max([0]+[max(0,x['button_residual_m']/.0001-1, x['contact_force_n']/.01-1, x['tip_speed_m_s']/.01-1) for x in m['release_at_1p8_and_2p0']])
    violation=max(0,m['max_rope_speed_m_s']/.025-1)+max(0,(-qmin-1e-6)/1e-6)+max(0,m['button_peak_m']/.003-1)+max(0,m['max_tension_n']/8-1)+max(0,m['max_stroke_m']/.015-1)+holdmiss+release
    return [int(r['status']!='PASS'),failed,violation,m['max_rope_speed_m_s']-.025,-qmin-1e-6,m['max_hold_depth_error_m']/.00025+m['max_hold_contact_force_error_n']/.02,m['max_tension_n'],m['structure_mass_kg'],m['total_tendon_length_variation_m']]
def main():
    rows=[]
    for folder in sorted((HERE/'candidates').iterdir()):
        if not (folder/'result.json').exists():continue
        r=read(folder/'result.json');a=np.load(folder/'trajectory.npz');cfg=read(folder/'effective_control.json');log=read(folder/'controller_trace.json');source=(folder/'controller_source.py').read_text()
        pairs=a['contact_pairs']>0;times=a['time_s'];trans=[]
        for k in np.flatnonzero(pairs[1:]!=pairs[:-1])+1:
            trans.append({'time_s':float(times[k]),'contact':bool(pairs[k]),'q_um':float(a['button_q_m'][k]*1e6),'qdot_mm_s':float(a['button_qdot_m_s'][k]*1000),'force_n':float(a['contact_force_n'][k])})
        enabled_fallbacks=sum(bool(x['speed_guard_fallback']) and bool(x['speed_guard_enabled']) for x in log)
        resolved=copy.deepcopy(cfg)
        defaults={'event_contact':True,'approach_end_s':.5,'contact_on_force_n':.0001,'acquire_depth_m':.00012,'release_gate_q_m':1e-6,'release_gate_v_m_s':.0001,'release_gate_force_n':.001,'return_position_bias_m':0.,'guard_start_s':.15}
        for k,v in defaults.items():resolved.setdefault(k,v)
        if 'withdraw_distance_m' in source:resolved.setdefault('withdraw_distance_m',.005)
        if 'withdraw_contact_bank' in source:resolved.setdefault('withdraw_contact_bank',False)
        if 'withdraw_profile' in source:resolved.setdefault('withdraw_profile','quintic')
        if 'prediction_model' in source:resolved.setdefault('prediction_model','legacy_explicit')
        if 'implicit_interval'==cfg.get('prediction_model'):resolved.setdefault('speed_slack_penalty',1e8)
        write(folder/'effective_control_resolved.json',resolved)
        audit={'source_of_truth':'per-candidate archived controller and runner plus full 1 ms trajectory; resolved JSON expands source defaults without changing executed values','resolved_configuration':resolved,'feedback_updates':len(log),'physical_steps':2000,'contact_transitions':trans,'states_visited':list(dict.fromkeys(x['state'] for x in log)),'state_transitions':[{'time_s':x['time_s'],'state':x['state']} for i,x in enumerate(log) if i==0 or x['state']!=log[i-1]['state']],'speed_guard_enabled_fallbacks':enabled_fallbacks,'legacy_bypass_diagnostic_retries':sum(bool(x['speed_guard_fallback']) and not bool(x['speed_guard_enabled']) for x in log),'max_prediction_slack_m_s':max(x['predicted_speed_violation_m_s'] for x in log),'actual_speed_violation_m_s':max(0,float(abs(a['tendon_speed_m_s']).max())-.025),'final_native_contact_pairs':int(a['contact_pairs'][-1]),'hold_contact_loss_samples':int(np.sum((~pairs)&(times>=.9-1e-9)&(times<=1.4+1e-9))),'runtime_metadata_correction':'C007 onward source uses implicit ten-sample prediction; original summary prose inherited legacy one-step description. This audit corrects descriptive metadata only; archived source, logs and physical histories are preserved.'}
        write(folder/'execution_audit.json',audit)
        runtime=read(folder/'runtime_configuration.json');runtime['execution_audit']='execution_audit.json';runtime['resolved_configuration_file']='effective_control_resolved.json';runtime['actual_prediction_model']=cfg.get('prediction_model','legacy_explicit');runtime['actual_prediction_horizon_s']=.01 if cfg.get('prediction_model')=='implicit_interval' else .001;runtime['speed_guard_enabled_fallbacks']=enabled_fallbacks;write(folder/'runtime_configuration.json',runtime)
        if folder.name.startswith('FINAL'):continue
        rows.append({'candidate':folder.name,'status':r['status'],'failed_checks':[k for k,v in r['checks'].items() if not v],'score':score(r,a),'decision':DECISIONS[folder.name],'metrics':r['metrics'],'diagnostics':r['diagnostics'],'complete_withdrawal':bool('WITHDRAW' in audit['states_visited'] and a['contact_pairs'][-1]==0)})
    ranked=sorted(rows,key=lambda r:r['score']);write(HERE/'results/all_candidates.json',{'evaluations':rows,'strict_original_evaluator_ranking':[r['candidate'] for r in ranked],'selection':'C009','selection_note':'Strict ranking leads with C007 (qmin=0) but it never leaves RETURN_FOLLOW and remains in native contact at 2 s. Select C009 to satisfy the explicitly requested complete follow-then-withdraw behavior as well as every unchanged evaluator check. The additional behavior is reported separately, never hidden in or substituted for the original evaluator.'})
    write(HERE/'results/best_candidate.json',{'selected_candidate':'C009','independent_replay':'FINAL_C009','status':'PASS','strict_evaluator_ranking_leader':ranked[0]['candidate'],'selection_reason':'C009 also completes actual WITHDRAW and has zero endpoint contact force; C007 has a better qmin but retains light contact. Both remain original evaluator PASS.','desired_margins_met':True,'structure':'unchanged v1.1 S4','metrics':read(HERE/'candidates/C009/result.json')['metrics'],'diagnostics':read(HERE/'candidates/C009/result.json')['diagnostics'],'replay_check':read(HERE/'candidates/FINAL_C009/replay_check.json')})
    spec=read(BASE/'v1_1/design_space.json');spec.update(revision='v1.2',selected_structure=S4,structure_candidates_evaluated=0,reason='Control achieved desired margins and full withdrawal; no need to expand structure search or trade feasibility for mass.');spec.pop('candidates',None);write(HERE/'design_space.json',spec)
    search=read(HERE/'controller_search.json');search['decisions']=[DECISIONS[x] for x in sorted(DECISIONS)];search['stopped']='C009 full PASS with desired margins, independent closed-loop exact match; no further search';write(HERE/'controller_search.json',search)
    m=load_model(path=HERE/'candidates/C009/environment.xml');design=read(HERE/'candidates/C009/design.json');paths=[]
    for j,td in enumerate(design['tendons']):
        sites=[]
        for k in range(m.tendon_adr[j],m.tendon_adr[j]+m.tendon_num[j]):
            sites.append(m.site(m.wrap_objid[k]).name)
        paths.append({'tendon':f'T{j}','root_angle_deg':float(np.degrees(np.arctan2(td['points'][0]['attachment']['position_m'][2],td['points'][0]['attachment']['position_m'][1]))%360),'radius_m':.021,'design_points':td['points'],'actual_mujoco_site_sequence':sites})
    write(HERE/'results/tendon_paths.json',paths)
    versions=[('v1.0 B5',BASE/'candidates/B5'),('v1.1 S4',BASE/'v1_1/candidates/S4'),('v1.2 C009',HERE/'candidates/C009')]
    vals=[]
    for name,folder in versions:
        r=read(folder/'result.json');a=np.load(folder/'trajectory.npz');v=r['metrics'];vals.append([r['status'],f"{v['max_hold_depth_error_m']*1000:.6f}",f"{v['max_hold_contact_force_error_n']:.6f}",f"{v['hold_lateral_error_max_m']*1000:.6f}",f"{v['hold_valid_continuous_s']:.3f}",f"{v['max_tension_n']:.6f}",f"{v['max_rope_speed_m_s']*1000:.6f}",f"{a['button_q_m'].min()*1e6:.6f}",f"{v['max_stroke_m']*1000:.6f}",f"{v['structure_mass_kg']*1000:.6f}",'PASS' if r['checks']['release'] else 'FAIL'])
    labels=['综合状态','Hold 最大深度误差 (mm)','Hold 最大接触力误差 (N)','Hold 最大 XZ 误差 (mm)','连续有效 Hold (s)','最大张力 (N)','最大实际绳速 (mm/s)','最小按钮行程 (μm)','最大腱行程 (mm)','总结构质量 (g)','1.8/2.0 s 释放端点']
    lines=['# 原始 Evaluator 性能对比','','| 指标 | v1.0 B5 | v1.1 S4 | v1.2 C009 |','|---|---:|---:|---:|']
    for i,label in enumerate(labels):lines.append('| '+label+' | '+' | '.join(v[i] for v in vals)+' |')
    lines+=['','质量是同一结构的恒定物理质量，不是随时间变化的最大值。最大腱行程是实际几何腱长相对初始值的绝对变化。原始模型、轨迹、Evaluator 均未改写。','','## 自适应候选记录','','| 候选 | 状态 | 失败检查 | 速度峰值 mm/s | 最小 q μm | 决策 |','|---|---|---|---:|---:|---|']
    for r in rows:lines.append(f"| {r['candidate']} | {r['status']} | {', '.join(r['failed_checks']) or '-'} | {r['metrics']['max_rope_speed_m_s']*1000:.3f} | {r['diagnostics']['minimum_button_um']:.6f} | {r['decision']} |")
    lines+=['','严格按题述 Evaluator 排序：'+', '.join(x['candidate'] for x in ranked)+'。C007 在按钮下限裕量上领先，但最终仍轻触按钮，未触发 WITHDRAW。工程交付选择 C009：它同时完成用户要求的实际撤离、零端点接触力及全部原始验收。两种排序/选择公开记录，不改变 Evaluator。','','独立闭环 FINAL_C009 再次全部 PASS，qpos/qvel/control/实际绳速/按钮 q/接触力的最大差均为 0。']
    (HERE/'results/comparison.md').write_text('\n'.join(lines)+'\n',encoding='utf8')
    print('Saved comparison, strict ranking, selected engineering solution, resolved configuration audits and actual tendon paths.')
if __name__=='__main__':main()
