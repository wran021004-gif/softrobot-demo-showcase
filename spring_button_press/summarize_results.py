"""Postprocess the completed bounded experiment; no physics or new search."""
import subprocess
import numpy as np
from environment import HERE,CORE,read,write,LABEL

def main():
    # Correct the first run's structural-mass bookkeeping: fixed button housing
    # is environment, not robot. This never changes a physical/acceptance result.
    p=HERE/'baseline/A1/result.json';a=read(p);a['metrics']['structure_mass_kg']=.06732013780996088
    a['bookkeeping_note']='Excluded fixed button housing from robot structural mass; identical C1 mass to A2/A3.';write(p,a)
    rows=[]
    for group,pattern in [('control_only','baseline/*/result.json'),('co_design','candidates/*/result.json')]:
        for p in sorted(HERE.glob(pattern)):
            a=read(p);rows.append({'id':p.parent.name,'group':group,'status':a['status'],
                'failed_checks':[k for k,v in a['checks'].items() if not v],'metrics':a['metrics']})
    write(HERE/'results/all_candidates.json',{'label':LABEL,'selection':'fewest failed hard checks within each group; both selected candidates still FAIL','candidates':rows})
    aa=next(r for r in rows if r['id']=='A1')['metrics'];bb=next(r for r in rows if r['id']=='B5')['metrics']
    fields=[('Task status','FAIL','FAIL'),('Max Hold contact-force error (N)',aa['max_hold_contact_force_error_n'],bb['max_hold_contact_force_error_n']),
        ('Max Hold travel error (mm)',aa['max_hold_depth_error_m']*1000,bb['max_hold_depth_error_m']*1000),
        ('Valid continuous Hold (s)',aa['hold_valid_continuous_s'],bb['hold_valid_continuous_s']),
        ('Max tendon tension (N)',aa['max_tension_n'],bb['max_tension_n']),
        ('Max rope speed (mm/s)',aa['max_rope_speed_m_s']*1000,bb['max_rope_speed_m_s']*1000),
        ('Max stroke from initial (mm)',aa['max_stroke_m']*1000,bb['max_stroke_m']*1000),
        ('Robot structure mass including tool (g)',aa['structure_mass_kg']*1000,bb['structure_mass_kg']*1000),
        ('Total tendon length variation (mm)',aa['total_tendon_length_variation_m']*1000,bb['total_tendon_length_variation_m']*1000),
        ('Max Hold lateral XZ error (mm)',aa['hold_lateral_error_max_m']*1000,bb['hold_lateral_error_max_m']*1000)]
    fmt=lambda x:f'{x:.6g}' if isinstance(x,(float,int)) else x
    lines=['# Control-only versus co-design','',''+LABEL,'','| Metric | Control-only A1 | Co-design B5 |','|---|---:|---:|']
    lines += [f'| {name} | {fmt(x)} | {fmt(y)} |' for name,x,y in fields]
    lines += ['', 'Neither group produced a feasible candidate. B5 improves depth/force tracking but uses more mass/tension, has higher rope speed and fails lateral alignment. This is not an overall co-design success.', '', 'All candidate outcomes:', '', '| ID | Status | Failed hard checks |', '|---|---|---|']
    lines += [f'| {r["id"]} | {r["status"]} | {", ".join(r["failed_checks"])} |' for r in rows]
    (HERE/'results/comparison.md').write_text('\n'.join(lines)+'\n',encoding='utf8')
    static=read(HERE/'precheck_static.json')['forward_evaluations']+read(HERE/'baseline/refinement.json')['forward_evaluations']+read(HERE/'baseline/predictor.json')['forward_evaluations']
    static+=sum(read(p)['forward_evaluations'] for p in HERE.glob('candidates/*/optimization.json'))
    write(HERE/'results/budget_usage.json',{'label':LABEL,'provider_calls':0,'main_calls':0,'reporter_calls':0,
        'prechecks':2,'fixed_structure_control_candidates':3,'co_design_structures':6,
        'final_independent_full_replays':1,'full_task_evaluations':10,'integrated_backend_trajectories':12,
        'physics_steps':20700,'algebraic_inverse_initializer_mj_forward_evaluations':static,
        'extra_robustness_or_step_size_runs':0,'search_stopped':'All allowed structure/control candidates consumed; no complete feasible solution',
        'core_commit':subprocess.check_output(['git','-C',str(CORE),'rev-parse','HEAD'],text=True).strip(),
        'core_changes':'none; task-local adapter only','classification':LABEL})
    print('Summarized 9 candidates and one independent replay.')
if __name__=='__main__':main()
