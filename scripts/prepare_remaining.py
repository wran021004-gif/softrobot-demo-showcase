"""Copy existing results and add visual-only scenes. No solver or physics integration.
PHS sampling uses the original view_phs2d/phs2d_io state layout verbatim.
"""
import argparse,json,shutil
from pathlib import Path
import numpy as np
from prepare_showcase import decorate_xml
ROOT=Path(__file__).resolve().parents[1]
SOURCE=Path(r'D:\softrobot-benchmark-exports\base_only_accepted_20261028')
FOLDERS={'m2':'M2_loaded_hold','m3':'M3_fast_reach','m4':'M4_legacy_tracking','m5':'M5_tool_axis','m6':'M6_loaded_scan','c1':'C1_four_tendons','phs':'PHS2D_head_stabilization'}
COLORS=[[.63,.28,.82,1],[.02,.64,.69,1],[.92,.65,.08,1],[.15,.34,.8,1]]
def read(p):return json.loads(p.read_text(encoding='utf8'))
def write(p,v):p.write_text(json.dumps(v,indent=2,ensure_ascii=False,allow_nan=False)+'\n',encoding='utf8')
def copy(source,dest,names):
    dest.mkdir(parents=True,exist_ok=True)
    for n in names:shutil.copyfile(source/n,dest/n)
def active(demos):
    for demo in demos:
        out=ROOT/FOLDERS[demo]
        if demo=='c1':
            src=SOURCE/'C1_min_tendons_multi_task'
            copy(src,out,['joint_map.json','design.json','summary.json','task.txt'])
            shutil.copyfile(src/'model.xml',out/'robot.xml')
            for case in ['M1_A','M1_B','M1_C','M5']:
                copy(src/'cases'/case,out/case,['configuration.json','trajectory.csv','control.csv','schedule.json','result.json'])
        else:
            src=SOURCE/demo.upper()
            copy(src,out,['robot.xml','joint_map.json','structure.json','structure.md'])
            for case in (['zero','positive','negative'] if demo=='m2' else ['default']):
                copy(src/case,out/case,['task.json','configuration.json','control_schedule.csv','trajectory_timestep.csv','reference.csv','original_evaluation.json','validation.json'])
        decorate_xml(out/'robot.xml',out/'scene.xml',visual_floor_z=-.10 if demo=='m5' else None)
        if demo=='c1':
            import xml.etree.ElementTree as ET
            tree=ET.parse(out/'scene.xml')
            for i,t in enumerate(tree.getroot().findall('./tendon/spatial')):
                t.set('rgba',' '.join(map(str,COLORS[i])))
                for site in t.findall('site'):
                    s=tree.getroot().find(f'.//site[@name="{site.get("site")}"]')
                    s.set('rgba',' '.join(map(str,COLORS[i])))
            tree.write(out/'scene.xml',encoding='unicode')
        print(demo,'original results copied; visual-only scene ready',flush=True)
def phs():
    src=SOURCE/'PHS2D_passive_head_stabilization';out=ROOT/FOLDERS['phs']
    copy(src,out,['robot.xml','joint_map.json','structure.md','task.json','task_zh.md','payout_lengths.json'])
    shutil.copyfile(src/'dataset.json',out/'original_dataset.json')
    structure=read(src/'structure.json');physics=structure['physics'];dataset=read(src/'dataset.json')
    # Only the original named parts/material bindings needed for kinematic display.
    runtime=dict(parts=[{k:p[k] for k in ['entity','length_m','dofs']} for p in physics['parts']],tip=physics['tip'],total_physical_mass_kg=structure['total_physical_mass_kg'],source='original structure.json physics; no re-discretization')
    write(out/'display_geometry.json',runtime)
    decorate_xml(out/'robot.xml',out/'scene.xml',passive=True,skin_radius=.0033,probe=False)
    sampling=dict(origin='Codex-assisted existing engineering results; not Main autonomous Study',source=str(src),description='DISPLAY SAMPLES ONLY; every 80th original state plus final state, around 100 Hz. Original timestamps, qpos/qvel and external force retained exactly. No interpolation or integration.',original_timestep_s=.000125,display_interval_s=.01,cases={})
    for entry in dataset['trajectories']:
        case=entry['condition'];folder=out/case
        copy(src/case,folder,['initial_state.json','replay_spec.json'])
        if (src/case/'original_evaluation.json').exists():shutil.copyfile(src/case/'original_evaluation.json',folder/'original_evaluation.json')
        with np.load(src/case/'states_timestep.npz',allow_pickle=False) as a:
            raw=a['trajectory'];n=144
            ids=np.unique(np.r_[np.arange(0,len(raw),80),len(raw)-1])
            rows=raw[ids]
            force=np.column_stack([rows[:,1],np.zeros((len(ids),2))])
            np.savez_compressed(folder/'display_replay.npz',source_index=ids,time_s=rows[:,0],qpos=rows[:,2:2+n],qvel=rows[:,2+n:2+2*n],force_n=force,tension_n=rows[:,-4:],tip_m=a['tip_position_m'][ids],tip_rotation=a['tip_rotation'][ids],force_point_m=a['force_point_m'][ids])
            if case=='equilibrium':
                np.savez_compressed(out/'unloaded_baseline.npz',source_index=len(raw)-1,time_s=raw[-1,0],qpos=raw[-1,2:2+n],qvel=raw[-1,2+n:2+2*n],tip_m=a['tip_position_m'][-1])
        # Keep matching force schedule rows, not the 32,001-row original schedule.
        schedule=src/case/'external_force_schedule.csv'
        with schedule.open(encoding='utf8') as f:header=f.readline().strip()
        force_schedule=np.loadtxt(schedule,delimiter=',',skiprows=1,ndmin=2)
        # Its rows correspond to the same saved-state timestamps; preserve columns verbatim.
        assert len(force_schedule)==len(raw)
        np.testing.assert_array_equal(force_schedule[ids,3:6],force)
        np.testing.assert_allclose(force_schedule[ids,2],rows[:,0],atol=1e-11,rtol=0)
        np.savetxt(folder/'external_force_schedule_display.csv',force_schedule[ids],delimiter=',',header=header,comments='',fmt='%.17g')
        sampling['cases'][case]=dict(original_rows=len(raw),display_rows=len(ids),s=entry['s'],force_n=entry['force_n'],material_point_binding=entry['material_point_binding'],source_rows_file='states_timestep.npz',original_npz_bytes=(src/case/'states_timestep.npz').stat().st_size,display_npz_bytes=(folder/'display_replay.npz').stat().st_size)
        del raw,rows,force_schedule
        print('phs',case,len(ids),'original states sampled',flush=True)
    write(out/'display_sampling.json',sampling)
def main():
    p=argparse.ArgumentParser();p.add_argument('--tasks',nargs='+',choices=list(FOLDERS),default=list(FOLDERS));args=p.parse_args()
    active([x for x in args.tasks if x!='phs'])
    if 'phs' in args.tasks:phs()
if __name__=='__main__':main()
