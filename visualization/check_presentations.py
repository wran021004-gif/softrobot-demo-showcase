"""One case per class, exactly three saved poses. No historical integration."""
import argparse,json
from pathlib import Path
import numpy as np
from .mujoco_renderer import Presentation,forbid_integration
REPRESENTATIVES={'m1':'central_high','m2':'positive','m3':'default','m4':'default','m5':'default','m6':'default','c1':'M1_A','p3':'plus_x','phs2d':'s050_positive','spring_button':'FINAL_C009','m7':'positive_y'}
def main():
    p=argparse.ArgumentParser();p.add_argument('--historical-root',type=Path,required=True);p.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[1]);p.add_argument('--tasks',nargs='+',default=list(REPRESENTATIVES));a=p.parse_args();forbid_integration();out=a.root/'visualization/results';reports=json.loads((out/'presentation_checks.json').read_text(encoding='utf8')) if (out/'presentation_checks.json').exists() else {}
    for task in a.tasks:
        root=a.root if task in ['spring_button','m7'] else a.historical_root;r=Presentation(root,task,REPRESENTATIVES[task]);folder=out/task;folder.mkdir(parents=True,exist_ok=True);poses=[]
        for name,i in [('initial',0),('middle',len(r.case.time)//2),('final',len(r.case.time)-1)]:
            image=r.frame(i);image.save(folder/f'{name}.png',dpi=(600,600));tip_error=None if r.case.tip is None else float(abs(r.tip-r.case.tip[i]).max());validation=r.verify();assert validation['all_physical_arrays_unchanged'] and validation['xml_bytes_unchanged'] and validation['saved_qpos_restored_exactly'] and validation['saved_qvel_restored_exactly'] and not validation['tendon_visualization'];assert tip_error is None or tip_error<1e-9
            poses.append({'image':f'{task}/{name}.png','source_index':i,'physical_time_s':float(r.case.time[i]),'max_tip_reconstruction_error_m':tip_error,'validation':validation})
        reports[task]={'case':r.case.case,'source_root':str(root),'poses':poses,'visual_inspection':'pending manual native-render review','physical_steps':0};r.close();print(task,'three native saved poses, physical invariants unchanged',flush=True)
    (out/'presentation_checks.json').write_text(json.dumps(reports,indent=2)+'\n',encoding='utf8')
if __name__=='__main__':main()
