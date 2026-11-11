"""Representative videos for ten frozen task classes and all five M7 cases."""
import argparse,json
from pathlib import Path
from .video_export import export
from .mujoco_renderer import forbid_integration
from .check_presentations import REPRESENTATIVES
def main():
    p=argparse.ArgumentParser();p.add_argument('--historical-root',type=Path,required=True);p.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[1]);a=p.parse_args();forbid_integration();reports=[]
    for task,case in REPRESENTATIVES.items():
        if task=='m7':continue
        root=a.root if task=='spring_button' else a.historical_root;speed=.2 if task in ['m1','m2','m3','m4','m5','m6','c1'] else 1.
        reports.append(export(root,task,case,a.root/f'visualization/results/videos/{task}_{case}.mp4',speed))
    for case in ['no_impulse','positive_y','negative_y','positive_y_no_feedback','negative_y_no_feedback']:
        reports.append(export(a.root,'m7',case,a.root/f'M7_external_impact_active_recovery/results/videos/{case}.mp4',.2))
    reports.append(export(a.root,'m7','positive_y',a.root/'M7_external_impact_active_recovery/results/videos/positive_y_tip_detail.mp4',.2,tip_detail=True))
    (a.root/'visualization/results/video_index.json').write_text(json.dumps(reports,indent=2)+'\n',encoding='utf8')
if __name__=='__main__':main()
