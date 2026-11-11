"""MP4 from native MuJoCo frames; source states selected, never interpolated."""
import argparse,csv,json
from pathlib import Path
import numpy as np
import imageio.v2 as imageio
from .mujoco_renderer import Presentation,forbid_integration,TASKS
def export(root,task,case,out,speed=1.,fps=30,width=1280,height=960,tip_detail=False):
    if speed<=0 or fps<=0:raise ValueError('Positive speed and fps required')
    out=Path(out);out.parent.mkdir(parents=True,exist_ok=True);r=Presentation(root,task,case);t=r.case.time
    if tip_detail:
        r.camera.lookat[:]=r.case.reference[0]+[-.012,0,-.004] if r.case.reference is not None else r.tip;r.m.vis.global_.fovy=.09
    nominal=t[0]+np.arange(int(np.ceil((t[-1]-t[0])*fps/speed))+1)*speed/fps;nominal=np.minimum(nominal,t[-1]);indices=np.maximum(0,np.searchsorted(t,nominal,side='right')-1)
    with imageio.get_writer(out,fps=fps,codec='libx264',macro_block_size=16,ffmpeg_params=['-crf','18','-preset','medium'],pixelformat='yuv420p') as writer:
        for index in indices:writer.append_data(np.asarray(r.frame(int(index),width,height,speed=speed)))
    with out.with_suffix('.frames.csv').open('w',newline='',encoding='utf8') as f:
        w=csv.writer(f);w.writerow(['frame_index','source_state_index','physical_time_s','playback_time_s']);w.writerows((i,int(k),float(t[k]),i/fps) for i,k in enumerate(indices))
    report={'task':task,'case':r.case.case,'video':out.name,'view':'tip detail, static camera zoom' if tip_detail else 'whole robot','frame_count':len(indices),'fps':fps,'playback_speed':speed,'physical_duration_s':float(t[-1]-t[0]),'source_state_selection':'left nearest saved physical state; no state interpolation','actual_trail':'source indices <= current physical index only','native_renderer':'mujoco.Renderer','validation':r.verify()};out.with_suffix('.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf8');r.close();print(task,r.case.case,len(indices),'native frames ->',out,flush=True);return report
def main():
    p=argparse.ArgumentParser();p.add_argument('task',choices=TASKS);p.add_argument('--case');p.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[1]);p.add_argument('--output',type=Path,required=True);p.add_argument('--speed',type=float,default=1.);p.add_argument('--fps',type=int,default=30);p.add_argument('--tip-detail',action='store_true');a=p.parse_args();forbid_integration();export(a.root,a.task,a.case,a.output,a.speed,a.fps,tip_detail=a.tip_detail)
if __name__=='__main__':main()
