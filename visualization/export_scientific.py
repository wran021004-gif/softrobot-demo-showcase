"""Core 2-D scientific figures, original saved data only; PNG 600 dpi + SVG."""
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
from .palettes import BUGN,DARK2,scientific_style
from .adapters import load
from .check_presentations import REPRESENTATIVES
CMAP=LinearSegmentedColormap.from_list('ColorBrewer_BuGn_3',BUGN)
def save(fig,folder,name):
    folder.mkdir(parents=True,exist_ok=True)
    extra=[ax.get_legend() for ax in fig.axes if ax.get_legend() is not None]
    fig.savefig(folder/f'{name}.png',dpi=600,bbox_inches='tight',bbox_extra_artists=extra,pad_inches=.04,facecolor='white');fig.savefig(folder/f'{name}.svg',bbox_inches='tight',bbox_extra_artists=extra,pad_inches=.04,facecolor='white');plt.close(fig)
    svg=folder/f'{name}.svg';svg.write_text('\n'.join(line.rstrip() for line in svg.read_text(encoding='utf8').splitlines())+'\n',encoding='utf8')
def chart(title,xlabel,ylabel):
    f,ax=plt.subplots(figsize=(3.7,2.7),layout='constrained');ax.set(title=title,xlabel=xlabel,ylabel=ylabel);return f,ax
def main():
    p=argparse.ArgumentParser();p.add_argument('--historical-root',type=Path,required=True);p.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[1]);a=p.parse_args();scientific_style();index=[]
    for task,case in REPRESENTATIVES.items():
        if task=='m7':continue
        c=load(a.root if task=='spring_button' else a.historical_root,task,case);out=a.root/f'visualization/results/{task}'
        if task in ['m1','m2','m3']:
            f,ax=chart(task.upper()+(' / development draft' if task=='m3' else ' / frozen result'),'Time (s)','Tip target error (mm)');ax.plot(c.time,np.linalg.norm(c.tip-c.reference,axis=1)*1000,color=BUGN[2],label='Actual saved states');ax.axhline(10,color=DARK2[3],ls='--',label='10 mm position level')
            if c.metadata['hold_window']:ax.axvspan(*c.metadata['hold_window'],color=BUGN[1],alpha=.3,label='Original hold window')
            ax.legend();name='position_error'
        elif task=='m4':
            f,ax=chart('M4 / frozen timed polyline','Y (mm)','Z (mm)');ax.plot(c.reference[:,1]*1000,c.reference[:,2]*1000,color=DARK2[1],ls='--',label='Reference');ax.plot(c.tip[:,1]*1000,c.tip[:,2]*1000,color=BUGN[2],label='Actual');ax.scatter(c.reference[-1,1]*1000,c.reference[-1,2]*1000,c=DARK2[1],marker='D',s=18,label='Final reference');ax.legend();name='tracking_yz'
        elif task in ['m5','m6']:
            ref=c.reference_axis/np.linalg.norm(c.reference_axis,axis=1)[:,None];axis=c.actual_axis/np.linalg.norm(c.actual_axis,axis=1)[:,None];error=np.degrees(np.arccos(np.clip(np.sum(axis*ref,axis=1),-1,1)))
            f,ax=chart(task.upper()+' / tool-axis transient','Time (s)','Axis error (deg)');ax.plot(c.time,error,color=BUGN[2],label='Actual tool +X vs reference');ax.legend();name='tool_axis_error'
        elif task=='c1':
            f,ax=chart('C1 / four actual tension channels','Time (s)','Tendon');im=ax.pcolormesh(c.time,np.arange(4),c.tension.T,cmap=CMAP,vmin=0,vmax=8,shading='nearest',rasterized=False);ax.set(xlim=(c.time[0],c.time[-1]),ylim=(-.5,3.5));ax.set_yticks(range(4),[f'T{i}' for i in range(4)]);bar=f.colorbar(im,ax=ax,label='Tension (N)');bar.solids.set_rasterized(False);name='tension_channels'
        elif task=='p3':
            x=(c.force_point[:,0]-c.force_point[0,0])*1000;y=c.force[:,0];f,ax=chart('P3 / passive coarse-cell mechanism','Load-point displacement X (mm)','Applied force X (N)');ax.plot(x,y,color=BUGN[2],label='Actual loading / recovery');ax.scatter(x[0],y[0],marker='o',s=20,c=DARK2[1],label='Start');ax.scatter(x[-1],y[-1],marker='s',s=20,c=DARK2[2],label='End');ax.legend();name='force_displacement'
        elif task=='phs2d':
            delta=(c.tip-c.tip[0])*1000;f,ax=chart('PHS-2D / XY only; Z can move','Tip displacement X (mm)','Tip displacement Y (mm)');ax.plot(delta[:,0],delta[:,1],color=BUGN[2],label='Actual, from initial saved state');ax.scatter(0,0,marker='+',c=DARK2[1],s=50,label='Initial');ax.scatter(*delta[-1,:2],marker='s',c=DARK2[2],s=18,label='Final');ax.legend();name='tip_horizontal_xy'
        else:
            f,ax=chart('C009 / unchanged original PASS','Time (s)','Button travel (mm)');ax.plot(c.time,c.metadata['button_q_m']*1000,color=BUGN[2],label='Actual button q');ax.axvspan(.9,1.4,color=BUGN[1],alpha=.25,label='Hold');ax.axhline(2,color=DARK2[1],ls='--',label='Target');ax.legend();name='button_travel'
        save(f,out,name);index.append({'task':task,'case':case,'stem':f'{task}/{name}','data':'original saved states, no new physical evaluation','formats':['600 dpi PNG','vector SVG']})
    (a.root/'visualization/results/scientific_index.json').write_text(json.dumps(index,indent=2)+'\n',encoding='utf8');print('10 independent core scientific figures; no integration.')
if __name__=='__main__':main()
