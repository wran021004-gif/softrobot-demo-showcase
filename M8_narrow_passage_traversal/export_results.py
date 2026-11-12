"""Scientific plots and raw native stills only; no simulation or composition."""
import sys,json,platform
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import imageio.v3 as iio
import mujoco,scipy
from environment import H,read,write,sha
sys.path.insert(0,str(H.parent))
from visualization.palettes import scientific_style,DARK2
from visualization.mujoco_renderer import Presentation,forbid_integration
def main():
    forbid_integration();out=H/'results/figures';out.mkdir(exist_ok=True);a=np.load(H/'results/trajectory.npz');t=a['time_s'];spec=read(H/'task.json');scientific_style();index=[]
    def finish(fig,name):
        fig.tight_layout();fig.savefig(out/(name+'.png'),dpi=600);fig.savefig(out/(name+'.svg'));plt.close(fig);index.append({'name':name,'png_dpi':600,'svg':True})
    fig,ax=plt.subplots(figsize=(6.3,3.5))
    for i,name in enumerate(['baseline_fixed','six_augmented','compact_six']):
        b=np.load(H/'cases'/name/'trajectory.npz');ax.plot(b['time_s'],b['clearance_m']*1000,color=DARK2[i],ls=['-','--','-.'][i],marker=['o','s','^'][i],markevery=1600,ms=3,label=name+' (FAIL)')
    ax.axhline(1,color='#252525',ls=':',label='required 1 mm');ax.set(xlabel='Physical time (s)',ylabel='Minimum whole-body clearance (mm)');ax.legend();finish(fig,'clearance_comparison')
    fig,ax=plt.subplots(figsize=(6.3,3.5));err=np.linalg.norm(a['tip_m']-spec['target_m'],axis=1)*1000;ax.plot(t,err,color=DARK2[0],marker='o',markevery=1600,ms=3,label='actual error');ax.axhline(8,color=DARK2[1],ls='--',label='8 mm limit');ax.axvspan(5.8,6,color='#D9D9D9',alpha=.5,label='final 0.2 s hold');ax.set(xlabel='Physical time (s)',ylabel='Tip position error (mm)');ax.legend();finish(fig,'tip_error')
    for key,title,scale,limit,name in [('tension_n','Actual tendon tension (N)',1,8,'tension'),('tendon_velocity_m_s','Actual rope velocity (mm/s)',1000,25,'rope_speed'),('tendon_length_m','Stroke from initial (mm)',1000,15,'stroke')]:
        fig,ax=plt.subplots(figsize=(6.3,3.5));v=a[key]-(a[key][0] if name=='stroke' else 0)
        for j in range(v.shape[1]):ax.plot(t,v[:,j]*scale,color=DARK2[j%4],ls=['-','--','-.',':'][j%4],marker=['o','s','^','D'][j%4],markevery=1600,ms=2.5,label=f'T{j}')
        ax.axhline(limit,color='#252525',ls='--',lw=.8,label='upper limit')
        if name!='tension':ax.axhline(-limit,color='#252525',ls=':',lw=.8,label='lower limit')
        ax.set(xlabel='Physical time (s)',ylabel=title);ax.legend(ncol=3);finish(fig,name)
    fig,ax=plt.subplots(figsize=(6.3,3.5));ax.plot(t,a['wall_contacts'],color=DARK2[2],drawstyle='steps-post',marker='s',markevery=1600,ms=3,label='native wall contact count');ax.set(xlabel='Physical time (s)',ylabel='Actual robot-wall contact count');ax.legend();finish(fig,'wall_contacts')
    validations={}
    for technical in [False,True]:
        p=Presentation(H.parent,'m8',technical=technical)
        for label,k in [('initial',0),('middle',len(t)//2),('final',len(t)-1)]:
            name=label+('_technical' if technical else '');iio.imwrite(out/(name+'.png'),p.frame(k,width=1600,height=1200,raw=True));validations[name]={'time_s':float(t[k]),**p.verify()}
        p.close()
    # Exactly one historical task is loaded, no historical stepping/evaluation.
    p=Presentation(H.parent,'m1','central_high');validations['historical_m1_load_only']=p.verify();p.close()
    write(H/'results/render_validation.json',validations);write(out/'index.json',{'scientific':index,'native_stills':['initial.png','middle.png','final.png','initial_technical.png','middle_technical.png','final_technical.png'],'raw_native_only':True,'task_status':'FAIL'})
    write(H/'provenance.json',{'classification':spec['classification'],'python':platform.python_version(),'mujoco':mujoco.__version__,'numpy':np.__version__,'scipy':scipy.__version__,'core_commit':'2c83f8b7bd654328dd7648f844295c54a7c15223','showcase_base_commit':'2e5d1088746df0916e2a7ec60f4780998c6ba004','source_model_sha256':{str(p.relative_to(H.parent)):sha(p) for p in [H.parent/'C1_four_tendons/robot.xml',H.parent/'M2_loaded_hold/robot.xml',H.parent/'figure_eight_tracking/robot.xml']},'distance_api_verified':mujoco.mj_geomDistance.__doc__,'units':'SI m, s, kg, N; signed negative distance indicates overlap','distance_api_source':'https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-geomdistance','palette_source':'https://colorbrewer2.org/#type=sequential&scheme=BuGn&n=3','main_calls':0,'reporter_calls':0,'provider_calls':0})
if __name__=='__main__':main()
