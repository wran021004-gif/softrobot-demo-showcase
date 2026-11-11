"""M7 independent native poses and scientific curves; zero integration."""
import sys,json
from pathlib import Path
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE.parent))
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from visualization.palettes import BUGN,DARK2,scientific_style
from visualization.export_scientific import chart,save,CMAP
from visualization.mujoco_renderer import Presentation,forbid_integration
def main():
    forbid_integration();scientific_style();out=HERE/'results/figures';out.mkdir(parents=True,exist_ok=True);names=['no_impulse','positive_y','negative_y','positive_y_no_feedback','negative_y_no_feedback'];a={n:np.load(HERE/'cases'/n/'trajectory.npz') for n in names};target=np.array([.29,-.035,.19]);t=a['no_impulse']['time_s'];report=json.loads((HERE/'results/evaluation.json').read_text(encoding='utf8'));index=[]
    def finish(f,name,title):save(f,out,name);index.append({'stem':name,'title':title,'formats':['600 dpi PNG','vector SVG']})
    f,ax=chart('M7 / actual tip motion','Y (mm)','Z (mm)')
    for name,color,style in [('positive_y',DARK2[0],'-'),('negative_y',DARK2[2],'--')]:
        p=a[name]['tip_m']*1000;ax.plot(p[:,1],p[:,2],c=color,ls=style,label=name.replace('_',' '));ax.scatter(*p[1000,[1,2]],c=color,marker='o',s=16)
    ax.scatter(target[1]*1000,target[2]*1000,marker='D',s=35,facecolors='none',edgecolors=DARK2[1],label='Fixed target');ax.legend();finish(f,'tip_trajectory_yz','Actual YZ projection, full 1 s trajectory')
    for quantity,ylabel,limit,name in [('error','Target error (mm)',10,'position_error_comparison'),('speed','Tip speed (mm/s)',20,'tip_speed_comparison')]:
        f,ax=chart('M7 / active vs frozen input','Physical time (s)',ylabel)
        for sign,color in [('positive_y',DARK2[0]),('negative_y',DARK2[2])]:
            for frozen,style in [(False,'-'),(True,'--')]:
                c=sign+('_no_feedback' if frozen else '');z=a[c];v=np.linalg.norm(z['tip_m']-target,axis=1)*1000 if quantity=='error' else np.linalg.norm(z['tip_velocity_m_s'],axis=1)*1000;ax.plot(t,v,color=color,ls=style,label=('+' if sign.startswith('positive') else '-')+'Y '+('frozen' if frozen else 'active'))
        ax.axhline(limit,c=DARK2[3],ls=':',label='Acceptance limit');ax.axvspan(.5,.55,color=BUGN[1],alpha=.3,label='Pulse');ax.axvspan(.9,1,color=BUGN[0],alpha=.7,label='Final hold');ax.set_xlim(.3,1);ax.set_ylim(0,12 if quantity=='error' else 70)
        legend=ax.legend(loc='upper left',bbox_to_anchor=(1.02,1),frameon=False);legend.set_in_layout(False)
        finish(f,name,'All physical timestep '+quantity+'; pre-hold through final hold')
    f,ax=chart('M7 / frozen half-sine world force','Physical time (s)','Force at payload COM (N)')
    for c,col,ls in [('positive_y',DARK2[0],'-'),('negative_y',DARK2[2],'--')]:ax.plot(t,a[c]['force_n'][:,1],c=col,ls=ls,label=c.replace('_',' '))
    ax.set_xlim(.48,.58);ax.legend();finish(f,'external_force','Actual force at every physical timestep')
    f,ax=chart('M7 / accumulated actual impulse','Physical time (s)','Y impulse (mN s)')
    for c,col,ls in [('positive_y',DARK2[0],'-'),('negative_y',DARK2[2],'--')]:ax.plot(t,a[c]['impulse_ns'][:,1]*1000,c=col,ls=ls,label=c.replace('_',' '))
    ax.set_xlim(.48,.62);ax.legend();finish(f,'impulse','Numerical cumulative applied impulse')
    f,ax=chart('M7 / six actual tensions, positive Y','Physical time (s)','Tendon');im=ax.pcolormesh(t,np.arange(6),a['positive_y']['tension_n'].T,shading='nearest',rasterized=False,cmap=CMAP,vmin=0,vmax=8);ax.set(xlim=(0,1),ylim=(-.5,5.5));ax.set_yticks(range(6),[f'T{j}' for j in range(6)]);bar=f.colorbar(im,ax=ax,label='Tension (N)');bar.solids.set_rasterized(False);finish(f,'six_tendon_tensions','Six-channel actual tension history with continuous BuGn color scale')
    f,ax=chart('M7 / feedback response to unknown direction','Physical time (s)','Largest |u - no-impulse u| (N)')
    for c,col,ls in [('positive_y',DARK2[0],'-'),('negative_y',DARK2[2],'--')]:ax.plot(t,abs(a[c]['control_n']-a['no_impulse']['control_n']).max(axis=1),c=col,ls=ls,label=c.replace('_',' '))
    ax.set_xlim(.45,1);ax.axvspan(.5,.55,color=BUGN[1],alpha=.3);ax.legend();finish(f,'feedback_control_response','Actual state-dependent commands differ after the unknown pulse')
    f,ax=chart('M7 / actual geometric tendon speeds','Physical time (s)','Tendon speed (mm/s)')
    speeds=a['positive_y']['tendon_velocity_m_s']*1000
    for j,ls in enumerate(['-','--','-.',':',(0,(5,2,1,2)),(0,(2,2,1,2))]):ax.plot(t,speeds[:,j],c=BUGN[2] if j%2==0 else DARK2[0],ls=ls,label=f'T{j}')
    for lim in [-25,25]:ax.axhline(lim,c=DARK2[3],ls=':',lw=.8)
    ax.legend(ncol=3);finish(f,'tendon_speeds','Every 0.5 ms actual tendon speed, frozen 25 mm/s limits')
    r=Presentation(HERE.parent,'m7','positive_y');peak=report['active_recovery_contrasts']['positive_y']['impact_induced_peak_time_s'];k=round(peak/.0005)
    for name,i in [('before_impact',980),('maximum_impact_deviation',k),('recovered',1900)]:
        r.frame(i,width=2400,height=1800,hud=False).save(out/f'{name}.png',dpi=(600,600));index.append({'stem':name,'title':f'Native MuJoCo pose at {t[i]:.4f} s','source_index':i,'formats':['2400x1800 PNG, 600 dpi'],'note':'Maximum deviation is relative to the no-impulse control, distinct from normal settling drift.' if name=='maximum_impact_deviation' else ''})
    r.close();(out/'index.json').write_text(json.dumps(index,indent=2)+'\n',encoding='utf8');print('M7: 8 independent scientific PNG/SVG and 3 native pose PNG, zero integration.')
if __name__=='__main__':main()
