"""Independent, colorblind-friendly PNG/SVG/PDF exports; no integration."""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image,ImageDraw,ImageFont
import mujoco
from mechanics import HERE,write,LABEL,COLORS
import sys
sys.path.insert(0,str(HERE))
from replay import Replay,forbid_steps,phase
forbid_steps();OUT=HERE/'results/figures';OUT.mkdir(parents=True,exist_ok=True)
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':8,'axes.labelsize':8,'axes.titlesize':8,'legend.fontsize':6,
    'xtick.labelsize':7,'ytick.labelsize':7,'axes.spines.top':False,'axes.spines.right':False,'pdf.fonttype':42,'svg.fonttype':'none','lines.linewidth':1.2})
INDEX=[];STYLES=['-','--','-.',':']
def formats(fig,name,title,png=True):
    for ext in ['pdf','svg']+(['png'] if png else []):
        fig.savefig(OUT/f'{name}.{ext}',dpi=600,bbox_inches='tight',pad_inches=.04,facecolor='white',metadata={'Subject':LABEL} if ext=='pdf' else {'Description':LABEL})
    svg=OUT/f'{name}.svg';svg.write_text('\n'.join(line.rstrip() for line in svg.read_text(encoding='utf8').splitlines())+'\n',encoding='utf8')
    plt.close(fig);INDEX.append({'stem':name,'title':title})
def chart(title,ylabel):
    fig,ax=plt.subplots(figsize=(3.4,2.55),layout='constrained');ax.set(title=title,xlabel='Time (s)',ylabel=ylabel);ax.axvspan(.9,1.4,color='#cccccc',alpha=.25,label='Hold window');return fig,ax
def main():
    r=Replay();a=r.a;t=a['time_s'];first=int(np.flatnonzero(a['contact_pairs']>0)[0]);maximum=int(a['button_q_m'].argmax())
    entries=[('initial_technical',0,'overall','technical'),('initial_presentation',0,'overall','presentation'),('base_guides',0,'base','technical'),
        ('midpoint_guides',1200,'segment0_mid','technical'),('junction_guides',1200,'junction','technical'),('distal_guides',1200,'segment1_end','technical'),
        ('contact',first,'contact','technical'),('maximum_press',maximum,'contact','technical'),('hold_technical',1200,'overall','technical'),
        ('hold_presentation',1200,'overall','presentation'),('hold_close',1200,'contact','technical'),('release',1650,'contact','technical')]
    for name,i,kind,view in entries:
        im=r.image(i,kind,view);Image.fromarray(im).save(OUT/f'{name}.png',dpi=(600,600))
        fig,ax=plt.subplots(figsize=(4,3));ax.imshow(im);ax.axis('off');fig.subplots_adjust(left=0,right=1,bottom=0,top=1);formats(fig,name,f'{name.replace("_"," ")} at {t[i]:.3f} s',png=False)
    f,ax=chart('S4 button travel (overall FAIL)','Button travel (mm)');ax.plot(t,a['button_q_m']*1000,color=COLORS[0],label='Actual slide q');ax.axhspan(1.75,2.25,color=COLORS[0],alpha=.10,label='Required Hold range');ax.axhline(2,color=COLORS[1],ls='--',label='Target');ax.legend();formats(f,'button_travel','Actual button slide travel')
    f,ax=chart('Actual probe-cap contact force','Normal force (N)');ax.plot(t,a['contact_force_n'],color=COLORS[0],label='mj_contactForce');ax.axhspan(.08,.12,color=COLORS[0],alpha=.10,label='Required Hold range');ax.axhline(.1,color=COLORS[1],ls='--',label='Target');ax.legend();formats(f,'contact_force','True native contact force')
    f,ax=chart('Probe-cap lateral alignment','XZ center error (mm)');ax.plot(t,a['lateral_error_m']*1000,color=COLORS[2],label='Actual XZ');ax.axhline(2,color='#333333',ls='--',label='Limit');ax.legend();formats(f,'lateral_error','Lateral alignment')
    f,ax=chart('Four independent tendon tensions','Tension (N)')
    for j in range(4):ax.plot(t,a['tension_n'][:,j],color=COLORS[j],ls=STYLES[j],label=f'T{j}')
    ax.axhline(8,color='#333333',ls='--',lw=.8,label='Limit');ax.legend(ncol=2);formats(f,'tendon_tensions','Actual four-channel tension')
    f,ax=chart('Actual rope speed: limit exceeded','Rope speed (mm/s)')
    for j in range(4):ax.plot(t,a['tendon_speed_m_s'][:,j]*1000,color=COLORS[j],ls=STYLES[j],label=f'T{j}')
    for y in [-25,25]:ax.axhline(y,color='#333333',ls='--',lw=.8)
    ax.legend(ncol=2);formats(f,'rope_speed','All 1 ms analytical tendon velocities, no smoothing')
    f,ax=chart('Lower-stop excursions: limit exceeded','Button travel (micrometre)');ax.plot(t,a['button_q_m']*1e6,color=COLORS[0],label='Actual q');ax.axhline(-1,color='#333333',ls='--',label='Frozen lower tolerance');ax.set_ylim(-7,12);ax.legend();formats(f,'lower_stop','Lower-stop excursions; intentionally restricted Y view')
    f,ax=chart('Probe speed and release endpoints','Probe speed (mm/s)');ax.plot(t,a['tip_speed_m_s']*1000,color=COLORS[2],label='Actual probe speed');ax.axhline(10,color='#333333',ls='--',label='Endpoint limit');ax.scatter(t[[1800,2000]],a['tip_speed_m_s'][[1800,2000]]*1000,color=COLORS[1],marker='s',label='Release checks');ax.legend();formats(f,'probe_speed','Probe speed and frozen release endpoints')
    try:font=ImageFont.truetype(r'C:\Windows\Fonts\arial.ttf',17)
    except OSError:font=ImageFont.load_default()
    for kind in ['overall','contact']:
        frames=[];r.set_view('technical' if kind=='overall' else 'presentation')
        with mujoco.Renderer(r.m,width=900,height=675) as ren:
            for i in np.unique(np.r_[np.arange(0,2001,33),2000]):
                r.restore(int(i));r.camera_view(kind);ren.update_scene(r.d,camera=r.camera,scene_option=r.option);r.decorate(ren.scene,int(i));im=Image.fromarray(ren.render().copy());draw=ImageDraw.Draw(im)
                draw.rectangle((0,0,900,78),fill='white');draw.text((10,4),f'v1.1 S4 / FAIL | {t[i]:.3f} s | {phase(t[i])}\nButton {a["button_q_m"][i]*1000:.3f} mm | Normal force {a["contact_force_n"][i]:.4f} N\nRemaining failures: 27.83 mm/s rope peak; lower-stop rebound',font=font,fill='black');frames.append(im)
        frames[0].save(HERE/f'results/replay_{kind}.gif',save_all=True,append_images=frames[1:],duration=33,loop=0,optimize=True)
    write(OUT/'index.json',{'label':LABEL,'candidate':'S4','status':'FAIL','full_data_states':2001,'native_pose_pixels':[2400,1800],'png_dpi':600,
        'palette':'ColorBrewer Dark2 three-class + gray; distinct line styles','independent_figures':INDEX})
    lines=['# Independent v1.1 figures','','All S4 images retain FAIL status; no combined figure is exported. Every chart uses all 2,001 final replay states.','','| Image | PDF | SVG | PNG |','|---|---|---|---|']
    for e in INDEX:
        n=e['stem'];lines.append(f'| {e["title"]} | [PDF]({n}.pdf) | [SVG]({n}.svg) | [PNG]({n}.png) |')
    (OUT/'README.md').write_text('\n'.join(lines)+'\n',encoding='utf8');print('Exported',len(INDEX),'independent figures and two saved-state GIFs.')
if __name__=='__main__':main()
