"""Independent result images and animation, from saved states only."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image,ImageDraw,ImageFont
from environment import HERE,LABEL,write
from replay import Replay,forbid_steps,COLORS,phase
forbid_steps()
OUT=HERE/'results/figures';OUT.mkdir(parents=True,exist_ok=True)
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':8,'axes.labelsize':8,
    'axes.titlesize':8,'legend.fontsize':7,'xtick.labelsize':7,'ytick.labelsize':7,
    'axes.spines.top':False,'axes.spines.right':False,'axes.linewidth':.7,
    'pdf.fonttype':42,'svg.fonttype':'none','lines.linewidth':1.2})
STYLES=['-','--','-.',':']
INDEX=[]
def save(fig,name,title):
    for ext in ['pdf','svg','png']:
        metadata={'Subject':LABEL} if ext=='pdf' else {'Description':LABEL}
        fig.savefig(OUT/f'{name}.{ext}',dpi=600,bbox_inches='tight',pad_inches=.04,facecolor='white',metadata=metadata)
    plt.close(fig);INDEX.append({'stem':name,'title':title})

def chart(title,ylabel):
    f,a=plt.subplots(figsize=(3.4,2.55),layout='constrained');a.set(title=title,xlabel='Time (s)',ylabel=ylabel)
    a.axvspan(.9,1.4,color='#cccccc',alpha=.25,label='Hold window')
    return f,a

def main():
    r=Replay();a=r.a;t=a['time_s'];contact=int(np.flatnonzero(a['contact_pairs']>0)[0]);peak=int(a['button_q_m'].argmax())
    cases=[('initial',0,False),('contact',contact,True),('maximum_press',peak,True),('hold',1200,False),('release',1600,True)]
    for name,i,close in cases:
        im=r.image(i,close=close);Image.fromarray(im).save(OUT/f'{name}_render.png')
        fig,ax=plt.subplots(figsize=(3.4,2.55));ax.imshow(im);ax.axis('off');ax.set_title(f'B5 {name.replace("_"," ")} | {t[i]:.3f} s',fontsize=8)
        fig.subplots_adjust(left=0,right=1,bottom=0,top=.92);save(fig,name,f'B5 {name}')
    im=r.image(1200,close=True);Image.fromarray(im).save(OUT/'hold_close_render.png')
    fig,ax=plt.subplots(figsize=(3.4,2.55));ax.imshow(im);ax.axis('off');ax.set_title('B5 Hold: actual probe-cap contact',fontsize=8);fig.subplots_adjust(left=0,right=1,bottom=0,top=.92);save(fig,'hold_close','B5 Hold close-up')
    f,ax=chart('Button travel (B5; overall FAIL)','Button travel (mm)')
    ax.axhspan(1.75,2.25,color=COLORS[0],alpha=.12,label='Required Hold range')
    ax.plot(t,a['button_q_m']*1000,color=COLORS[0],label='Actual slide state');ax.axhline(2,color=COLORS[1],ls='--',lw=1,label='2 mm target');ax.legend(fontsize=6);save(f,'button_travel','Button travel')
    f,ax=chart('True probe-cap normal force','Normal force (N)');ax.axhspan(.08,.12,color=COLORS[0],alpha=.12,label='Required Hold range')
    ax.plot(t,a['contact_force_n'],color=COLORS[0],label='mj_contactForce');ax.axhline(.1,color=COLORS[1],ls='--',lw=1,label='0.10 N target');ax.legend(fontsize=6);save(f,'contact_force','Actual contact normal force')
    f,ax=chart('Four independent tendon tensions','Tension (N)')
    for i in range(4):ax.plot(t,a['tension_n'][:,i],color=COLORS[i],ls=STYLES[i],label=f'T{i}')
    ax.axhline(8,color='#333333',ls='--',lw=.8,label='8 N limit');ax.legend(ncol=2,fontsize=6);save(f,'tendon_tensions','Tendon tensions')
    f,ax=chart('Probe-button XZ center error','Lateral error (mm)');ax.plot(t,a['lateral_error_m']*1000,color=COLORS[2],label='Actual XZ error');ax.axhline(2,color='#333333',ls='--',label='2 mm limit');ax.legend(fontsize=6);save(f,'lateral_error','Failed lateral alignment constraint')
    f,ax=chart('Actual geometric rope speed','Rope speed (mm/s)')
    for i in range(4):ax.plot(t,a['tendon_speed_m_s'][:,i]*1000,color=COLORS[i],ls=STYLES[i],label=f'T{i}')
    ax.axhline(25,color='#333333',ls='--',lw=.8);ax.axhline(-25,color='#333333',ls='--',lw=.8);ax.legend(ncol=2,fontsize=6);save(f,'rope_speed','Failed rope speed constraint')
    f,ax=chart('Measured force and passive reactions','Force (N)')
    for data,col,ls,label in [(a['contact_force_n'],COLORS[0],'-','Contact normal'),(50*a['button_q_m'],COLORS[1],'--','Spring kq'),(.2*a['button_qdot_m_s'],COLORS[2],'-.','Damping c qdot')]:ax.plot(t,data,color=col,ls=ls,label=label)
    ax.legend(fontsize=6);save(f,'force_crosscheck','Contact versus spring/damping (separate quantities)')
    # Lightweight visual replay. Only restores saved states and mj_forward.
    frames=[]
    try:font=ImageFont.truetype(r'C:\Windows\Fonts\arial.ttf',17)
    except OSError:font=ImageFont.load_default()
    with __import__('mujoco').Renderer(r.m,height=675,width=900) as ren:
        r.camera_view(True)
        for i in np.unique(np.r_[np.arange(0,2001,33),2000]):
            r.restore(int(i));ren.update_scene(r.d,camera=r.camera);r.decorate(ren.scene,int(i));im=Image.fromarray(ren.render().copy());draw=ImageDraw.Draw(im)
            draw.rectangle((0,0,900,90),fill='white');draw.text((12,6),f'B5 / FAIL | {t[i]:.3f} s | {phase(t[i])}\nButton {a["button_q_m"][i]*1000:.3f} mm | Normal force {a["contact_force_n"][i]:.4f} N\nXZ error {a["lateral_error_m"][i]*1000:.3f} mm (limit 2 mm)',fill='black',font=font)
            frames.append(im)
    frames[0].save(HERE/'results/button_press_replay.gif',save_all=True,append_images=frames[1:],duration=33,loop=0,optimize=True)
    write(OUT/'index.json',{'label':LABEL,'candidate':'B5','status':'FAIL','native_renders_px':[2400,1800],'png_dpi':600,'independent_panels':INDEX,'palette':'ColorBrewer Dark2 (3) + gray; redundant line styles'})
    lines=['# Independent Spring Button Press figures','','All files depict the best failed co-design candidate B5. No combined figure is delivered.','','| Panel | PDF | SVG | PNG |','|---|---|---|---|']
    for entry in INDEX:
        n=entry['stem'];lines.append(f'| {entry["title"]} | [PDF]({n}.pdf) | [SVG]({n}.svg) | [PNG]({n}.png) |')
    (OUT/'README.md').write_text('\n'.join(lines)+'\n',encoding='utf8')
    print('Exported',len(INDEX),'independent panels and GIF; no physics integration.')
if __name__=='__main__':main()
