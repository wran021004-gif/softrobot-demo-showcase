"""Paper figures from frozen saved states. This process forbids integration.
Run with Showcase's Python environment. Raw exports are read in place.
"""
import argparse, json, sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import mujoco
from PIL import Image

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'viewer'))
from showcase import Replay
from saved_states import forbid_integration, centerline, material_point
forbid_integration()
OUT = ROOT / 'paper_figures'
RAW = Path(r'D:\softrobot-benchmark-exports\base_only_accepted_20261028')
ORANGE, GREEN, BLUE, RED = '#e68a26', '#24965c', '#3378b5', '#ce4343'
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':7,'axes.labelsize':7,
    'axes.titlesize':7,'legend.fontsize':6,'lines.linewidth':1.1,
    'axes.spines.top':False,'axes.spines.right':False,'axes.linewidth':.6,
    'xtick.labelsize':6,'ytick.labelsize':6,'pdf.fonttype':42,'svg.fonttype':'none',
    'savefig.dpi':600})
META = {'physics_steps':0, 'render_pixels':[2400,1800], 'png_dpi':600,
        'camera_direction':{'azimuth_deg':115,'elevation_deg':-18},'figures':{},'renders':[]}

def write(p, a):
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(a,indent=2,allow_nan=False)+'\n',encoding='utf8')

def select(t, y, limit=3500):
    # Display reduction only; include per-channel global extrema. No smoothing.
    y=np.asarray(y); flat=y.reshape(len(y),-1)
    ids=np.unique(np.r_[np.linspace(0,len(t)-1,min(limit,len(t)),dtype=int),
        np.argmin(flat,axis=0),np.argmax(flat,axis=0)])
    return np.asarray(t)[ids],y[ids]

def line(ax,t,y,**kw):
    a,b=select(t,y); ax.plot(a,b,**kw)

def panel(ax,label,title):
    ax.set_title(title,loc='left',pad=5)

def render(r, i, name, close=False, unloaded=False):
    r.restore(i)
    if unloaded:
        r.d.qpos[:]=r.unloaded_qpos;r.d.qvel[:]=r.unloaded_qvel
        mujoco.mj_forward(r.m,r.d)
        r.current_centerline=centerline(r.m,r.d,r.geometry)
    if r.demo!='p3':
        expected=r.unloaded_tip if unloaded else r.tip[i]
        assert np.max(abs(r.d.site_xpos[r.tip_id]-expected))<1e-10
    else:
        b=r.m.body(r.binding['body_name']).id
        p=r.d.xpos[b]+r.d.xmat[b].reshape(3,3)@r.binding['local_position_m']
        assert np.max(abs(p-r.point[i]))<1e-10
    m=r.m;m.vis.global_.offwidth=2400;m.vis.global_.offheight=1800
    m.vis.headlight.ambient[:]=[.55,.55,.55];m.vis.headlight.diffuse[:]=[.65,.65,.65]
    # Appearance-only changes on the in-memory model; no XML/state writes.
    m.vis.rgba.haze[:]=[.96,.97,.98,1]
    for g in range(m.ngeom):
        if m.geom_type[g]==mujoco.mjtGeom.mjGEOM_PLANE:
            m.geom_rgba[g]=[.94,.95,.96,1];m.geom_matid[g]=-1
    c=mujoco.MjvCamera();mujoco.mjv_defaultCamera(c)
    c.azimuth=115;c.elevation=-18
    if r.demo=='p3':c.lookat[:]=[0,0,.25];c.distance=.70
    elif r.demo=='phs':
        c.lookat[:]=[0,0,.29];c.distance=.78
        if close:
            p=r.point[i]; tip=r.tip[i]
            c.lookat[:]=(p+tip)/2;c.distance=max(.20,float(np.linalg.norm(p-tip))*2.5)
            c.azimuth=100;c.elevation=-10
    else:c.lookat[:]=[.14,0,.15];c.distance=.57
    opt=mujoco.MjvOption();opt.geomgroup[3]=1
    with mujoco.Renderer(m,height=1800,width=2400) as ren:
        ren.update_scene(r.d,camera=c,scene_option=opt)
        if not unloaded:r.decorate(ren.scene,i)
        if r.demo=='phs' and not unloaded:
            g=ren.scene.geoms[ren.scene.ngeom];ren.scene.ngeom+=1
            mujoco.mjv_initGeom(g,mujoco.mjtGeom.mjGEOM_SPHERE,np.array([.0025]*3),r.unloaded_tip,np.eye(3).ravel(),np.array([1,.48,.08,1],np.float32))
        if r.demo=='phs' and close:
            # A real 20 mm world-space segment at the force-point depth.
            a=c.lookat+np.array([-.035,0,-.035]);b=a+np.array([0,0,.02])
            g=ren.scene.geoms[ren.scene.ngeom];ren.scene.ngeom+=1
            mujoco.mjv_initGeom(g,mujoco.mjtGeom.mjGEOM_LINE,np.zeros(3),a,np.eye(3).ravel(),np.array([.1,.15,.2,1],np.float32))
            mujoco.mjv_connector(g,mujoco.mjtGeom.mjGEOM_LINE,4,a,b)
            g.label=''
        im=ren.render().copy()
    p=OUT/'renders'/f'{name}.png';p.parent.mkdir(parents=True,exist_ok=True)
    Image.fromarray(im).save(p)
    META['renders'].append({'path':str(p.relative_to(ROOT)).replace('\\','/'),'demo':r.demo,
        'case':r.case,'saved_time_s':float(r.d.time),'saved_index':int(i),
        'unloaded':unloaded,'close':close,'camera':{'lookat_m':c.lookat.tolist(),
        'distance_m':c.distance,'azimuth_deg':c.azimuth,'elevation_deg':c.elevation},
        'projection':'perspective; original world geometry; no deformation scaling'})
    return im

def picture(ax,im,label,title):
    ax.imshow(im);ax.axis('off');panel(ax,label,title)

def save(fig,name,sources):
    folder=OUT/'main';folder.mkdir(parents=True,exist_ok=True)
    # Save each viewport/chart separately at native vector quality. The layout
    # is only an internal authoring canvas, never an exported composite.
    export_axes(fig,folder,name,sources)
    plt.close(fig)
    print('EXPORTED',name,flush=True)

def export_axes(fig,folder,name,sources):
    fig.canvas.draw();renderer=fig.canvas.get_renderer()
    for j,ax in enumerate(fig.axes):
        stem=f'{name}_{chr(97+j)}';box=ax.get_tightbbox(renderer).transformed(fig.dpi_scale_trans.inverted()).padded(.045)
        for ext in ['pdf','svg','png']:
            fig.savefig(folder/f'{stem}.{ext}',dpi=600,facecolor='white',bbox_inches=box)
        META['figures'][stem]={'title':ax.get_title(loc='left'),'size_inches':[box.width,box.height],
            'sources':sources,'formats':['pdf','svg','png'],'full_data_statistics':True,
            'independent_panel':True}

def m1():
    fig=plt.figure(figsize=(7,4.7),layout='constrained');gs=fig.add_gridspec(2,6,height_ratios=[1.3,1])
    rr=[]
    for j,case in enumerate(['central_high','positive_high','negative_low']):
        r=Replay('m1',case);rr.append(r)
        picture(fig.add_subplot(gs[0,2*j:2*j+2]),render(r,-1,f'm1_{case}'),chr(97+j),case.replace('_',' '))
    a=fig.add_subplot(gs[1,:3]);b=fig.add_subplot(gs[1,3:])
    for r,col in zip(rr,[GREEN,BLUE,ORANGE]):
        a.plot(r.tip[:,1]*1000,r.tip[:,2]*1000,color=col,label=r.case.replace('_',' '))
        a.scatter(r.reference[-1,1]*1000,r.reference[-1,2]*1000,color=col,marker='x',s=25)
        line(b,r.times,np.linalg.norm(r.tip-r.reference,axis=1)*1000,color=col,label=r.case.replace('_',' '))
    panel(a,'d','Independent tip paths; crosses = targets');a.set(xlabel='Y (mm)',ylabel='Z (mm)');a.legend()
    panel(b,'e','Position error');b.set(xlabel='Time (s)',ylabel='Error (mm)');b.axvspan(.54,.6,color=GREEN,alpha=.09);b.legend()
    save(fig,'fig01_m1_reach_hold',{'data':'M1_reach_hold/*/trajectory_timestep.csv','rows_per_case':[len(r.times) for r in rr],'sampling':'all saved physical timesteps; three independent experiments'})

def eight():
    r=Replay('eight');fig=plt.figure(figsize=(7,4.8),layout='constrained');gs=fig.add_gridspec(2,3,height_ratios=[1.3,1])
    for j,t in enumerate([5,10,15]):
        i=int(np.argmin(abs(r.times-t)))
        picture(fig.add_subplot(gs[0,j]),render(r,i,f'eight_{t}s'),chr(97+j),f't = {t} s')
    a,b,c=[fig.add_subplot(gs[1,j]) for j in range(3)]
    a.plot(r.reference[:,1]*1000,r.reference[:,2]*1000,color=ORANGE,label='Reference');a.plot(r.tip[:,1]*1000,r.tip[:,2]*1000,color=GREEN,label='Actual')
    a.set(xlabel='Y (mm)',ylabel='Z (mm)');a.legend();panel(a,'d','Tip trajectory')
    line(b,r.times,np.linalg.norm(r.tip-r.reference,axis=1)*1000,color=BLUE);b.set(xlabel='Time (s)',ylabel='Error (mm)');panel(b,'e','Full-run error')
    for j in range(4):line(c,r.times,r.tension[:,j],label=f'T{j}')
    c.set(xlabel='Time (s)',ylabel='Tension (N)');c.legend(ncol=2);panel(c,'f','Four tendon tensions')
    save(fig,'fig02_figure_eight',{'data':'figure_eight_tracking/trajectory_full.npz','rows':len(r.times),'sampling':'complete 1 ms states; display reduction includes global extrema'})

def p3():
    r=Replay('p3','plus_x');p=RAW/'P3_passive_progressive_stiffness/cases/plus_x/trajectory.csv'
    a=np.genfromtxt(p,delimiter=',',names=True);names=list(a.dtype.names)
    r.times=a['time_s'];r.qpos=np.column_stack([a[f'qpos_{i}'] for i in range(6)])
    r.qvel=np.column_stack([a[f'qvel_{i}'] for i in range(6)])
    r.point=np.column_stack([a[f'point_{v}_m'] for v in 'xyz']);r.force=np.column_stack([a[f'force_{v}_n'] for v in 'xyz'])
    r.tension=np.column_stack([a[f'tension_n_{i}'] for i in range(2)]);r.stages=a['stage'].astype(int)
    fig=plt.figure(figsize=(7,4.7),layout='constrained');gs=fig.add_gridspec(2,4,height_ratios=[1.3,1])
    for j,stage in enumerate([0,1,3,4]):
        i=np.flatnonzero(r.stages==stage)[-1]
        picture(fig.add_subplot(gs[0,j]),render(r,int(i),f'p3_stage_{stage}'),chr(97+j),r.summary['stage_names'][stage])
    b=fig.add_subplot(gs[1,:2]);c=fig.add_subplot(gs[1,2:]);dx=(r.point[:,0]-r.point[0,0])*1000
    ii=np.unique(np.r_[np.linspace(0,len(a)-1,3500,dtype=int),np.argmax(dx)])
    b.plot(dx[ii],r.force[ii,0],color=GREEN,alpha=.6,label='Full run (display subset)')
    met=r.summary['cases']['plus_x']['evaluation']['metrics'];disp=np.array(met['signed_displacements_m'])*1000
    b.plot(np.r_[0,disp], [0,.05,.10,.20],'o-',color=ORANGE,label='Original scored plateaus')
    b.set(xlabel='Load-point X displacement (mm)',ylabel='External force (N)');b.legend();panel(b,'e','Passive force-displacement response')
    c.plot([1,2,3],met['secant_stiffness_n_m'],'o-',color=GREEN,label='Secant stiffness')
    c.plot([1.5,2.5],met['interval_stiffness_n_m'],'s-',color=BLUE,label='Interval stiffness')
    c.set(xticks=[1,2,3],xticklabels=['Low','Middle','High'],ylabel='Stiffness (N/m)');c.legend();panel(c,'f','Original plateau stiffness')
    save(fig,'fig03_p3_stiffness',{'data':str(p),'rows':len(a),'sampling':'complete 0.125 ms record; original scoring plateaus and stiffness retained'})

def phs():
    cases=['s050_positive','s050_negative','s070_positive','s070_negative'];rr=[];full=[]
    for case in cases:
        r=Replay('phs',case)
        with np.load(RAW/'PHS2D_passive_head_stabilization'/case/'states_timestep.npz') as a:
            tr=a['trajectory']; cols=a['columns'].tolist(); print(case,'raw columns',cols[:3],flush=True)
            full.append({'time':tr[:,0].copy(),'tip':a['tip_position_m'].copy(),'point':a['force_point_m'].copy()})
        rr.append(r)
    # Original NPZ convention is time_s, force_x_n, qpos[144], qvel[144], ...
    fig=plt.figure(figsize=(7,7.6),layout='constrained');gs=fig.add_gridspec(3,6,height_ratios=[1.3,1.3,1])
    for j,(r,unloaded,title) in enumerate([(rr[0],True,'Unloaded: 9 segments'),(rr[0],False,'s050 positive load'),(rr[1],False,'s050 negative load')]):
        picture(fig.add_subplot(gs[0,j*2:j*2+2]),render(r,len(r.times)-1,f'phs_full_{j}',unloaded=unloaded),chr(97+j),title)
    for j in range(2):
        r=rr[j];axclose=fig.add_subplot(gs[1,j*3:j*3+3]);picture(axclose,render(r,len(r.times)-1,f'phs_close_{r.case}',close=True),chr(100+j),r.case.replace('_',' '))
        delta=(r.tip[-1]-r.unloaded_tip)*1000
        axclose.text(.03,.02,f'World-space bar: 20 mm\nOrange: unloaded tip\nFinal tip: XY {np.linalg.norm(delta[:2]):.2f} mm, Z {delta[2]:+.2f} mm',transform=axclose.transAxes,fontsize=6,va='bottom',bbox={'facecolor':'white','alpha':.8,'edgecolor':'none'})
    ax=[fig.add_subplot(gs[2,j*2:j*2+2]) for j in range(3)];stats={}
    for r,f,col,style in zip(rr,full,[GREEN,BLUE,GREEN,BLUE],['-','-','--','--']):
        delta=(f['tip']-r.unloaded_tip)*1000;pd=(f['point']-r.unloaded_point)*1000
        vals=[np.linalg.norm(delta[:,:2],axis=1),delta[:,2],np.linalg.norm(pd[:,:2],axis=1)]
        for a,v in zip(ax,vals):line(a,f['time'],v,color=col,linestyle=style,label=r.case.replace('_',' '))
        stats[r.case]={'states':len(f['time']),'tip_xy_peak_mm':float(vals[0].max()),'tip_z_min_mm':float(vals[1].min()),'tip_z_max_mm':float(vals[1].max()),'point_xy_peak_mm':float(vals[2].max())}
    for a,l,title,ylabel in zip(ax,'fgh',['Tip horizontal shift','Tip vertical change','Load-point horizontal shift'],['XY displacement (mm)','Z change (mm)','XY displacement (mm)']):
        panel(a,l,title);a.set(xlabel='Time (s)',ylabel=ylabel);a.legend(fontsize=5)
    save(fig,'fig04_phs2d',{'data':str(RAW/'PHS2D_passive_head_stabilization'),
        'sampling':'32001 original states per case; no 401-row display data used for curves/statistics',
        'full_data_descriptive_extrema':stats,'poses':'exact original-state display indices; no interpolation',
        'interpretation':'XY stabilization only, not 3D fixation; no unloading data'})
    # Supplemental s070 close-ups at the same physical scale convention.
    fig,axs=plt.subplots(1,2,figsize=(7,2.9),layout='constrained')
    for j,r in enumerate(rr[2:]):picture(axs[j],render(r,len(r.times)-1,f'phs_close_{r.case}',close=True),chr(97+j),r.case.replace('_',' ')+'; 20 mm scale')
    folder=OUT/'supplementary';folder.mkdir(exist_ok=True)
    export_axes(fig,folder,'figS01_phs2d_s070',{'sampling':'exact final saved states; no integration'})
    plt.close(fig)

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--only',choices=['m1','eight','p3','phs']);args=ap.parse_args()
    for name,fn in [('m1',m1),('eight',eight),('p3',p3),('phs',phs)]:
        if args.only is None or args.only==name:fn()
    write(OUT/'figure_provenance.json',META)
