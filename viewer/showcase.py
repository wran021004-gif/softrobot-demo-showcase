"""Unified saved-state viewer. Space pause, R restart, L loop, +/- speed, 1/2/3 camera.
Adapted from the accepted trajectory, P3/C1 replay and PHS saved-state scripts.
Rendering and mj_forward only: this player never calls mj_step.
"""
import argparse, json, time
from pathlib import Path
import numpy as np
import mujoco
from saved_states import FOLDERS, CASES, load_model, load_active, load_phs, centerline, material_point, forbid_integration
ROOT=Path(__file__).resolve().parents[1]
ORANGE=[1,.48,.08,1];GREEN=[.02,.75,.35,1];RED=[.93,.15,.12,1]
TARGETS={'positive_high':([.285,.035,.185],[.8,.2,.7,1]),'negative_low':([.275,-.04,.135],[.2,.4,1,1]),'central_high':([.295,0,.215],[1,.66,.05,1])}
def read(p):return json.loads(p.read_text(encoding='utf8'))
def csv_read(p):
    with p.open(encoding='utf8') as f: columns=f.readline().strip().split(',')
    return columns,np.loadtxt(p,delimiter=',',skiprows=1,ndmin=2)
class Replay:
    def __init__(self,demo,case=None):
        self.demo=demo;self.case=case;self.force=None;self.stages=None;self.binding=None
        self.axis=None;self.reference_axis=None;self.vertices=np.empty((0,3));self.holding_window=None;self.control=None
        if demo in FOLDERS and demo!='phs':load_active(self,demo,case)
        elif demo=='phs':load_phs(self,case)
        elif demo=='m1':
            self.case=case or 'central_high'
            if self.case not in TARGETS:raise ValueError('M1 case must be central_high, positive_high or negative_low')
            folder=ROOT/'M1_reach_hold';mapping=read(folder/'joint_map.json');cols,a=csv_read(folder/self.case/'trajectory_timestep.csv');ix={c:i for i,c in enumerate(cols)}
            self.qpos=a[:,[ix[j['qpos_column']] for j in mapping['joints']]];self.qvel=a[:,[ix[j['qvel_column']] for j in mapping['joints']]]
            self.tip=a[:,[ix['tip_position_m_'+c] for c in 'xyz']];self.times=a[:,ix['time_s']]
            self.reference=np.tile(TARGETS[self.case][0],(len(a),1));self.model_path=folder/'scene.xml';self.base_path=folder/'robot.xml'
            self.tension=a[:,[ix[c] for c in mapping['actual_tension_columns']]]
            self.title='M1 / '+self.case+' / independent experiment'
        elif demo=='eight':
            folder=ROOT/'figure_eight_tracking';a=np.load(folder/'trajectory_full.npz')
            for attr,key in [('qpos','qpos'),('qvel','qvel'),('tip','tip_m'),('times','time_s'),('reference','reference_m'),('tension','tension_n')]:setattr(self,attr,a[key])
            self.model_path=folder/'scene.xml';self.base_path=folder/'robot.xml';self.title='Figure-eight / four independent tension channels'
        else:
            self.case=case or 'plus_x'
            if self.case not in ['plus_x','minus_x']:raise ValueError('P3 case must be plus_x or minus_x')
            folder=ROOT/'P3_progressive_stiffness';summary=read(folder/'summary.json');self.summary=summary
            a=np.load(folder/self.case/'display_replay.npz');cols=a['columns'].tolist();a=a['rows'];ix={c:i for i,c in enumerate(cols)}
            self.qpos=a[:,[ix[f'qpos_{i}'] for i in range(6)]];self.qvel=a[:,[ix[f'qvel_{i}'] for i in range(6)]];self.times=a[:,ix['time_s']]
            self.point=a[:,[ix[f'point_{c}_m'] for c in 'xyz']];self.force=a[:,[ix[f'force_{c}_n'] for c in 'xyz']];self.stages=a[:,ix['stage']].astype(int)
            self.tension=a[:,[ix[f'tension_n_{i}'] for i in range(2)]];self.binding=summary['force_point_binding']
            self.model_path=folder/self.case/'scene.xml';self.base_path=folder/self.case/'robot.xml';self.title='P3 / '+self.case+' / passive fixed payout'
            self.reference=np.empty((0,3));self.tip=None
        self.m=load_model(self.model_path,deep=demo=='phs');self.d=mujoco.MjData(self.m);self.tip_id=self.m.site('tip_site').id
        if demo=='phs':
            self.d.qpos[:]=self.unloaded_qpos;self.d.qvel[:]=self.unloaded_qvel;mujoco.mj_forward(self.m,self.d)
            self.unloaded_centerline=centerline(self.m,self.d,self.geometry)
            self.unloaded_point=material_point(self.m,self.d,self.geometry,self.binding)
        self.index=0;self.restore(0)
        self.figure=mujoco.MjvFigure();mujoco.mjv_defaultFigure(self.figure)
        f=self.figure;f.flg_legend=True;f.flg_extend=False;f.flg_ticklabel[0]=True;f.flg_ticklabel[1]=True;f.figurergba[:]=[.95,.97,.99,.96];f.panergba[:]=[.95,.97,.99,1];f.textrgb[:]=[.15,.2,.27];f.gridrgb[:]=[.8,.84,.88]
        self.aux_figure=mujoco.MjvFigure();mujoco.mjv_defaultFigure(self.aux_figure)
        for field in ['figurergba','panergba','textrgb','gridrgb']:getattr(self.aux_figure,field)[:]=getattr(f,field)
        self.aux_figure.flg_legend=True;self.aux_figure.flg_extend=False
        self.camera=mujoco.MjvCamera();mujoco.mjv_defaultCamera(self.camera);self.set_camera(1)
    def restore(self,i):
        self.index=i;self.d.qpos[:]=self.qpos[i];self.d.qvel[:]=self.qvel[i];self.d.time=self.times[i]
        if self.m.nu:self.d.ctrl[:]=self.control[i] if self.control is not None else self.tension[i]
        mujoco.mj_forward(self.m,self.d)
        if self.demo=='phs':self.current_centerline=centerline(self.m,self.d,self.geometry)
        return self.d
    def set_camera(self,preset,camera=None):
        c=camera or self.camera
        c.lookat[:]=[.12,0,.17] if self.demo!='p3' else [-.02,0,.27]
        c.distance=.65 if self.demo!='p3' else .85;c.azimuth=120 if self.demo!='p3' else 90;c.elevation=-18
        if self.demo=='m5':c.lookat[:]=[.12,0,.12];c.distance=.72
        if self.demo=='phs':c.lookat[:]=[-.02,0,.28];c.distance=.83;c.azimuth=115;c.elevation=-12
        if preset==2:
            c.lookat[:]=[.23,0,.17] if self.demo!='phs' else [0,0,.29];c.azimuth=0 if self.demo!='phs' else 90;c.elevation=0;c.distance=.42 if self.demo!='phs' else .49
        if preset==3:c.azimuth=90;c.elevation=0
    def labels(self,i):
        t=self.times[i];tip=self.d.site_xpos[self.tip_id];xyz=lambda p:'({:+.3f}, {:+.3f}, {:+.3f}) m'.format(*p)
        lines=[self.title,f'Time {t:.3f} / {self.times[-1]:.3f} s','Tip '+xyz(tip)]
        if self.demo=='p3':
            lines += ['Stage: '+self.summary['stage_names'][self.stages[i]],f'External force {np.linalg.norm(self.force[i]):.3f} N at s=0.70',f'Point displacement {(self.point[i,0]-self.point[0,0])*1000:+.2f} mm','Visual loading marker / direct force, no contact model','100 Hz display samples; original timestamps']
        elif self.demo=='phs':
            delta=self.tip_delta[i]*1000;fp=np.linalg.norm((self.point[i]-self.unloaded_point)[:2])*1000
            lines += [f'Tip dX {delta[0]:+.3f} / dY {delta[1]:+.4f} / dZ {delta[2]:+.3f} mm',f'Tip horizontal shift {np.linalg.norm(delta[:2]):.3f} mm; load-point XY {fp:.2f} mm',f'Force X {self.force[i,0]:+.4f} N at material s={self.entry["s"]:.2f}','UNLOADED BASELINE' if self.case=='equilibrium' else 'LOAD RAMP / HOLD LOAD; no unloading recorded','4 passive tendons / 0 active channels; 9 x 8 cells','Orange unloaded / green deformed centerline','100 Hz original-state samples; no interpolation',self.notes,self.origin_note]
        else:
            lines += ['Target '+xyz(self.reference[i]),f'Position error {np.linalg.norm(self.tip[i]-self.reference[i])*1000:.2f} mm']
            if self.demo=='m1':lines+=['HOLD (last 60 ms)' if t>=.54-1e-9 else 'REACH','Three separate initial states; choose case on command line']
            elif self.demo=='eight':
                t0=min(max(t,0),22);s=min(t0/2,1);e=max(0,min((t0-20)/2,1));integ=lambda x:2.5*x**4-3*x**5+x**6
                cycles=.1*(2*integ(s) if t0<2 else t0-1 if t0<20 else 19+2*(e-integ(e)))
                lines += [f'Cycles {cycles:.2f} / 2.00; YZ figure-eight','Prepared equilibrium; smooth phase start and stop']
            else:
                lines += [f'Tip speed {np.linalg.norm(self.velocity[i])*1000:.3f} mm/s']
                if self.holding_window:
                    a,b=self.holding_window;lines += [f'{"HOLD WINDOW" if a-1e-9<=t<=b+1e-9 else "REACH"}: {a:.2f}-{b:.2f} s only']
                if self.reference_axis is not None:
                    lines += [f'Tool +X angle error {self.angle_error[i]:.3f} deg','Green actual axis / orange target axis; no roll constraint']
                if self.demo=='m2':
                    force=self.force[i];label='ZERO FORCE COMPARISON' if self.case=='zero' else ('APPLIED LOAD' if np.linalg.norm(force)>0 else 'PRE-LOAD')
                    lines += [label+' / configured load window 0.10-0.35 s',f'Payload COM F = ({force[0]:+.3f}, {force[1]:+.3f}, {force[2]:+.3f}) N',f'Force magnitude {np.linalg.norm(force):.5f} N / real payload 10 g']
                    if self.case=='positive':lines += ['Hold speed margin to 20 mm/s: only 0.000051 mm/s']
                if self.demo=='c1':
                    lines += ['4 Independent Actuation Channels','T0 purple / T1 teal / T2 gold / T3 blue', ' / '.join(f'T{k} {v:.3f} N' for k,v in enumerate(self.tension[i]))]
                    if self.case=='M1_B':lines += ['Dense hold peak 30.396 mm/s > 30 mm/s']
                    if self.case=='M5':lines += ['Endpoint speed 84.781 mm/s; no stationary-aim claim']
                lines += [self.notes,self.origin_note]
        if self.demo not in FOLDERS:lines += ['Codex-assisted existing result / not Main autonomous Study']
        return '\n'.join(lines)
    def decorate(self,scene,i):
        def sphere(pos,r,color,label=None):
            if scene.ngeom>=scene.maxgeom:return
            g=scene.geoms[scene.ngeom];scene.ngeom+=1;mujoco.mjv_initGeom(g,mujoco.mjtGeom.mjGEOM_SPHERE,np.array([r]*3),np.asarray(pos),np.eye(3).ravel(),np.asarray(color,dtype=np.float32))
            if label:g.label=label
        def connector(a,b,width,color,kind=mujoco.mjtGeom.mjGEOM_LINE):
            if scene.ngeom>=scene.maxgeom or np.linalg.norm(np.asarray(a)-b)<1e-10:return
            g=scene.geoms[scene.ngeom];scene.ngeom+=1;mujoco.mjv_initGeom(g,kind,np.zeros(3),np.zeros(3),np.eye(3).ravel(),np.asarray(color,dtype=np.float32));mujoco.mjv_connector(g,kind,width,np.asarray(a),np.asarray(b))
        def path(p,color,width=2):
            ids=np.unique(np.r_[np.linspace(0,len(p)-1,min(450,len(p)),dtype=int),len(p)-1]) if len(p) else []
            for a,b in zip(ids[:-1],ids[1:]):connector(p[a],p[b],width,color)
        if self.demo=='m1':
            for name,(pos,color) in TARGETS.items():sphere(pos,.004,color)
        if self.demo=='phs':
            path(self.unloaded_centerline,ORANGE,3);path(self.current_centerline,GREEN,3);path(self.tip[:i+1],GREEN,2)
            sphere(self.tip[i],.004,GREEN);sphere(self.point[i],.005,[1,.72,.08,1])
            f=np.linalg.norm(self.force[i])
            if f>1e-12:connector(self.point[i],self.point[i]+self.force[i]/f*(.035+.25*f),.0025,RED,mujoco.mjtGeom.mjGEOM_ARROW)
        elif self.demo!='p3':
            path(self.reference,ORANGE);path(self.tip[:i+1],GREEN,3)
            sphere(self.reference[i],.0035,ORANGE);sphere(self.tip[i],.003,GREEN)
            for k,point in enumerate(self.vertices):sphere(point,.004,[[.75,.2,.75,1],[.2,.45,1,1],[1,.7,.05,1]][k%3])
            if self.reference_axis is not None:
                connector(self.tip[i],self.tip[i]+.052*self.axis[i],.0022,GREEN,mujoco.mjtGeom.mjGEOM_ARROW)
                connector(self.reference[i],self.reference[i]+.052*self.reference_axis[i],.0022,ORANGE,mujoco.mjtGeom.mjGEOM_ARROW)
            if self.demo=='m2':
                com=self.d.xipos[self.m.body('payload').id];sphere(com,.010,[1,.72,.08,.5]);f=np.linalg.norm(self.force[i])
                if f>1e-12:connector(com,com+self.force[i]/f*(.03+.9*f),.0025,RED,mujoco.mjtGeom.mjGEOM_ARROW)
            if self.holding_window and self.times[i]>=self.holding_window[0]:
                path(self.tip[(self.times>=self.holding_window[0])&(self.times<=self.times[i])],[.02,.45,.25,1],4)
            if self.demo=='m1' and self.times[i]>=.54:
                path(self.tip[(self.times>=.54)&(self.times<=self.times[i])],[.02,.45,.25,1],4)
        else:
            p=self.point[i];force=self.force[i];f=np.linalg.norm(force);direction=force/f if f>1e-9 else np.array([1 if self.case=='plus_x' else -1,0,0])
            sphere(p,.014,[1,.75,.05,.8]);connector(p-direction*.035,p-direction*.007,.006,[.48,.5,.54,1],mujoco.mjtGeom.mjGEOM_CYLINDER)
            if f>1e-9:connector(p-direction*(.025+.25*f),p,.003,RED,mujoco.mjtGeom.mjGEOM_ARROW)
            for j in range(self.m.ntendon):self.m.tendon_rgba[j]=[.85,.18,.12,1] if self.tension[i,j]>.005 else [.35,.35,.4,1]
    def update_figure(self,i):
        f=self.figure
        def line(n,x,y,color,name):
            ids=np.linspace(0,len(x)-1,min(450,len(x)),dtype=int) if len(x) else []
            f.linepnt[n]=len(ids);f.linedata[n,:2*len(ids)]=np.column_stack([np.asarray(x)[ids],np.asarray(y)[ids]]).ravel();f.linergb[n]=color;f.linename[n]=name
        if self.demo=='p3':
            sign=1 if self.case=='plus_x' else -1;x=sign*(self.point[:,0]-self.point[0,0])*1000;y=sign*self.force[:,0]
            f.title='Force - displacement / display samples';f.xlabel='Displacement (mm)';f.range[:]=[[-6,23],[0,.23]]
            line(0,x,y,[.72,.76,.8],'full record');line(1,x[:i+1],y[:i+1],[.9,.15,.1],'played');line(2,[x[i]-.35,x[i]+.35],[y[i],y[i]],[.1,.5,.9],'current')
        elif self.demo=='phs':
            delta=self.tip_delta*1000;limit=max(1.,float(np.max(abs(delta[:,:2])))*1.2)
            f.title='Magnified Tip horizontal XY (mm)';f.xlabel='dX (mm)';f.range[:]=[[-limit,limit],[-limit,limit]]
            line(0,[0,0],[-.08,.08],[.9,.5,.08],'unloaded');line(1,delta[:i+1,0],delta[:i+1,1],[.02,.7,.3],'Tip XY')
            line(2,[delta[i,0]-.06,delta[i,0]+.06],[delta[i,1],delta[i,1]],[.2,.35,.9],'current')
        elif self.demo=='eight':
            f.title='YZ trace / orange reference, green actual';f.xlabel='Y (mm)';f.range[:]=[[-25,25],[158,182]]
            line(0,self.reference[:,1]*1000,self.reference[:,2]*1000,[1,.48,.08],'reference');line(1,self.tip[:i+1,1]*1000,self.tip[:i+1,2]*1000,[.02,.7,.3],'actual')
        else:
            e=np.linalg.norm(self.tip-self.reference,axis=1)*1000;f.title='Position error (mm)'+(' / hold: 0.54-0.60 s' if self.demo=='m1' else '');f.xlabel='Time (s)';f.range[:]=[[0,float(self.times[-1])],[0,max(.1,float(e.max()*1.1))]]
            line(0,self.times,e,[.73,.76,.8],'full record');line(1,self.times[:i+1],e[:i+1],[.02,.7,.3],'played')
        return f

    def update_aux_figure(self,i):
        if self.demo!='phs' and self.reference_axis is None:return None
        f=self.aux_figure;x=self.times
        y=self.tip_delta[:,2]*1000 if self.demo=='phs' else self.angle_error
        f.title='Tip vertical dZ (mm) / not fixed in 3D' if self.demo=='phs' else 'Tool +X angle error (deg) / no roll'
        f.xlabel='Original time (s)';f.range[:]=[[0,float(x[-1])],[min(0,float(y.min()*1.1)),max(.1,float(y.max()*1.1))]]
        for k,end,color,label in [(0,len(x),[.72,.76,.8],'full record'),(1,i+1,[.1,.55,.85],'played')]:
            ids=np.linspace(0,end-1,min(450,end),dtype=int);f.linepnt[k]=len(ids);f.linedata[k,:2*len(ids)]=np.column_stack([x[ids],y[ids]]).ravel();f.linergb[k]=color;f.linename[k]=label
        return f

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('demo',choices=['m1','eight','p3',*FOLDERS]);p.add_argument('--case');p.add_argument('--speed',type=float,default=1);p.add_argument('--loop',action='store_true');p.add_argument('--technical',action='store_true');p.add_argument('--smoke-seconds',type=float);p.add_argument('--render',type=Path);p.add_argument('--list-cases',action='store_true')
    a=p.parse_args()
    if a.speed<=0:p.error('speed must be positive')
    if a.list_cases:
        print(' / '.join(CASES.get(a.demo,['default'])));return
    r=Replay(a.demo,a.case)
    if a.render:
        render_preview(r,a.render);return
    from mujoco import viewer
    state=dict(paused=False,restart=False,loop=a.loop,speed=a.speed,camera=None)
    def key(k):
        if k==32:state['paused']=not state['paused']
        elif k in [82,114]:state['restart']=True
        elif k in [76,108]:state['loop']=not state['loop']
        elif k in [61,43,334]:state['speed']=min(8,state['speed']*2)
        elif k in [45,333]:state['speed']=max(.0625,state['speed']/2)
        elif k in [49,50,51]:state['camera']=k-48
    cursor=float(r.times[0]);previous=time.perf_counter();started=previous;frames=0
    with viewer.launch_passive(r.m,r.d,key_callback=key,show_left_ui=False,show_right_ui=False) as v:
        r.set_camera(1,v.cam);v.opt.geomgroup[3]=not a.technical
        while v.is_running():
            now=time.perf_counter();elapsed=now-previous;previous=now
            if a.smoke_seconds and now-started>=a.smoke_seconds:break
            if state['restart']:cursor=float(r.times[0]);state['restart']=False
            if not state['paused']:cursor+=elapsed*state['speed']
            # A 2 s final frame dwell makes short M1 hold visible without changing saved time.
            if state['loop'] and cursor>r.times[-1]+2*state['speed']:cursor=float(r.times[0])
            i=int(np.clip(np.searchsorted(r.times,cursor,side='right')-1,0,len(r.times)-1))
            with v.lock():
                r.restore(i)
                if state['camera']:r.set_camera(state['camera'],v.cam);state['camera']=None
                v.user_scn.ngeom=0;r.decorate(v.user_scn,i)
                fig=r.update_figure(i);vp=v.viewport;width=max(260,int(vp.width*.31));height=max(170,int(vp.height*.30))
                panels=[(mujoco.MjrRect(max(0,vp.width-width-12),12,width,height),fig)]
                aux=r.update_aux_figure(i)
                if aux is not None:panels.append((mujoco.MjrRect(max(0,vp.width-width-12),height+24,width,max(150,int(vp.height*.23))),aux))
                v.set_figures(panels)
                v.set_texts((mujoco.mjtFontScale.mjFONTSCALE_100,None,r.labels(i)+'\n'+f'{state["speed"]:g}x  '+('PAUSED' if state['paused'] else 'END / saved final pose' if cursor>=r.times[-1] else 'PLAY')+'\nSpace pause | R restart | L loop | +/- speed | 1/2/3 camera',''))
            v.sync();frames+=1;time.sleep(.012)
    print(json.dumps(dict(demo=a.demo,case=r.case,frames=frames,physics_steps=0)))

def render_preview(r,dest):
    if r.demo in FOLDERS:return render_remaining_preview(r,dest)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    if r.demo=='m1':ids=[0,len(r.times)//2,len(r.times)-1]
    elif r.demo=='p3':ids=[0,int(np.flatnonzero(r.stages==3)[len(np.flatnonzero(r.stages==3))//2]),len(r.times)-1]
    else:ids=[int(np.argmin(abs(r.times-t))) for t in [4.5,7,9.5]]
    fig,axes=plt.subplots(2,3,figsize=(15,8),gridspec_kw={'height_ratios':[1.4,1]});fig.patch.set_facecolor('#eff4f8')
    opt=mujoco.MjvOption();opt.geomgroup[3]=True
    with mujoco.Renderer(r.m,height=540,width=800) as renderer:
        for ax,i in zip(axes[0],ids):
            r.restore(i);renderer.update_scene(r.d,camera=r.camera,scene_option=opt);r.decorate(renderer.scene,i);ax.imshow(renderer.render());ax.axis('off');ax.set_title(f't = {r.times[i]:.3f} s'+(' / '+r.summary['stage_names'][r.stages[i]] if r.demo=='p3' else ''),fontsize=11)
    a,b,c=axes[1]
    if r.demo=='p3':
        sign=1 if r.case=='plus_x' else -1;x=sign*(r.point[:,0]-r.point[0,0])*1000;force=sign*r.force[:,0]
        a.plot(x,force,color='#dc3931');a.set(xlabel='Displacement at s=0.70 (mm)',ylabel='External force (N)',title='Original loading and unloading')
        b.plot(r.times,x,color='#10a765');b.set(xlabel='Original time (s)',ylabel='Displacement (mm)',title='100 Hz display samples')
        c.axis('off');c.text(.02,.95,'3 segments / 2 passive tendons\nFixed payout: 143.17 mm each\nForce applied directly at s=0.70\nLoading probe is a visual marker\nNo contact dynamics claimed\nOriginal evaluator: PASS\nStiffness ratio: 3.70\nResidual: 0.000382 mm',va='top',fontsize=11)
    else:
        a.plot(r.reference[:,1]*1000,r.reference[:,2]*1000,color='#ff8014',label='reference');a.plot(r.tip[:,1]*1000,r.tip[:,2]*1000,color='#10a765',label='actual');a.set(xlabel='Y (mm)',ylabel='Z (mm)',title='YZ tip trace');a.legend(fontsize=9);a.set_aspect('equal',adjustable='datalim')
        e=np.linalg.norm(r.tip-r.reference,axis=1)*1000;b.plot(r.times,e,color='#10a765');b.set(xlabel='Time (s)',ylabel='Position error (mm)',title='All saved timesteps')
        c.axis('off')
        if r.demo=='eight':
            v=read(ROOT/'figure_eight_tracking'/'results.json')['metrics'];text=f'1 segment / 4 actuators / 2 g pointer\n270 mm long, 16 mm circular section\nYZ amplitude: 20 x 8 mm; 0.10 Hz\n2 full cycles; smooth start and stop\nRMSE: {v["tracking_rmse_m"]*1000:.2f} mm\nMax: {v["max_error_m"]*1000:.2f} mm\nP95: {v["p95_error_m"]*1000:.2f} mm\nNo tension, stroke or speed saturation'
        else:text='Original accepted M1 recording\n2 segments / 6 ideal tension channels\n'+r.case+' ONLY\nIndependent initial state for each target\nHold window: 0.54-0.60 s\nPointer: massless visual overlay\nOriginal 10 g payload retained'
        c.text(.02,.95,text,va='top',fontsize=11)
    for ax in [a,b]:ax.grid(alpha=.25)
    fig.suptitle(r.title,fontsize=15,fontweight='bold');fig.tight_layout(rect=[0,0,1,.95]);dest.parent.mkdir(parents=True,exist_ok=True);fig.savefig(dest,dpi=130);plt.close(fig)
    print(dest)
def render_remaining_preview(r,dest):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    ids=[0,len(r.times)//2,len(r.times)-1]
    fig,axes=plt.subplots(2,3,figsize=(16,9),gridspec_kw={'height_ratios':[1.4,1]});fig.patch.set_facecolor('#eff4f8')
    opt=mujoco.MjvOption();opt.geomgroup[3]=True
    with mujoco.Renderer(r.m,height=580,width=860) as renderer:
        for ax,i in zip(axes[0],ids):
            r.restore(i);renderer.update_scene(r.d,camera=r.camera,scene_option=opt);r.decorate(renderer.scene,i);ax.imshow(renderer.render());ax.axis('off');ax.set_title(f't = {r.times[i]:.3f} s',fontsize=11)
    a,b,c=axes[1];c.axis('off')
    if r.demo=='phs':
        delta=r.tip_delta*1000;fp=np.linalg.norm((r.point-r.unloaded_point)[:,:2],axis=1)*1000
        a.plot(delta[:,0],delta[:,1],color='#10a765');a.scatter(0,0,color='#ff8014',label='unloaded');a.scatter(*delta[-1,:2],color='#265dca',label='loaded tip');a.set(xlabel='Tip dX (mm)',ylabel='Tip dY (mm)',title='Magnified horizontal Tip motion');a.set_aspect('equal',adjustable='datalim');a.legend(fontsize=9)
        b.plot(r.times,fp,color='#e34b40',label='load point XY');b.plot(r.times,np.linalg.norm(delta[:,:2],axis=1),color='#10a765',label='Tip XY');b.plot(r.times,delta[:,2],color='#2861ba',label='Tip Z');b.set(xlabel='Original time (s)',ylabel='Displacement (mm)',title='Mid-arm moves; horizontal Tip stays close');b.legend(fontsize=9)
        note=f'9 segments x 8 cells / 4 passive tendons\nLoad at material s={r.entry["s"]:.2f}; F={r.force[-1,0]:+.3f} N\nTip final XY: {np.linalg.norm(delta[-1,:2]):.3f} mm\nTip final Z: {delta[-1,2]:+.3f} mm\n100 Hz exact original-state samples\nOrange unloaded / green deformed\nNo unloading or 3D fixation claim'
    elif r.reference_axis is not None:
        a.plot(r.times,r.position_error*1000,color='#10a765');a.set(xlabel='Time (s)',ylabel='Position error (mm)',title='Saved-state position error')
        b.plot(r.times,r.angle_error,color='#2861ba');b.set(xlabel='Time (s)',ylabel='Tool +X angle error (deg)',title='Axis direction only; roll unconstrained')
        note=f'{r.m.nu} independent ideal tension channels\nOriginal payload: {r.m.body_mass[r.m.body("payload").id]*1000:.0f} g\nEnd speed: {np.linalg.norm(r.velocity[-1])*1000:.3f} mm/s\nEnd position error: {r.position_error[-1]*1000:.3f} mm\nEnd axis error: {r.angle_error[-1]:.3f} deg\nGreen actual / orange reference axis\nTransient motion; no static-aim claim'
    elif r.demo=='m2':
        a.plot(r.times,r.position_error*1000,color='#10a765');a.axvspan(*r.holding_window,color='#afd8c0',alpha=.35);a.set(xlabel='Time (s)',ylabel='Position error (mm)',title='Hold window: 0.30-0.35 s')
        for k,(color,name) in enumerate([('#e34b40','Fx'),('#bf7936','Fy'),('#2861ba','Fz')]):b.plot(r.times,r.force[:,k],color=color,label=name)
        b.set(xlabel='Time (s)',ylabel='World force at payload COM (N)',title='Direct COM force; record ends at 0.35 s');b.legend()
        note=f'Independent {r.case} case / real 10 g payload\nForce starts at 0.10 s\nNo unloading / recovery record\nTarget / actual tip / green trail\nGold marker: actual payload COM\nRed arrow: saved world force\nPositive speed margin is very small'
    else:
        a.plot(r.reference[:,1]*1000,r.reference[:,2]*1000,color='#ff8014',label='reference');a.plot(r.tip[:,1]*1000,r.tip[:,2]*1000,color='#10a765',label='actual');a.scatter(r.reference[-1,1]*1000,r.reference[-1,2]*1000,color='#ff8014');a.set(xlabel='Y (mm)',ylabel='Z (mm)',title='YZ tip trace');a.legend(fontsize=9)
        b.plot(r.times,r.position_error*1000,color='#10a765')
        if r.holding_window:b.axvspan(*r.holding_window,color='#afd8c0',alpha=.4)
        b.set(xlabel='Time (s)',ylabel='Position error (mm)',title='Saved-state error / shaded hold window')
        note=f'{r.m.nu} independent actuation channels\nOriginal recording: {r.times[-1]:.3f} s\nEnd speed: {np.linalg.norm(r.velocity[-1])*1000:.3f} mm/s\nEnd error: {r.position_error[-1]*1000:.3f} mm\n'
        if r.demo=='m3':note+='Development draft / last 50 ms only'
        elif r.demo=='m4':note+='Legacy polyline / dense peak 12.095 mm\nNo precision or stable-stop claim'
        else:note+='Four independent initial-state experiments\nM1_B dense hold speed: 30.396 mm/s\nNo global actuator-minimum claim'
    c.text(.015,.98,note+'\n\nCodex-assisted existing result\nNot Main autonomous Study',va='top',fontsize=10.5)
    for ax in [a,b]:ax.grid(alpha=.25)
    fig.suptitle(r.title,fontsize=15,fontweight='bold');fig.tight_layout(rect=[0,0,1,.95]);dest.parent.mkdir(parents=True,exist_ok=True);fig.savefig(dest,dpi=125);plt.close(fig);print(dest)

if __name__=='__main__':
    forbid_integration();main()
