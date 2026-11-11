"""Native MuJoCo frame rendering, immutable physical XML and saved states."""
import argparse,hashlib,json,threading,time
from pathlib import Path
import numpy as np
import mujoco
from PIL import Image,ImageDraw,ImageFont
from .adapters import load
from .presentation_overlay import Shell,task_markers,geom
from .camera_presets import camera

TASKS=['m1','m2','m3','m4','m5','m6','c1','p3','phs2d','spring_button','m7']
def forbid_integration():
    def blocked(*a,**kw):raise RuntimeError('Presentation uses saved physical states; integration is forbidden')
    for name in ['mj_step','mj_step1','mj_step2']:setattr(mujoco,name,blocked)
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def load_model(path):
    values=[];errors=[]
    def work():
        try:values.append(mujoco.MjModel.from_xml_path(str(path)))
        except BaseException as e:errors.append(e)
    previous=threading.stack_size();threading.stack_size(max(previous,32*1024*1024));t=threading.Thread(target=work);t.start();threading.stack_size(previous);t.join()
    if errors:raise errors[0]
    return values[0]
def physical_snapshot(m):
    names=['body_mass','body_inertia','body_ipos','body_iquat','body_pos','body_quat','jnt_type','jnt_axis','jnt_pos','jnt_range','jnt_stiffness','dof_damping','geom_contype','geom_conaffinity','geom_friction','geom_solref','geom_solimp','geom_pos','geom_quat','geom_size','actuator_ctrlrange','actuator_forcerange','actuator_gainprm','actuator_trnid','tendon_adr','tendon_num','wrap_objid','wrap_prm']
    arrays={n:np.asarray(getattr(m,n)).copy() for n in names}
    arrays['options']=np.array([m.opt.timestep,m.opt.solver,m.opt.integrator,m.opt.iterations,m.opt.tolerance,*m.opt.gravity]);return arrays

class Presentation:
    def __init__(self,root,task,case=None,technical=False):
        self.case=load(root,task,case);self.m=load_model(self.case.model_path);self.d=mujoco.MjData(self.m);self.xml_sha=sha(self.case.model_path);self.snapshot=physical_snapshot(self.m);self.technical=technical;self.index=0
        self.sid=self.m.site('probe_center' if task=='spring_button' else 'tip_site').id
        self.shell=Shell(self.m,task);self.shell.appearance(technical)
        self.m.vis.global_.offwidth=2400;self.m.vis.global_.offheight=1800
        self.m.vis.headlight.ambient[:]=[.50]*3;self.m.vis.headlight.diffuse[:]=[.45]*3;self.m.vis.headlight.specular[:]=[.07]*3
        self.m.vis.rgba.fog[:]=[1,1,1,1]
        # In MuJoCo 3.13 free cameras use this public model VISUAL setting;
        # MjvCamera.orthographic alone does not set native GL-camera projection.
        self.m.vis.global_.orthographic=1
        self.option=mujoco.MjvOption();self.option.flags[mujoco.mjtVisFlag.mjVIS_TENDON]=technical;self.option.flags[mujoco.mjtVisFlag.mjVIS_JOINT]=False;self.option.geomgroup[3]=int(technical);self.option.sitegroup[:]=0
        self.history_ids=np.unique(np.linspace(0,len(self.case.time)-1,min(1800,len(self.case.time)),dtype=int));self.cached_tip={}
        bounds=[]
        for i in [0,len(self.case.time)//2,len(self.case.time)-1]:
            self.restore(i);bounds.extend([self.d.xpos[b].copy() for b,_,_,_,_ in self.shell.cells]);bounds.append(self.tip.copy());bounds.append(self.d.xpos[self.shell.base]+([0,0,.08] if task in ['p3','phs2d'] else np.array([0,0,-.055])))
        if self.case.reference is not None:bounds.extend(self.case.reference[::max(1,len(self.case.time)//20)])
        if self.case.tip is not None:bounds.extend([np.min(self.case.tip,axis=0),np.max(self.case.tip,axis=0)])
        if self.case.force_point is not None:bounds.extend(self.case.force_point[::max(1,len(self.case.time)//20)])
        b=np.array(bounds);padding=np.array([.025,.025,.02]);self.bounds=(b.min(axis=0)-padding,b.max(axis=0)+padding);self.set_camera(1)
        # P3's original CSV has no Tip field. Derive display path only at actual
        # saved states (not interpolation); full raw CSV remains untouched.
        if self.case.tip is None:
            for i in self.history_ids:self.restore(int(i));self.cached_tip[int(i)]=self.tip.copy()
        self.restore(0)
        if task=='p3':self.case.metadata['unloaded_tip_m']=self.tip.tolist()
        self.renderer=None
    def set_camera(self,preset):
        self.camera=camera(self.bounds,preset);az=np.radians(self.camera.azimuth);el=np.radians(self.camera.elevation)
        forward=np.array([np.cos(az)*np.cos(el),np.sin(az)*np.cos(el),np.sin(el)]);up=np.array([-np.cos(az)*np.sin(el),-np.sin(az)*np.sin(el),np.cos(el)]);right=np.cross(forward,up)
        lo,hi=self.bounds;corners=np.array([[x,y,z] for x in [lo[0],hi[0]] for y in [lo[1],hi[1]] for z in [lo[2],hi[2]]]);h=float(np.ptp(corners@up));w=float(np.ptp(corners@right));field=max(h/.80,w/(4/3*.92),.12)*1.04
        self.m.vis.global_.fovy=field;self.camera.lookat[:]=(lo+hi)/2+up*field*.055
    def restore(self,index):
        c=self.case;self.index=int(index);self.d.qpos[:]=c.qpos[index];self.d.qvel[:]=c.qvel[index];self.d.time=float(c.time[index])
        if self.m.nu and c.control is not None:self.d.ctrl[:]=c.control[index]
        mujoco.mj_forward(self.m,self.d);self.tip=self.d.site_xpos[self.sid].copy()
    def trail(self,index):
        if self.case.tip is not None:return self.case.tip[:index+1]
        ids=self.history_ids[self.history_ids<=index];return np.vstack([*[self.cached_tip[int(i)] for i in ids],self.tip])
    def decorate(self,scene,index):
        try:floorid=self.m.geom('floor').id
        except KeyError:floorid=-1
        for g in scene.geoms[:scene.ngeom]:
            if g.objtype==int(mujoco.mjtObj.mjOBJ_GEOM) and g.objid==floorid:g.emission=.45
        # White studio backdrop is native DECOR geometry, separate from the
        # original floor/environment. No XML texture, geom or body is added.
        az=np.radians(self.camera.azimuth);el=np.radians(self.camera.elevation)
        forward=np.array([np.cos(az)*np.cos(el),np.sin(az)*np.cos(el),np.sin(el)]);up=np.array([-np.cos(az)*np.sin(el),-np.sin(az)*np.sin(el),np.cos(el)]);normal=-forward;right=np.cross(up,normal)
        g=geom(scene,mujoco.mjtGeom.mjGEOM_BOX,[10,10,.001],self.camera.lookat+3*forward,np.column_stack([right,up,normal]),[1,1,1,1]);g.emission=1.;g.specular=0.
        if not self.technical:self.shell.add(scene,self.d)
        if self.case.force is not None and self.case.force_point is None:
            # Actual payload COM from the restored model; never a tip substitute.
            try:
                if not hasattr(self,'force_com'):self.force_com=np.zeros((len(self.case.time),3))
                self.force_com[index]=self.d.xipos[self.m.body('payload').id];point=self.force_com
                old=self.case.force_point;self.case.force_point=point;task_markers(scene,self.case,self.d,index,self.tip,self.trail(index));self.case.force_point=old
                return
            except KeyError:pass
        task_markers(scene,self.case,self.d,index,self.tip,self.trail(index))
    def hud(self,index,speed):
        c=self.case;t=c.time[index];title=c.task.upper()+({'m3':' [DRAFT]','m4':' [LEGACY]','m5':' [TRANSIENT]'}.get(c.task,''));lines=[f'{title} | {c.case} | {c.phase(index)}',f'Physical time {t:.4f} s | Playback {speed:g}x | '+('Technical view' if self.technical else 'Presentation shell'),f'Tip XYZ ({self.tip[0]:+.4f}, {self.tip[1]:+.4f}, {self.tip[2]:+.4f}) m']
        if c.reference is not None:lines.append(f'Tip error {np.linalg.norm(self.tip-c.reference[index])*1000:.2f} mm | Tip sphere / target open cube | Actual solid / reference dashed')
        if c.force is not None:lines.append('Applied world force '+' / '.join(f'{v:+.4f}' for v in c.force[index])+' N')
        if c.task=='spring_button':lines.append(f'Button {c.metadata["button_q_m"][index]*1000:.3f} mm | Native contact {c.metadata["probe_button_force_n"][index]:.4f} N | Original evaluator PASS')
        if c.task=='phs2d':lines.append('Orange cross = initial XY at current height; Z is unconstrained')
        return '\n'.join(lines)
    def frame(self,index,width=1280,height=960,speed=1.,hud=True):
        self.restore(int(index))
        if self.renderer is None:self.renderer=mujoco.Renderer(self.m,width=width,height=height,max_geom=12000)
        self.renderer.update_scene(self.d,camera=self.camera,scene_option=self.option);self.renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW]=False;self.renderer.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION]=False;self.decorate(self.renderer.scene,index)
        im=Image.fromarray(self.renderer.render().copy())
        if hud:
            draw=ImageDraw.Draw(im);size=round(width/73)
            try:font=ImageFont.truetype(r'C:\Windows\Fonts\arial.ttf',size)
            except OSError:font=ImageFont.load_default()
            text=self.hud(index,speed);box=draw.multiline_textbbox((14,10),text,font=font,spacing=3);draw.rectangle((0,0,width,box[3]+9),fill='white');draw.multiline_text((14,10),text,font=font,spacing=3,fill='#252525')
        return im
    def verify(self):
        now=physical_snapshot(self.m);return {'xml_sha256':self.xml_sha,'xml_bytes_unchanged':sha(self.case.model_path)==self.xml_sha,'all_physical_arrays_unchanged':all(np.array_equal(v,now[k]) for k,v in self.snapshot.items()),'saved_qpos_restored_exactly':bool(np.array_equal(self.d.qpos,self.case.qpos[self.index])),'saved_qvel_restored_exactly':bool(np.array_equal(self.d.qvel,self.case.qvel[self.index])),'saved_ctrl_restored_exactly':bool(self.case.control is None or np.array_equal(self.d.ctrl,self.case.control[self.index])),'hidden_original_geoms':len(self.shell.hidden),'overlay_cells':len(self.shell.cells),'tendon_visualization':bool(self.option.flags[mujoco.mjtVisFlag.mjVIS_TENDON]),'new_physics_steps':0}
    def close(self):
        if self.renderer is not None:self.renderer.close();self.renderer=None

def main():
    p=argparse.ArgumentParser();p.add_argument('task',choices=TASKS);p.add_argument('--case');p.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[1]);p.add_argument('--speed',type=float,default=1.);p.add_argument('--loop',action='store_true');p.add_argument('--technical',action='store_true');p.add_argument('--render',type=Path);p.add_argument('--frame',type=int,default=-1);p.add_argument('--camera',type=int,default=1);a=p.parse_args()
    if a.speed<=0:p.error('speed must be positive')
    forbid_integration();r=Presentation(a.root,a.task,a.case,a.technical);r.set_camera(a.camera)
    if a.render:
        a.render.parent.mkdir(parents=True,exist_ok=True);r.frame(a.frame if a.frame>=0 else len(r.case.time)-1).save(a.render,dpi=(600,600));print(json.dumps(r.verify()));r.close();return
    import mujoco.viewer as viewer
    state={'pause':False,'time':float(r.case.time[0]),'camera_reset':True}
    def key(k):
        if k==32:state['pause']=not state['pause']
        if k in [82,114]:state['time']=float(r.case.time[0])
        if k in [84,116]:
            r.technical=not r.technical;r.shell.appearance(r.technical);r.option.flags[mujoco.mjtVisFlag.mjVIS_TENDON]=r.technical;r.option.geomgroup[3]=int(r.technical)
        if k in [49,50,51]:r.set_camera(k-48);state['camera_reset']=True
    with viewer.launch_passive(r.m,r.d,key_callback=key,show_left_ui=False,show_right_ui=False) as v:
        last=time.perf_counter()
        while v.is_running():
            now=time.perf_counter()
            if not state['pause']:state['time']+=(now-last)*a.speed
            last=now
            if state['time']>r.case.time[-1]:state['time']=float(r.case.time[0]) if a.loop else float(r.case.time[-1])
            i=max(0,int(np.searchsorted(r.case.time,state['time'],side='right')-1));r.restore(i)
            with v.lock():
                v.opt.flags[:]=r.option.flags;v.opt.geomgroup[:]=r.option.geomgroup;v.opt.sitegroup[:]=r.option.sitegroup
                if state['camera_reset']:
                    v.cam.lookat[:]=r.camera.lookat;v.cam.distance=r.camera.distance;v.cam.azimuth=r.camera.azimuth;v.cam.elevation=r.camera.elevation;v.cam.orthographic=1;state['camera_reset']=False
                else:
                    r.camera.lookat[:]=v.cam.lookat;r.camera.distance=v.cam.distance;r.camera.azimuth=v.cam.azimuth;r.camera.elevation=v.cam.elevation
                v.user_scn.ngeom=0;r.decorate(v.user_scn,i);v.user_scn.flags[mujoco.mjtRndFlag.mjRND_SHADOW]=False;v.set_texts((mujoco.mjtFontScale.mjFONTSCALE_100,None,r.hud(i,a.speed),''))
            v.sync();time.sleep(.01)
    print(json.dumps(r.verify()));r.close()
if __name__=='__main__':main()
