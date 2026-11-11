"""Technical/Presentation saved-state viewer. No physics integration."""
import argparse,time
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
from mechanics import HERE,read,load_model,rgba,COLORS,LABEL

def phase(t):return 'Approach' if t<.5 else 'Press' if t<.9 else 'Hold' if t<1.4 else 'Release' if t<1.8 else 'Settle'
def forbid_steps():
    def stop(*args,**kwargs):raise RuntimeError('Saved-state replay cannot integrate physics')
    for name in ['mj_step','mj_step1','mj_step2']:setattr(mujoco,name,stop)

class Replay:
    def __init__(self,view='technical'):
        folder=HERE/'candidates/FINAL_C009';self.a=np.load(folder/'trajectory.npz');tree=ET.parse(folder/'environment.xml').getroot()
        ET.SubElement(tree.find('asset'),'texture',name='white_sky',type='skybox',builtin='flat',rgb1='1 1 1',rgb2='1 1 1',width='128',height='128')
        # The old non-colliding display post obscures the lower base exit.
        # Reroute that display stand behind the routing plane only in replay;
        # fixed-base coordinates and every physical/contact geom stay intact.
        post=tree.find('.//geom[@name="visual_mount"]');post.set('pos','-.03 0 .075');post.set('size','.009 .045')
        old_target=tree.find('.//site[@name="task_target"]')
        if old_target is not None:old_target.set('rgba','0 0 0 0')
        ET.SubElement(tree.find('worldbody'),'geom',name='display_support_elbow',type='capsule',fromto='-.03 0 .12 0 0 .15',size='.006',rgba='.5 .55 .6 1',contype='0',conaffinity='0',density='0')
        self.m=load_model(xml=ET.tostring(tree,encoding='unicode'));self.d=mujoco.MjData(self.m);self.info=read(folder/'mechanics.json')
        # Replace inherited debug-green markers in this display model only.
        self.m.site_rgba[self.m.site('tip_site').id]=[0,0,0,0]
        self.m.site_rgba[self.m.site('probe_center').id]=[.2,.2,.2,1]
        self.m.site_size[self.m.site('probe_center').id,0]=.0003
        self.view=view;self.camera=mujoco.MjvCamera();mujoco.mjv_defaultCamera(self.camera)
        self.option=mujoco.MjvOption();self.option.geomgroup[3]=1
        self.m.vis.global_.offwidth=2400;self.m.vis.global_.offheight=1800
        self.m.vis.headlight.ambient[:]=[.6,.6,.6];self.m.vis.headlight.diffuse[:]=[.65,.65,.65]
        self.original_rgba=self.m.geom_rgba.copy();self.set_view(view);self.camera_view('overall')
    def set_view(self,view):
        self.view=view;self.m.geom_rgba[:]=self.original_rgba
        for i in range(self.m.ngeom):
            n=self.m.geom(i).name
            if n.startswith('segment_'):self.m.geom_rgba[i]=[.64,.79,.82,1]
            elif n in ['tool_block','tool_bracket','probe']:self.m.geom_rgba[i]=rgba('#d95f02')
            elif n in ['button_cap','button_rod']:self.m.geom_rgba[i]=rgba('#7570b3')
            elif n=='floor':self.m.geom_rgba[i]=[.96,.97,.97,1]
            elif n.startswith('guide_') and ('_eye' in n or '_spoke' in n) and view=='presentation':self.m.geom_rgba[i]=[.48,.54,.59,1]
        # All four tendons remain enabled and colored in both display modes.
        for i in range(4):self.m.tendon_rgba[i]=rgba(COLORS[i]);self.m.tendon_width[i]=.0005
    def restore(self,i):
        self.d.qpos[:]=self.a['qpos'][i];self.d.qvel[:]=self.a['qvel'][i];self.d.ctrl[:]=self.a['control_n'][i];self.d.time=self.a['time_s'][i]
        mujoco.mj_forward(self.m,self.d)
    def camera_view(self,kind):
        self.camera.azimuth=140 if self.view=='technical' else 115;self.camera.elevation=-20
        if kind=='overall':self.camera.lookat[:]=[.15,.01,.115];self.camera.distance=.59
        elif kind=='contact':self.camera.lookat[:]=[.2955,.030,.15];self.camera.distance=.115;self.camera.azimuth=120
        else:
            geoms=[g for g in self.info['guides'] if g['group']==kind]
            self.camera.lookat[:]=np.mean([self.d.site_xpos[self.m.site(g['name']+'_hole_center').id] for g in geoms],axis=0)
            self.camera.distance=.145;self.camera.azimuth=150;self.camera.elevation=-15
    def decorate(self,scene,i):
        def point(pos,r,color,label=''):
            g=scene.geoms[scene.ngeom];scene.ngeom+=1;mujoco.mjv_initGeom(g,mujoco.mjtGeom.mjGEOM_SPHERE,np.array([r]*3),pos,np.eye(3).ravel(),np.asarray(color,np.float32));g.label=label
        cp=self.a['contact_position_m'][i]
        if np.isfinite(cp).all():point(cp,.0007,[.05,.05,.05,1])
        if self.view=='technical':
            for g in self.info['guides']:
                if g['group'] in ['base','junction'] and g['routing_site']:
                    point(self.d.site_xpos[self.m.site(g['name']+'_hole_center').id],.0001,rgba(COLORS[int(g['tendon'][1:])]),g['tendon'])
    def image(self,i,kind='overall',view='technical',width=2400,height=1800):
        self.restore(i);self.set_view(view);self.camera_view(kind)
        with mujoco.Renderer(self.m,width=width,height=height) as ren:
            ren.update_scene(self.d,camera=self.camera,scene_option=self.option);self.decorate(ren.scene,i);return ren.render().copy()
    def text(self,i):
        a=self.a;t=a['time_s'][i]
        return f'v1.2 C009 / PASS | {self.view}\n{t:.3f} s | {phase(t)}\nButton {a["button_q_m"][i]*1000:.3f} mm\nContact force {a["contact_force_n"][i]:.4f} N\nXZ error {a["lateral_error_m"][i]*1000:.3f} mm\n20 open guide eyes; all four tendons shown\nT0/T1 anchor at junction; T2/T3 reach distal anchor\n{LABEL}\nSpace pause | R restart | T/P view | 1 overall | 2 contact | 3 base | 4 junction'

def main():
    p=argparse.ArgumentParser();p.add_argument('--view',choices=['technical','presentation'],default='technical');p.add_argument('--close',action='store_true');p.add_argument('--loop',action='store_true');args=p.parse_args();forbid_steps();r=Replay(args.view);r.restore(0);r.camera_view('contact' if args.close else 'overall')
    state={'pause':False,'t':0.,'camera':'contact' if args.close else 'overall'}
    def key(k):
        if k==32:state['pause']=not state['pause']
        if k in [82,114]:state['t']=0.
        if k in [84,116]:r.set_view('technical')
        if k in [80,112]:r.set_view('presentation')
        if k in [49,50,51,52]:state['camera']={49:'overall',50:'contact',51:'base',52:'junction'}[k]
    import mujoco.viewer as viewer
    with viewer.launch_passive(r.m,r.d,key_callback=key,show_left_ui=False,show_right_ui=False) as v:
        last=time.perf_counter()
        while v.is_running():
            now=time.perf_counter()
            if not state['pause']:state['t']+=now-last
            last=now
            if state['t']>2:state['t']=state['t']%2 if args.loop else 2.
            i=min(2000,round(state['t']/.001));r.restore(i);r.camera_view(state['camera'])
            with v.lock():
                v.opt.geomgroup[3]=1
                v.cam.lookat[:]=r.camera.lookat;v.cam.distance=r.camera.distance;v.cam.azimuth=r.camera.azimuth;v.cam.elevation=r.camera.elevation
                v.user_scn.ngeom=0;r.decorate(v.user_scn,i);v.set_texts((mujoco.mjtFontScale.mjFONTSCALE_100,None,r.text(i),''))
            v.sync();time.sleep(.01)
if __name__=='__main__':main()
