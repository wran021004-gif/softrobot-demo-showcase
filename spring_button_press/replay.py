"""Saved-state B5 failure demonstrator. Rendering only; never integrates."""
import argparse,time
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
from environment import HERE,load_model,LABEL

COLORS=['#1b9e77','#d95f02','#7570b3','#555555']
def rgb(s):return [int(s[i:i+2],16)/255 for i in [1,3,5]]
def phase(t):
    return 'Approach' if t<.5 else 'Press' if t<.9 else 'Hold' if t<1.4 else 'Release' if t<1.8 else 'Settle'

class Replay:
    def __init__(self):
        self.a=np.load(HERE/'results/final_replay/trajectory.npz')
        tree=ET.parse(HERE/'results/final_environment.xml').getroot()
        asset=tree.find('asset')
        ET.SubElement(asset,'texture',name='paper_sky',type='skybox',builtin='flat',rgb1='1 1 1',rgb2='1 1 1',width='128',height='128')
        self.m=load_model(xml=ET.tostring(tree,encoding='unicode'));self.d=mujoco.MjData(self.m)
        for i in range(self.m.ngeom):
            name=self.m.geom(i).name
            if name.startswith('segment_'):self.m.geom_rgba[i,:3]=rgb('#66c2a4')
            elif name in ['tool_block','tool_bracket','probe']:self.m.geom_rgba[i,:3]=rgb('#d95f02')
            elif name in ['button_cap','button_rod']:self.m.geom_rgba[i,:3]=rgb('#7570b3')
            elif name=='floor':self.m.geom_rgba[i]=[.96,.97,.97,1]
        for i,c in enumerate(COLORS):self.m.tendon_rgba[i]=[*rgb(c),1];self.m.tendon_width[i]=.0006
        self.m.vis.headlight.ambient[:]=[.6,.6,.6];self.m.vis.headlight.diffuse[:]=[.65,.65,.65]
        self.m.vis.global_.offwidth=2400;self.m.vis.global_.offheight=1800
        self.camera=mujoco.MjvCamera();mujoco.mjv_defaultCamera(self.camera);self.camera_view(False)
    def camera_view(self,close):
        self.camera.azimuth=115;self.camera.elevation=-22
        self.camera.lookat[:]=[.29,.030,.15] if close else [.145,.01,.12]
        self.camera.distance=.11 if close else .59
    def restore(self,i):
        self.d.qpos[:]=self.a['qpos'][i];self.d.qvel[:]=self.a['qvel'][i]
        self.d.ctrl[:]=self.a['control_n'][i];self.d.time=self.a['time_s'][i]
        mujoco.mj_forward(self.m,self.d)
    def decorate(self,scene,i):
        p=self.a['contact_position_m'][i]
        if np.isfinite(p).all():
            g=scene.geoms[scene.ngeom];scene.ngeom+=1
            mujoco.mjv_initGeom(g,mujoco.mjtGeom.mjGEOM_SPHERE,np.array([.0008]*3),p,np.eye(3).ravel(),np.array([.05,.05,.05,1],np.float32))
    def labels(self,i):
        a=self.a;t=a['time_s'][i]
        return f'B5 / FAIL: incomplete feasibility\n{t:.3f} s / {phase(t)}\nButton {a["button_q_m"][i]*1000:.3f} mm\nContact normal force {a["contact_force_n"][i]:.4f} N\nXZ center error {a["lateral_error_m"][i]*1000:.3f} mm\n{LABEL}\nSpace pause | R restart | 1 overall | 2 close-up'
    def image(self,i,close=False,width=2400,height=1800):
        self.restore(i);self.camera_view(close)
        with mujoco.Renderer(self.m,height=height,width=width) as ren:
            ren.update_scene(self.d,camera=self.camera);self.decorate(ren.scene,i)
            return ren.render().copy()

def forbid_steps():
    def fail(*args,**kwargs):raise RuntimeError('Replay is saved-state rendering only')
    for n in ['mj_step','mj_step1','mj_step2']:setattr(mujoco,n,fail)

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--close',action='store_true');ap.add_argument('--loop',action='store_true');args=ap.parse_args()
    forbid_steps();r=Replay();r.camera_view(args.close)
    state={'paused':False,'cursor':0.}
    def key(k):
        if k==32:state['paused']=not state['paused']
        if k in [ord('R'),ord('r')]:state['cursor']=0.
        if k==ord('1'):r.camera_view(False)
        if k==ord('2'):r.camera_view(True)
    import mujoco.viewer as viewer
    with viewer.launch_passive(r.m,r.d,key_callback=key,show_left_ui=False,show_right_ui=False) as v:
        last=time.perf_counter()
        while v.is_running():
            now=time.perf_counter()
            if not state['paused']:state['cursor']+=now-last
            last=now
            if state['cursor']>2:state['cursor']=state['cursor']%2 if args.loop else 2.
            i=min(2000,round(state['cursor']/.001));r.restore(i)
            with v.lock():
                v.cam.lookat[:]=r.camera.lookat;v.cam.distance=r.camera.distance;v.cam.azimuth=r.camera.azimuth;v.cam.elevation=r.camera.elevation
                v.user_scn.ngeom=0;r.decorate(v.user_scn,i)
                v.set_texts((mujoco.mjtFontScale.mjFONTSCALE_100,None,r.labels(i),''))
            v.sync();time.sleep(.01)
if __name__=='__main__':main()
