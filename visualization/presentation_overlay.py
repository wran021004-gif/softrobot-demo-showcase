"""Pure MjvScene geometry, never model bodies/geoms or saved state changes."""
import re
import numpy as np
import mujoco
from .palettes import BUGN,DARK2,rgba

def geom(scene,kind,size,pos,rotation,color):
    if scene.ngeom>=scene.maxgeom:raise RuntimeError('Native scene capacity exceeded')
    g=scene.geoms[scene.ngeom];scene.ngeom+=1
    mujoco.mjv_initGeom(g,kind,np.asarray(size,float),np.asarray(pos,float),np.asarray(rotation,float).ravel(),np.asarray(color,np.float32))
    g.category=int(mujoco.mjtCatBit.mjCAT_DECOR);g.objid=-1;g.segid=-1;g.specular=.12;g.shininess=.25
    return g
def connector(scene,a,b,width,color,kind=mujoco.mjtGeom.mjGEOM_CAPSULE):
    if np.linalg.norm(np.asarray(a)-b)<1e-10:return
    g=geom(scene,kind,[0,0,0],[0,0,0],np.eye(3),color);mujoco.mjv_connector(g,kind,width,np.asarray(a,float),np.asarray(b,float));return g
def path(scene,points,color,dashed=False,width=2.):
    if len(points)<2:return
    ids=np.unique(np.linspace(0,len(points)-1,min(350,len(points)),dtype=int))
    for j,(a,b) in enumerate(zip(ids[:-1],ids[1:])):
        if not dashed or j%3!=2:connector(scene,points[a],points[b],width,color,mujoco.mjtGeom.mjGEOM_LINE)

class Shell:
    def __init__(self,m,task):
        self.m=m;self.task=task;self.cells=[];self.connectors=[]
        for b in range(1,m.nbody):
            name=m.body(b).name
            if not re.fullmatch(r'segment_\d+_cell_\d+',name):continue
            shapes=[g for g in range(m.body_geomadr[b],m.body_geomadr[b]+m.body_geomnum[b]) if m.geom(g).name.endswith('_shape')]
            if not shapes:continue
            g=shapes[0];length=2*float(m.geom_pos[g,0]);sizes=m.geom_size[g];r=float(np.max(np.delete(sizes,np.argmin(abs(sizes-length/2)))))*1.18
            seg,cell=map(int,re.findall(r'\d+',name));self.cells.append((b,length,r,seg,cell))
        self.geoms_original=m.geom_rgba.copy();self.materials_original=m.geom_matid.copy();self.sites_original=m.site_rgba.copy()
        self.hidden=[]
        for g in range(m.ngeom):
            name=m.geom(g).name;body=m.body(m.geom_bodyid[g]).name
            if name.endswith('_shape') and re.fullmatch(r'segment_\d+_cell_\d+',body) or body=='fixed_base' or body.startswith('guide_') or body.startswith('connector_') or body in ['mid_guide','end_guide'] or name.startswith('visual_'):
                self.hidden.append(g)
            if re.fullmatch(r'(connector_\d+|guide_\d+|mid_guide|end_guide)',body):
                self.connectors.append((int(m.geom_bodyid[g]),m.geom_pos[g].copy(),m.geom_size[g].copy()))
        self.base=m.body('fixed_base').id
    def appearance(self,technical):
        m=self.m;m.geom_rgba[:]=self.geoms_original;m.geom_matid[:]=self.materials_original;m.site_rgba[:]=self.sites_original
        # Sensor/route sites are replaced by visible task markers in the scene.
        m.site_rgba[:,3]=0
        if not technical:
            for g in self.hidden:m.geom_matid[g]=-1;m.geom_rgba[g,3]=0
        for g in range(m.ngeom):
            name=m.geom(g).name
            if self.task=='m8' and name.startswith('tunnel_'):m.geom_rgba[g]=rgba('#737373',1. if technical else .16)
            if name=='floor':m.geom_rgba[g]=rgba('#F0F0F0');m.geom_matid[g]=-1
            if name in ['tool_block','tool_bracket','probe'] or m.body(m.geom_bodyid[g]).name=='payload':m.geom_rgba[g]=rgba(DARK2[1])
            if name in ['button_cap','button_rod']:m.geom_rgba[g]=rgba(DARK2[2])
            if m.body(m.geom_bodyid[g]).name=='button_housing':m.geom_rgba[g]=rgba('#525252')
    def add(self,scene,d):
        if self.task=='m8':
            # Reuse the exact native collision mesh as a conformal display shell.
            # No radial or axial inflation can conceal or invent wall clearance.
            for b,L,r,seg,cell in self.cells:
                for gi in range(self.m.body_geomadr[b],self.m.body_geomadr[b]+self.m.body_geomnum[b]):
                    g=geom(scene,self.m.geom_type[gi],self.m.geom_size[gi],d.geom_xpos[gi],d.geom_xmat[gi].reshape(3,3),rgba(BUGN[1 if seg==0 else 2]));g.dataid=int(self.m.geom_dataid[gi])
            # All connectors stay within the corresponding physical boxes.
            for b,pos,size in self.connectors:
                if self.m.body(b).name.startswith('guide_'):continue
                geom(scene,mujoco.mjtGeom.mjGEOM_BOX,size*.98,d.xpos[b]+d.xmat[b].reshape(3,3)@pos,d.xmat[b].reshape(3,3),rgba('#737373'))
            p=d.xpos[self.base];R=d.xmat[self.base].reshape(3,3)
            geom(scene,mujoco.mjtGeom.mjGEOM_BOX,[.014,.022,.022],p-R[:,0]*.016,R,rgba('#737373'))
            return
        for b,L,r,seg,cell in self.cells:
            R=d.xmat[b].reshape(3,3);origin=d.xpos[b];center=origin+R[:,0]*L/2
            last=not any(s==seg and c>cell for _,_,_,s,c in self.cells)
            half=.51*L if self.task=='p3' else (.5*L if last else 1.15*L)
            color=BUGN[seg%3] if self.task in ['p3','phs2d'] else BUGN[1 if seg%2==0 else 2]
            geom(scene,mujoco.mjtGeom.mjGEOM_ELLIPSOID,[half,r,r],center,R,rgba(color))
            # A sleeve follows the real hinge frame, never a smoothed centerline.
            if self.task!='p3':geom(scene,mujoco.mjtGeom.mjGEOM_ELLIPSOID,[min(L*.35,r*.65),r*.86,r*.86],origin,R,rgba(BUGN[seg%3] if self.task=='phs2d' else BUGN[1]))
            if cell==0 and seg>0:
                connector(scene,origin-R[:,0]*.0015,origin+R[:,0]*.0015,r*1.03,rgba('#737373'),mujoco.mjtGeom.mjGEOM_CYLINDER)
        p=d.xpos[self.base];R=d.xmat[self.base].reshape(3,3)
        for b,pos,size in self.connectors:
            RR=d.xmat[b].reshape(3,3);center=d.xpos[b]+RR@pos;radius=.005 if self.task=='phs2d' else .009 if self.task=='p3' else .011
            connector(scene,center-RR[:,0]*max(float(size[0]),.002),center+RR[:,0]*max(float(size[0]),.002),radius,rgba('#737373'),mujoco.mjtGeom.mjGEOM_CYLINDER)
        connector(scene,p-R[:,0]*.024,p-R[:,0]*.002,.022 if self.task!='phs2d' else .013,rgba('#737373'),mujoco.mjtGeom.mjGEOM_CYLINDER)
        # Compact display stand and mounting foot; no physical support claim.
        vertical=self.task in ['p3','phs2d'];bottom=p-R[:,0]*.065 if vertical else p+np.array([0,0,-.045]);connector(scene,bottom,p-R[:,0]*.018,.009,rgba('#969696'))
        geom(scene,mujoco.mjtGeom.mjGEOM_BOX,[.025,.025,.005],bottom+([0,0,.005] if vertical else np.array([0,0,-.005])),np.eye(3),rgba('#737373'))
        # Short rounded transition at the actual terminal section, with no tool
        # length/position change or invented end-effector geometry.
        if self.cells:
            b,L,r,_,_=self.cells[-1];R=d.xmat[b].reshape(3,3);end=d.xpos[b]+R[:,0]*L
            geom(scene,mujoco.mjtGeom.mjGEOM_ELLIPSOID,[min(.003,L*.3),r*.87,r*.87],end-R[:,0]*min(.002,L*.2),R,rgba(BUGN[1]))

def task_markers(scene,case,d,index,tip,trail):
    path(scene,trail,rgba(DARK2[0]),width=2.5)
    geom(scene,mujoco.mjtGeom.mjGEOM_SPHERE,[.0006 if case.task=='spring_button' else .0023]*3,tip,np.eye(3),rgba(DARK2[2]))
    ref=case.reference
    if case.task=='m8':ref=None  # Native open target ring is added by m8_overlay.
    if ref is None and 'unloaded_tip_m' in case.metadata:ref=np.asarray([case.metadata['unloaded_tip_m']])
    if ref is not None:
        target=ref[min(index,len(ref)-1)]
        if np.ptp(ref,axis=0).max()>1e-6:path(scene,ref,rgba(DARK2[1]),dashed=True,width=2.)
        # Target/current reference is an open cube; the actual tip is a sphere.
        r=.0038
        for axis in range(3):
            others=[j for j in range(3) if j!=axis]
            for a in [-1,1]:
                for b in [-1,1]:
                    x=target.copy();y=target.copy();x[axis]-=r;y[axis]+=r;x[others]+=[a*r,b*r];y[others]+=[a*r,b*r];connector(scene,x,y,1.5,rgba(DARK2[1]),mujoco.mjtGeom.mjGEOM_LINE)
    if case.task=='phs2d':
        point=tip.copy();point[:2]=case.tip[0,:2]
        for axis in [0,1]:
            lo=point.copy();hi=point.copy();lo[axis]-=.004;hi[axis]+=.004;connector(scene,lo,hi,2,rgba(DARK2[1]),mujoco.mjtGeom.mjGEOM_LINE)
    if case.force is not None:
        f=case.force[index];size=np.linalg.norm(f)
        if size>1e-12:
            if case.force_point is not None:point=case.force_point[index]
            else:point=tip
            geom(scene,mujoco.mjtGeom.mjGEOM_SPHERE,[.0028]*3,point,np.eye(3),rgba(DARK2[1]))
            connector(scene,point,point+f/size*min(.08,.025+size*.9),.0025,rgba(DARK2[2]),mujoco.mjtGeom.mjGEOM_ARROW)
    if case.actual_axis is not None and case.reference_axis is not None:
        connector(scene,tip,tip+.028*case.actual_axis[index],.0013,rgba(DARK2[0]),mujoco.mjtGeom.mjGEOM_ARROW)
        connector(scene,case.reference[index],case.reference[index]+.028*case.reference_axis[index],.0013,rgba(DARK2[1]),mujoco.mjtGeom.mjGEOM_ARROW)
