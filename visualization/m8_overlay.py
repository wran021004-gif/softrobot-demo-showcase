"""Native scene-only markers; physical tunnel remains intact."""
import numpy as np
import mujoco
from .presentation_overlay import connector,geom,path
from .palettes import rgba
def markers(scene,case,index,technical):
    md=case.metadata;tun=md['tunnel'];yc,zc=tun['center_yz_m'];w,h=tun['width_m']/2,tun['height_m']/2
    for x,color in [(tun['entry_x_m'],'#d95f02'),(tun['exit_x_m'],'#525252')]:
        vertices=np.array([[x,yc-w,zc-h],[x,yc+w,zc-h],[x,yc+w,zc+h],[x,yc-w,zc+h]])
        for a,b in zip(vertices,np.roll(vertices,-1,axis=0)):connector(scene,a,b,2.5,rgba(color),mujoco.mjtGeom.mjGEOM_LINE)
    goal=case.reference[-1];angles=np.linspace(0,2*np.pi,41);ring=np.column_stack([np.full(len(angles),goal[0]),goal[1]+np.cos(angles)*.005,goal[2]+np.sin(angles)*.005])
    path(scene,ring,rgba('#252525'),width=2.5)
    if technical:
        path(scene,md['cell_centers_m'][index],rgba('#525252'),width=2.)
        path(scene,np.column_stack([np.linspace(tun['entry_x_m'],tun['exit_x_m'],25),np.full(25,yc),np.full(25,zc)]),rgba('#d95f02'),dashed=True,width=2.)
        a,b=md['nearest_fromto_m'][index].reshape(2,3);connector(scene,a,b,3.,rgba('#7570b3'),mujoco.mjtGeom.mjGEOM_LINE)
        for p in [a,b]:geom(scene,mujoco.mjtGeom.mjGEOM_SPHERE,[.0009]*3,p,np.eye(3),rgba('#7570b3'))
        # Outline the closest physical wall independently of color.
        wall=md['geometry_map']['walls'][str(int(md['nearest_wall_geom'][index]))]
        y=yc-w if wall.endswith('left') else yc+w if wall.endswith('right') else None
        z=zc-h if wall.endswith('bottom') else zc+h if wall.endswith('top') else None
        corners=[[tun['entry_x_m'],y if y is not None else yc-w,z if z is not None else zc-h],[tun['exit_x_m'],y if y is not None else yc-w,z if z is not None else zc-h],[tun['exit_x_m'],y if y is not None else yc+w,z if z is not None else zc+h],[tun['entry_x_m'],y if y is not None else yc+w,z if z is not None else zc+h]]
        for a,b in zip(corners,np.roll(corners,-1,axis=0)):connector(scene,a,b,3.5,rgba('#7570b3'),mujoco.mjtGeom.mjGEOM_LINE)
