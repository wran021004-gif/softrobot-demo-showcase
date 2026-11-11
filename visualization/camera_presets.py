import numpy as np
import mujoco
def camera(bounds,preset=1):
    lo,hi=bounds;extent=max(float(np.max(hi-lo)),.12);c=mujoco.MjvCamera();mujoco.mjv_defaultCamera(c);c.orthographic=1
    c.lookat[:]=(lo+hi)/2;c.distance=extent*1.8;c.azimuth=125;c.elevation=-20
    if preset==2:c.azimuth=90;c.elevation=-5
    if preset==3:c.azimuth=0;c.elevation=0
    return c
