"""Task-specific readers; original CSV/NPZ formats remain unchanged."""
from .base import SavedCase
def load(root,task,case=None):
    if task in ['m1','m2','m3','m4','m5','m6']:
        from .m_series import load_case
    elif task=='c1':from .c1 import load_case
    elif task=='p3':from .p3 import load_case
    elif task=='phs2d':from .phs2d import load_case
    elif task=='spring_button':from .spring_button import load_case
    elif task=='m7':from .m7 import load_case
    else:raise ValueError('Unknown task: '+task)
    return load_case(root,task,case)
