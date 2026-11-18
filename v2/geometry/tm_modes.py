"""V2-TM mode identification; retain raw parent indices and roundoff evidence."""
from enum import Enum
import numpy as np
from .unified_chart import nine_dimensional_progress
from ..core.contracts import require

class Mode(str, Enum):
    MOVE = 'MOVE'
    ADJUST = 'ADJUST'
    HOLD = 'HOLD'

def classify(path, dwell_duration=None):
    if dwell_duration is not None and float(dwell_duration)>0:
        raise ValueError('HOLD_NOT_IMPLEMENTED')
    q,u,source=nine_dimensional_progress(path.q)
    eps=4*np.spacing(max(1.,float(np.max(np.abs(q[:,:2])))))
    modes=[]; i=0
    while i<len(q)-1:
        anchor=q[i,:2]; j=i
        while j+1<len(q) and np.max(np.abs(q[j+1,:2]-anchor))<=eps:
            j+=1
        if j>i:
            require(np.any(q[j,2:] != q[i,2:]) or np.any(np.diff(q[i:j+1,2:],axis=0)),
                    'ADJUST_NO_MOTION','fixed XY segment has no adjustment')
            kind=Mode.ADJUST
        else:
            kind=Mode.MOVE; j=i+1
            while j<len(q)-1:
                k=j
                while k+1<len(q) and np.max(np.abs(q[k+1,:2]-q[j,:2]))<=eps:
                    k+=1
                if k>j: break
                j+=1
        p=q[-1,:2] if kind==Mode.ADJUST and j==len(q)-1 else q[i,:2]
        modes.append(dict(kind=kind.value,start=int(i),end=int(j),source_start=int(source[i]),
                          source_end=int(source[j]),u0=float(u[i]),u1=float(u[j]),
                          anchor=p.tolist() if kind==Mode.ADJUST else None))
        i=j
    return q,u,source,modes,float(eps)
