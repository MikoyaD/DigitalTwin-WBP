"""Recreate historical fixed-geometry G1 export rows from its analytic time law.

Only use after the stored geometry and time law were checked against the dense
export before it was compacted.  This is for numerical/collision consumers;
visualization continues to use the compact CSV.
"""
import json
from pathlib import Path
import numpy as np
from ..geometry.tm_rho_feedback_frozen import thaw
from . import g1_p01_finalize as f


def recover(folder, geometry):
    folder=Path(folder); geo=json.loads((Path(geometry)/'geometry.json').read_text())
    law=json.loads((folder/'time_law.json').read_text());block=thaw(geo)
    k=np.asarray(law['nodes']);x=np.asarray(law['lambda_values']);old=f.T0
    try:
        f.T0=float(law['g0_duration_s']);records=[]
        for ci,c in enumerate(block.charts):
            cuts=f.cuts_for(c,geo['construction']['progress'],k)
            ss=np.unique(np.concatenate([np.linspace(lo,hi,129) for lo,hi in zip(cuts[:-1],cuts[1:])]))
            t,u,q,v,a,j=f.eval_chart(c,ss,k,x)
            records.append((ss,t,u,q,v,a,j,np.full(len(ss),ci,int)))
        s,t,u,q,v,a,j,ci=[np.concatenate([r[z] for r in records]) for z in range(8)]
        order=np.argsort(t,kind='stable');s,t,u,q,v,a,j,ci=[z[order] for z in (s,t,u,q,v,a,j,ci)]
        _,reverse=np.unique(t[::-1],return_index=True);ix=np.sort(len(t)-1-reverse)
        s,t,u,q,v,a,j,ci=[z[ix] for z in (s,t,u,q,v,a,j,ci)]
        mode=np.array([next(i for i,m in enumerate(geo['construction']['modes'])
             if m['u0']-1e-14 <= (block.charts[int(c)].s0+block.charts[int(c)].s1)/2 <= m['u1']+1e-14) for c in ci])
        note=folder/'trajectory.csv.storage.json'
        if note.is_file():
            saved=json.loads(note.read_text()).get('dense_mode_runs')
            if saved is not None:
                rebuilt=np.empty(len(mode),dtype=mode.dtype);covered=0
                for start,end,value in saved:
                    if start!=covered or end<=start or end>len(mode):raise ValueError('DENSE_MODE_RUNS_INVALID')
                    rebuilt[start:end]=value;covered=end
                if covered!=len(mode):raise ValueError('DENSE_MODE_RUNS_INCOMPLETE')
                mode=rebuilt
        return np.column_stack([t,q,v,a,j,u,s,ci,mode])
    finally:
        f.T0=old


def read_numeric_rows(folder, geometry):
    folder=Path(folder);note=folder/'trajectory.csv.storage.json'
    if note.is_file():
        metadata=json.loads(note.read_text())
        if metadata.get('dense_reconstruction')=='g1_fixed_geometry_v1':
            if Path(metadata['geometry']).resolve()!=Path(geometry).resolve():
                raise ValueError('DENSE_RECOVERY_GEOMETRY_MISMATCH')
            rows=recover(folder,geometry)
            if len(rows)!=metadata['source_rows']:raise ValueError('DENSE_RECOVERY_ROW_COUNT_MISMATCH')
            compact=np.loadtxt(str(folder/'trajectory.csv'),delimiter=',',skiprows=1)
            ix=np.searchsorted(rows[:,0],compact[:,0])
            if np.any(ix>=len(rows)) or not np.array_equal(rows[ix],compact):
                raise ValueError('DENSE_RECOVERY_NO_LONGER_MATCHES_STORED_ROWS')
            return rows
        raise ValueError('DENSE_TRAJECTORY_UNAVAILABLE: sampled display CSV is not a collision input')
    return np.loadtxt(str(folder/'trajectory.csv'),delimiter=',',skiprows=1)
