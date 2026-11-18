"""Select unchanged dense CSV rows for display after dense acceptance."""
import json
from pathlib import Path
import numpy as np
from scipy.signal import find_peaks
from ..geometry.tm_rho_feedback_frozen import thaw
from . import g1_p01_finalize as f


def select_rows(a, events, max_gap=.01):
    t=a[:,0]
    if len(t)<2 or a.shape[1]!=41 or not np.isfinite(a).all() or np.any(np.diff(t)<=0):
        raise ValueError('INVALID_DENSE_TRAJECTORY')
    events=np.unique(events); right=np.clip(np.searchsorted(t,events),0,len(t)-1); left=np.maximum(right-1,0)
    event_idx=np.where(abs(t[left]-events)<abs(t[right]-events),left,right)
    if len(events) and np.max(abs(t[event_idx]-events))>1e-9:raise ValueError('EVENT_NOT_PRESENT_IN_SOURCE')
    chosen={0,len(t)-1};chosen.update(event_idx.tolist());jerk=a[:,28:37]
    for i in event_idx:
        for j in (i-1,i):
            if 0<=j<len(t)-1 and np.max(abs(jerk[j+1]-jerk[j]))>.05:
                chosen.update((j,j+1))
                if j+2<len(t):chosen.add(j+2)
    changes=np.flatnonzero(np.any(a[1:,39:41]!=a[:-1,39:41],axis=1))+1
    for i in changes:chosen.update((int(i-1),int(i)))
    for start in (10,19,28):
        for k in range(9):chosen.update((int(np.argmin(a[:,start+k])),int(np.argmax(a[:,start+k]))))
    for k in range(9):
        for sign in (1.,-1.):chosen.update(find_peaks(sign*jerk[:,k],prominence=.05)[0].tolist())
    i=0
    while i<len(t)-1:
        j=min(len(t)-1,int(np.searchsorted(t,t[i]+max_gap,side='right')-1))
        # A source gap cannot be filled by selecting original rows. Preserve
        # both of its endpoints and continue rather than rejecting valid data.
        if j<=i:j=i+1
        chosen.add(j);i=j
    ix=np.array(sorted(chosen))
    gaps=np.diff(t[ix]); unavoidable=(np.diff(ix)==1) & (np.diff(t)[ix[:-1]]>max_gap)
    if np.any((gaps>max_gap+1e-12) & ~unavoidable):raise ValueError('DISPLAY_GAP_EXCEEDS_SOURCE')
    return ix


def prepare(folder, geometry):
    folder=Path(folder);geometry=Path(geometry)
    geo=json.loads((geometry/'geometry.json').read_text());law=json.loads((folder/'time_law.json').read_text())
    k=np.asarray(law['nodes']);x=np.asarray(law['lambda_values']);block=thaw(geo)
    old=f.T0
    try:
        f.T0=law['g0_duration_s']
        cuts=np.unique(np.concatenate([f.cuts_for(c,geo['construction']['progress'],k) for c in block.charts]))
        events=f.law(cuts,k,x)[0]
    finally:f.T0=old
    source=folder/'trajectory.csv'
    with source.open('rb') as fp:header=fp.readline();rows=fp.readlines()
    data=np.loadtxt(source,delimiter=',',skiprows=1);ix=select_rows(data,events)
    if len(rows)!=len(data):raise ValueError('ROW_COUNT_MISMATCH')
    temp=folder/'trajectory.display.tmp'
    with temp.open('xb') as fp:
        fp.write(header)
        for i in ix:fp.write(rows[i])
    source_gaps=np.diff(data[:,0])
    return temp,{'method':'dense_original_rows_event_extrema','dense_rows':len(rows),'display_rows':len(ix),'dense_bytes':source.stat().st_size,'display_bytes':temp.stat().st_size,'events':len(events),'max_gap_s':float(np.max(np.diff(data[ix,0]))),'source_gaps_over_0_01_s':int(np.count_nonzero(source_gaps>.01)),'source_max_gap_s':float(np.max(source_gaps)),'acceptance_scope':'original_dense_trajectory; display interpolation is not independently certified'}
