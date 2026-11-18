"""Recorded edge classification, bounded candidates, no invented translations."""
import time
import numpy as np
from ..core.contracts import *
from ..core.math_tools import spin_jet,arm_only_jet,hold_jet
from .spline_builder import fit,candidates
from ..timing.feasible_profile import parameterize,concatenate


def wrap(x): return np.arctan2(np.sin(x),np.cos(x))


def build_geometry(path,limits,hold_duration=None):
    tic=time.perf_counter();q=path.q
    require(np.max(np.abs(q[0]-path.requested_start))<1e-6 and np.max(np.abs(q[-1]-path.requested_goal))<1e-6,
            'INPUT_ENDPOINT_MISMATCH','path must match requested endpoints; no target substitution')
    # Remove only exact full-state duplicates. Explicit HOLD remains a separate request.
    q=q[np.r_[True,np.any(np.diff(q,axis=0)!=0,axis=1)]]
    phases=[];decisions=[];current=q[0].copy()
    def append(kind,ev,provenance):
        nonlocal current
        ends=ev(np.array([0.,1.])).q
        phase=PhasePath('phase_%03d'%len(phases),kind,path.schema,ends[0],ends[1],ev,provenance)
        phases.append(phase);current=ends[-1].copy()
    def spin(target):
        delta=wrap(target-current[2])
        if abs(delta)>1e-9:
            start=current.copy()
            append(PhaseType.SPIN,lambda s,start=start,delta=delta:spin_jet(s,start,delta),'shortest spin; fixed arm; no collision claim')
    if len(q)==1:
        start=current.copy();append(PhaseType.HOLD,lambda s:hold_jet(start,len(s)),'explicit HOLD duration')
    else:
        kinds=[]
        for a,b in zip(q[:-1],q[1:]):
            xy=np.linalg.norm(b[:2]-a[:2]);arm=np.max(np.abs(b[3:]-a[3:]));theta=abs(wrap(b[2]-a[2]))
            if xy>1e-10:kinds.append(PhaseType.TRANSLATE)
            elif arm>1e-10:
                kinds.append('SPIN_THEN_ARM' if theta>=1e-9 else PhaseType.ARM_ONLY)
            elif theta>1e-9:
                require(abs(b[2]-a[2])<np.pi,'AMBIGUOUS_WINDING','explicit spin step must be smaller than pi')
                kinds.append(PhaseType.SPIN)
            else:raise ContractError('AMBIGUOUS_WINDING','theta-only full revolution requires explicit winding metadata')
        lo=0
        while lo<len(kinds):
            hi=lo+1
            while hi<len(kinds) and kinds[hi]==kinds[lo]: hi+=1
            part=q[lo:hi+1].copy();kind=kinds[lo]
            if kind==PhaseType.SPIN:
                # Preserve each explicit orientation waypoint, not just net rotation.
                for row in part[1:]:spin(row[2])
            elif kind=='SPIN_THEN_ARM':
                # Explicit deterministic candidate: stop, turn at fixed arm, then
                # traverse every distinct arm waypoint at fixed final heading.
                # It changes intermediate coordination, so only scene validation
                # can promote it to an accepted robot trajectory.
                orientations=part[:,2]
                require(np.all(np.abs(np.diff(orientations))<np.pi),
                        'AMBIGUOUS_WINDING','stationary mixed segment')
                signs=np.sign(np.diff(orientations))
                targets=[orientations[i] for i in range(1,len(orientations)-1)
                         if signs[i-1]*signs[i]<0]+[orientations[-1]]
                for target in targets:spin(target)
                arm_keep=np.r_[True,np.any(np.diff(part[:,3:],axis=0)!=0,axis=1)]
                arm_waypoints=part[arm_keep]
                require(len(arm_waypoints)>=2,'AMBIGUOUS_STATIONARY_MODE','no arm progress after split')
                spline=fit(arm_waypoints,limits);base=current[:3].copy()
                ev=lambda s,spline=spline,base=base:arm_only_jet(base,[spline.evaluate(s,k)[:,2:] for k in range(4)])
                ev.geometry_spline=spline
                decisions.append(dict(mode='SPIN_THEN_ARM',input_waypoints=len(part),
                                      spin_targets=len(targets),arm_waypoints=len(arm_waypoints),
                                      collision_recheck_required=True))
                append(PhaseType.ARM_ONLY,ev,'SPIN_THEN_ARM deterministic split of mixed stationary source')
            elif kind==PhaseType.ARM_ONLY:
                spin(part[0,2]);spline=fit(part,limits);base=current[:3].copy()
                ev=lambda s,spline=spline,base=base:arm_only_jet(base,[spline.evaluate(s,k)[:,2:] for k in range(4)])
                ev.geometry_spline=spline
                append(kind,ev,'8D QP fixed XY; arm-only shared parameter')
            else:
                options=[];rejected=[]
                for lift,meta in candidates(part,limits):
                    try:
                        ends=lift(np.array([0.,1.])).q
                        # Anchor candidate to the previous continuous theta branch.
                        shift=2*np.pi*np.round((current[2]-ends[0,2])/(2*np.pi))
                        def ev(s,lift=lift,shift=shift):
                            jet=lift(s);p=jet.q.copy();p[:,2]+=shift
                            return PathJet(p,jet.qs,jet.qss,jet.qsss)
                        ev.geometry_spline=lift.spline
                        ends=ev(np.array([0.,1.])).q
                        ph=PhasePath('candidate',kind,path.schema,ends[0],ends[1],ev,str(meta))
                        rr,vv=parameterize(ph,path,limits)
                        angles=[wrap(ends[0,2]-current[2]),wrap(part[-1,2]-ends[-1,2])]
                        cost=rr.time_s[-1]
                        for idx,delta in enumerate(angles):
                            if abs(delta)>1e-9:
                                st=current.copy() if idx==0 else ends[-1].copy()
                                fun=lambda s,st=st,delta=delta:spin_jet(s,st,delta)
                                ee=fun(np.array([0.,1.])).q
                                sp=PhasePath('cost_spin',PhaseType.SPIN,path.schema,ee[0],ee[-1],fun,'candidate cost')
                                sr,_=parameterize(sp,path,limits);cost+=sr.time_s[-1]
                        options.append(((cost,vv.metrics['xy_length_m'],sum(abs(a) for a in angles)),ev,meta))
                    except ContractError as e:rejected.append(e.code)
                require(bool(options),'G_NO_FEASIBLE_CANDIDATE',str(rejected))
                _,ev,meta=min(options,key=lambda item:item[0]);decisions.append(dict(selected=meta,rejected=rejected))
                spin(ev(np.array([0.])).q[0,2]);append(kind,ev,str(meta));spin(part[-1,2])
            lo=hi
    end_error=current-path.requested_goal;end_error[2]=wrap(end_error[2])
    require(np.max(np.abs(end_error))<1e-6,'FINAL_ENDPOINT','complete goal')
    return dict(phases=phases,decisions=decisions,geometry_s=time.perf_counter()-tic)


def parameterize_geometry(path,limits,geometry,hold_duration=None):
    tic=time.perf_counter();phases=geometry['phases'];records=[];checks=[];offset=0.
    for phase in phases:
        r,v=parameterize(phase,path,limits,offset,hold_duration)
        records.append(r);checks.append(v);offset=r.time_s[-1]
    output=concatenate(records)
    return dict(phases=phases,records=records,validation=checks,output=output,decisions=geometry['decisions'],
                backend_s=geometry.get('geometry_s',0.)+time.perf_counter()-tic,collision_status=Status.UNKNOWN,
                duration_s=float(output['time_s'][-1]),xy_length_m=sum(v.metrics['xy_length_m'] for v in checks))


def build_g(path,limits,hold_duration=None):
    return parameterize_geometry(path,limits,build_geometry(path,limits,hold_duration),hold_duration)
