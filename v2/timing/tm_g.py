"""TM global quintic law with all chart/profile knots and one-sided seam jets."""
import time
import numpy as np
from ..core.contracts import TrajectoryRecord,PhaseType,Status,require
from ..core.math_tools import quintic_time_jet,chain_rule
from ..timing.unified_g import _inverse_quintic
from ..timing.feasible_profile import ratio
from ..validation.numerical import validate

def critical(block,extra=()):
    out=[0.,1.,*extra]
    for c in block.charts:
        out.extend([c.s0,c.s1]);h=c.s1-c.s0
        spline=c.spline if hasattr(c,'spline') else c.rho
        knots=c.s0+h*spline.knots
        out.extend(knots)
        if hasattr(c,'theta'):
            a,b=c.profile_interval or (c.s0,c.s1)
            require(a<=c.s0+1e-14 and b>=c.s1-1e-14 and b>a,'TM_PROFILE_RANGE','chart outside profile')
            for spl in (c.theta,c.arm):
                nodes=a+(b-a)*spl.knots
                out.extend(nodes[(nodes>=c.s0)&(nodes<=c.s1)])
        # At least nine points inside every local span, including short charts.
        ks=np.unique(np.r_[c.s0,c.s1,knots]);ks=ks[(ks>=c.s0)&(ks<=c.s1)]
        for x,y in zip(ks[:-1],ks[1:]):out.extend(np.linspace(x,y,9))
    return np.unique(np.clip(out,0,1))

def sample(block,path,duration,n,extra=()):
    fixed=np.array([_inverse_quintic(float(u)) for u in critical(block,extra)])*duration
    t=np.unique(np.r_[np.linspace(0,duration,n),fixed]);u,v,a,j=quintic_time_jet(t,duration)
    jet=block.evaluate(u)
    r=TrajectoryRecord(path.trial_id,block.block_id,PhaseType.TRANSLATE,path.planner_id,path.parent_path_hash,
                       path.schema,t,jet.q,*chain_rule(jet,v,a,j),'TM analytic shared global quintic')
    return r,u

def seams(block,duration):
    out=[]
    for l,r in zip(block.charts[:-1],block.charts[1:]):
        t=_inverse_quintic(l.s1)*duration;_,v,a,j=quintic_time_jet(np.array([t]),duration)
        jl=l.evaluate_global([l.s1]);jr=r.evaluate_global([r.s0]);dl=chain_rule(jl,v,a,j);dr=chain_rule(jr,v,a,j)
        error=[float(np.max(abs(jl.q-jr.q)))]+[float(np.max(abs(x-y))) for x,y in zip(dl[:2],dr[:2])]
        require(max(error)<=1e-6,'TM_TIME_C2_SEAM',str(error))
        out.append(dict(u=l.s1,time_s=t,q_v_a_error=error,jerk_left=dl[2][0].tolist(),jerk_right=dr[2][0].tolist(),
                        whole_body_speed=float(np.max(abs(dl[0])))))
    return out

def parameterize(block,path,limits,extra=()):
    start=time.perf_counter();last=None
    for n in (1001,2001,4001,8001,16001):
        r,_=sample(block,path,1.,n,extra);peak=ratio(r,limits)
        if last is not None and abs(peak-last)<=1e-4*max(peak,1):break
        last=peak
    else: raise ValueError('TM_TIME_GRID_UNRESOLVED')
    T=max(.001,1.01*peak)
    for i in range(5):
        n=max(1001,int(np.ceil(T/.02))+1)*2**i
        require(n<=200001,'TM_TIME_BUDGET',str(n))
        r,u=sample(block,path,T,n,extra);scale=ratio(r,limits)
        if scale>1:T*=scale*1.01;continue
        check=validate(r,limits,block.start,block.end)
        if check.numerical_status==Status.PASS:
            sj=seams(block,T)
            return dict(record=r,validation=check,duration_s=T,uniform_n=n,seams=sj,
                        backend_s=time.perf_counter()-start)
        require(all(x.startswith('INCONSISTENT_') for x in check.failure_codes),'TM_G_VALIDATION',repr(check.failure_codes))
    raise ValueError('TM_G_VALIDATION_REFINEMENT_EXHAUSTED')
