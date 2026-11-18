"""Quintic rest-to-rest time law with uniform scaling and independent checks."""
import numpy as np
from ..core.contracts import *
from ..core.math_tools import quintic_time_jet,chain_rule,differential_kinematics
from ..validation.numerical import validate,validate_junctions
from ..core.limits import constraint_enabled


def sample(phase,path,duration,n,offset=0.):
    t=np.linspace(0,duration,n);s,v,a,j=quintic_time_jet(t,duration);jet=phase.evaluate(s)
    return TrajectoryRecord(path.trial_id,phase.phase_id,phase.phase_type,path.planner_id,path.parent_path_hash,
                            path.schema,t+offset,jet.q,*chain_rule(jet,v,a,j),'analytic spline + quintic chain rule')


def ratio(r,l):
    k=differential_kinematics(r.q,r.velocity,r.acceleration,float(l.wheel_radius.value),float(l.wheel_separation.value))
    vmax=[(r.velocity[:,:2],l.xy_component_velocity),(r.velocity[:,2],l.angular_velocity),(r.velocity[:,3:],l.arm_velocity)]
    if constraint_enabled(l,'xy_speed'): vmax.append((np.linalg.norm(r.velocity[:,:2],axis=1),l.xy_speed))
    if constraint_enabled(l,'wheel_velocity'): vmax.append((k['wheel_velocity'],l.wheel_velocity))
    amax=[(r.acceleration[:,:2],l.xy_component_acceleration),(r.acceleration[:,2],l.angular_acceleration),(r.acceleration[:,3:],l.arm_acceleration)]
    if constraint_enabled(l,'xy_acceleration'): amax.append((np.linalg.norm(r.acceleration[:,:2],axis=1),l.xy_acceleration))
    if constraint_enabled(l,'tangential_acceleration'): amax.append((k['forward_acceleration'],l.tangential_acceleration))
    if l.wheel_acceleration is not None and constraint_enabled(l,'wheel_acceleration'): amax.append((k['wheel_acceleration'],l.wheel_acceleration))
    scale=max([float(np.max(np.abs(v)/b.value)) for v,b in vmax]+[np.sqrt(float(np.max(np.abs(a)/b.value))) for a,b in amax])
    if r.phase_type==PhaseType.SPIN and constraint_enabled(l,'spin_jerk'):
        require(l.spin_jerk is not None,'MISSING_SPIN_JERK','explicit research bound')
        scale=max(scale,(float(np.max(np.abs(r.jerk[:,2])))/float(l.spin_jerk.value))**(1/3))
    return scale


def parameterize(phase,path,limits,offset=0.,hold_duration=None):
    if phase.phase_type==PhaseType.HOLD:
        require(hold_duration is not None and hold_duration>0,'HOLD_DURATION_REQUIRED','do not infer waits')
        duration=hold_duration
    else:
        # Nested grid convergence + 1% margin, not a continuous interval certificate.
        old=None
        for n in (1001,2001,4001,8001,16001):
            peak=ratio(sample(phase,path,1.,n),limits)
            if old is not None and abs(peak-old)<=1e-4*max(peak,1): break
            old=peak
        else: raise ContractError('TIME_GRID_UNRESOLVED','peak convergence budget')
        duration=max(.001,1.01*peak)
    for retry in range(5):
        n=max(1001,int(np.ceil(duration/.02))+1)*2**retry
        require(n<=200001,'TIME_SAMPLE_BUDGET',str(n))
        r=sample(phase,path,duration,n,offset)
        factor=ratio(r,limits)
        if factor>1: duration*=factor*1.01;continue
        result=validate(r,limits,phase.start,phase.end)
        if result.numerical_status==Status.PASS:return r,result
        require(all(c.startswith('INCONSISTENT_') for c in result.failure_codes),'G_VALIDATION',str(result.failure_codes))
    raise ContractError('G_VALIDATION','integration refinement exhausted')


def concatenate(records):
    if len(records)>1:
        result=validate_junctions(records)
        require(result.numerical_status==Status.PASS,'JUNCTION_DISCONTINUITY',str(result.checks))
    out={name:np.concatenate([getattr(r,name)[0 if i==0 else 1:] for i,r in enumerate(records)])
         for name in ('time_s','q','velocity','acceleration','jerk')}
    require(np.all(np.diff(out['time_s'])>0),'TIME_NOT_INCREASING','joined export')
    return out
