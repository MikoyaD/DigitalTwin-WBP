"""One legacy quintic G law per contract-v2 MotionBlock, not per chart."""
import time
import numpy as np
from scipy.optimize import brentq
from ..core.contracts import TrajectoryRecord,PhaseType,Status,ContractError,require
from ..core.math_tools import quintic_time_jet,chain_rule
from ..timing.feasible_profile import ratio as legacy_ratio
from ..validation.numerical import validate
from ..core.limits import constraint_enabled


def critical_progress(block):
    points=[0.,1.]
    for chart in block.charts:
        points += [chart.s0,chart.s1]
        knots=(chart.s0+(chart.s1-chart.s0)*chart.spline.knots
               if hasattr(chart,'spline') else chart.s0+(chart.s1-chart.s0)*chart.rho.knots)
        points += list(knots)
        if hasattr(chart,'theta'):
            profile=chart.profile_interval or (chart.s0,chart.s1)
            points += list(profile[0]+(profile[1]-profile[0])*chart.theta.knots)
    return np.unique(np.clip(points,0,1))


def _inverse_quintic(u):
    if u<=0:return 0.
    if u>=1:return 1.
    return brentq(lambda x:x*x*x*(10-15*x+6*x*x)-u,0,1,xtol=1e-14)


def sample_block(block,path,duration,n):
    fixed=np.array([_inverse_quintic(float(u)) for u in critical_progress(block)])*duration
    t=np.unique(np.r_[np.linspace(0,duration,n),fixed])
    u,v,a,j=quintic_time_jet(t,duration);jet=block.evaluate(u)
    record=TrajectoryRecord(path.trial_id,block.block_id,PhaseType.TRANSLATE,
                            path.planner_id,path.parent_path_hash,path.schema,t,jet.q,
                            *chain_rule(jet,v,a,j),
                            'contract_v2 analytic chart derivatives + one block quintic time')
    return record,u


def spin_scope(block,u):
    # Exact chart identity, not a tiny XY speed threshold.  The legacy
    # TRANSLATE enum is only an internal adapter for common limit checks.
    return np.asarray([any(getattr(c,'stationary',False) and c.s0-1e-14<=x<=c.s1+1e-14
                           for c in block.charts) for x in u])


def ratio_v2(record,limits,block,u):
    scale=legacy_ratio(record,limits)
    mask=spin_scope(block,u)
    if np.any(mask) and constraint_enabled(limits,'spin_jerk'):
        require(limits.spin_jerk is not None,'MISSING_SPIN_JERK','research limit')
        scale=max(scale,(float(np.max(np.abs(record.jerk[mask,2])))/float(limits.spin_jerk.value))**(1/3))
    return scale


def validate_v2(record,limits,block,u):
    result=validate(record,limits,block.start,block.end)
    mask=spin_scope(block,u)
    if np.any(mask) and constraint_enabled(limits,'spin_jerk'):
        if limits.spin_jerk is None:
            result.failure_codes.append('MISSING_SPIN_JERK');result.numerical_status=Status.FAIL
        else:
            peak=float(np.max(np.abs(record.jerk[mask,2])));cap=float(limits.spin_jerk.value)
            good=peak<=cap*(1+1e-4)+1e-9
            result.checks['physical_stationary_theta_jerk']=dict(status='PASS' if good else 'FAIL',
                           detail=dict(peak=peak,limit=cap,unit='rad/s^3',
                                       basis='identically fixed XY chart, arm may move'))
            if not good:result.failure_codes.append('LIMIT_SPIN_JERK');result.numerical_status=Status.FAIL
    elif np.any(mask):
        result.checks['physical_stationary_theta_jerk']=dict(status='DISABLED',
            detail=dict(peak=float(np.max(np.abs(record.jerk[mask,2]))),unit='rad/s^3'))
    result.metrics['spin_angle_rad']=float(np.trapz(np.abs(record.velocity[mask,2]),record.time_s[mask])) if np.sum(mask)>1 else 0.
    result.metrics['spin_time_s']=float(record.time_s[mask][-1]-record.time_s[mask][0]) if np.sum(mask)>1 else 0.
    return result


def parameterize_block(block,path,limits):
    started=time.perf_counter();previous=None
    for n in (1001,2001,4001,8001,16001):
        record,u=sample_block(block,path,1.,n);peak=ratio_v2(record,limits,block,u)
        if previous is not None and abs(peak-previous)<=1e-4*max(peak,1):break
        previous=peak
    else:raise ContractError('TIME_GRID_UNRESOLVED','V2 block peak convergence')
    duration=max(.001,1.01*peak)
    for retry in range(5):
        n=max(1001,int(np.ceil(duration/.02))+1)*2**retry
        require(n<=200001,'TIME_SAMPLE_BUDGET',str(n))
        record,u=sample_block(block,path,duration,n);factor=ratio_v2(record,limits,block,u)
        if factor>1:duration*=factor*1.01;continue
        result=validate_v2(record,limits,block,u)
        if result.numerical_status==Status.PASS:
            array=dict(time_s=record.time_s,q=record.q,velocity=record.velocity,
                       acceleration=record.acceleration,jerk=record.jerk)
            return dict(record=record,output=array,validation=result,
                        uniform_n=n,
                        duration_s=float(record.time_s[-1]),
                        xy_length_m=float(result.metrics['xy_length_m']),
                        backend_s=float(time.perf_counter()-started),
                        spin_scope_samples=int(np.sum(spin_scope(block,u))))
        require(all(code.startswith('INCONSISTENT_') for code in result.failure_codes),
                'G_VALIDATION',str(result.failure_codes))
    raise ContractError('G_VALIDATION','integration check refinement exhausted')
