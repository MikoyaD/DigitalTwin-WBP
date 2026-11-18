"""Connect joint proposal and exact projections for diagnostic iterations only."""
import numpy as np
from .tm_builder import _rho_matrix
from .tm_rho_feedback import (diagnostic_initialize, exact_rho_projection,
                              integral_jacobians, global_jet_residual, nonlinear_step_check)
from .tm_rho_feedback_joint import propose
from ..core.math_tools import QuinticSpline


def initialize(problem):
    """Initialize each MOVE; no diagnostic center is a publishable H."""
    betas=[p['beta'].copy() for p in problem['profiles']]
    controls=[]; scales=[]; logs=[]; feasible=[]
    for i,m in enumerate(problem['moves']):
        target=m['p1']-m['p0']; h=m['h']; orders=np.tile([0,1],len(m['d'])//2)
        out=exact_rho_projection(m['A'],m['ref'],target,m['J'],m['d'])
        if out['status']=='PROJECTED':
            c=out['controls']; residual=global_jet_residual(m['J'],m['d'],c,h,orders)
            if len(residual) and np.max(np.abs(residual))>1e-6:
                out=dict(status='RECHECK_FAILED',reason='global_jet')
        if out['status']=='PROJECTED':
            feasible.append(i); init=out
        else:
            r0=max(1.,float(np.linalg.norm(target)/h),float(np.max(np.abs(m['d']))) if len(m['d']) else 0.)
            init=diagnostic_initialize(m['A'],m['ref'],target,m['J'],m['d'],r0)
            if init['status']!='DIAGNOSTIC_INIT':
                return dict(status='INITIALIZATION_FAILED', segment=m['segment'], detail=init, previous=logs)
            c=init['controls']; residual=global_jet_residual(m['J'],m['d'],c,h,orders)
            if len(residual) and np.max(np.abs(residual))>1e-6:
                return dict(status='INITIALIZATION_FAILED',segment=m['segment'],reason='global_jet',previous=logs)
        c=np.asarray(init['controls']).copy()
        controls.append(c);scales.append(max(1.,float(np.linalg.norm(target)/h),float(np.max(np.abs(c)))))
        logs.append(dict(segment=m['segment'],original_status=out['status'],initial_status=init['status'], closure_m=init.get('closure_m'),tube_excess_m=init.get('tube_excess_m'),alpha=init.get('alpha_min')))
    return dict(status='DIAGNOSTIC_INIT',beta=betas,controls=controls,
                original_beta=[x.copy() for x in betas],original_controls=[x.copy() for x in controls],
                scales=scales,previously_feasible=feasible,logs=logs)


def callbacks(problem):
    def propose_step(center,trust):
        profiles=[];moves=[]
        for i,p in enumerate(problem['profiles']):
            profiles.append(dict(beta=center['beta'][i],initial=center['original_beta'][i],
                                 basis=p['basis'],reference=p['reference']))
        for i,m in enumerate(problem['moves']):
            k=m['profile']; theta=QuinticSpline(center['beta'][k][:,None]); rho=QuinticSpline(center['controls'][i][:,None])
            p,A,K=integral_jacobians(m['x'],theta,rho,m['h'])
            moves.append(dict(profile=k,controls=center['controls'][i],initial=center['original_controls'][i],
                              scale=center['scales'][i],J=m['J'],d=m['d'],P=p,A=A,K=K,
                              reference=m['ref'],endpoint=m['p1']-m['p0']))
        return propose(profiles,moves,problem['theta_eq'],problem['theta_rhs'],trust)
    def evaluate(center,proposal,fraction):
        metric=nonlinear_step_check(problem['moves'],center['beta'],center['controls'],
                                    proposal['delta_theta'],proposal['delta_rho'],fraction)
        candidate=dict(center)
        candidate['beta']=[b+fraction*d for b,d in zip(center['beta'],proposal['delta_theta'])]
        candidate['controls']=[c+fraction*d for c,d in zip(center['controls'],proposal['delta_rho'])]
        thetaerr=problem['theta_eq']@np.concatenate(candidate['beta'])-problem['theta_rhs']
        hard=not len(thetaerr) or np.max(np.abs(thetaerr))<=1e-6
        for i,m in enumerate(problem['moves']):
            c=candidate['controls'][i];orders=np.tile([0,1],len(m['d'])//2)
            e=global_jet_residual(m['J'],m['d'],c,m['h'],orders)
            hard=hard and np.min(c)>=-1e-9 and (not len(e) or np.max(np.abs(e))<=1e-6)
        for i,p in enumerate(problem['profiles']):
            hard=hard and np.max(np.abs(p['basis']@candidate['beta'][i]-p['reference']))<=np.pi/2
        metric['theta_global_c2_error']=float(np.max(np.abs(thetaerr))) if len(thetaerr) else 0.
        return candidate,metric,hard
    def project_all(candidate):
        out=[]
        for m in problem['moves']:
            theta=QuinticSpline(candidate['beta'][m['profile']][:,None])
            A=_rho_matrix(m['x'],theta,14,m['h'])
            result=exact_rho_projection(A,m['ref'],m['p1']-m['p0'],m['J'],m['d'])
            if result['status']=='PROJECTED':
                orders=np.tile([0,1],len(m['d'])//2)
                err=global_jet_residual(m['J'],m['d'],result['controls'],m['h'],orders)
                result['global_jet_error']=float(np.max(np.abs(err))) if len(err) else 0.
                if result['global_jet_error']>1e-6: result['status']='RECHECK_FAILED'
            result['segment']=m['segment'];out.append(result)
        return out
    return propose_step,evaluate,project_all
