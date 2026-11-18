"""Joint linearized theta/rho diagnostic proposal, not a trajectory builder."""
import numpy as np
import cvxpy as cp
from .tm_rho_feedback import TUBE, _solve


def propose(profiles, moves, theta_eq, theta_rhs, trust_factor=1.):
    """Solve the two-level elastic convex proposal for *all* explicit moves.

    profiles: [{'beta':(18,), 'initial':(18,), 'basis':(N,18),
                'reference':(N,)}], including ADJUST charts.
    moves: [{'profile':index, 'controls':(14,), 'initial':(14,),
             'scale':positive, 'J':(M,14), 'd':(M,),
             'P':(N,2), 'A':(N,2,14), 'K':(N,2,18),
             'reference':(N,2), 'endpoint':(2,)}].
    theta_eq/theta_rhs impose all global-u C2 and regular endpoint jets.
    The result has DIAGNOSTIC_PROPOSAL status, never formal success.
    """
    if not profiles or not moves or not 0 < trust_factor <= 1:
        raise ValueError('INVALID_FEEDBACK_PROBLEM')
    count=len(profiles); E=np.asarray(theta_eq,float); f=np.asarray(theta_rhs,float)
    if E.ndim != 2 or E.shape[1]!=18*count or f.shape != (len(E),):
        raise ValueError('INVALID_THETA_EQUALITY')
    beta=[np.asarray(p['beta'],float) for p in profiles]
    initial=[np.asarray(p['initial'],float) for p in profiles]
    if any(x.shape!=(18,) or not np.isfinite(x).all() for x in beta+initial):
        raise ValueError('INVALID_THETA_CONTROLS')
    db=[cp.Variable(18) for _ in profiles]; dc=[cp.Variable(14) for _ in moves]
    alpha=cp.Variable(nonneg=True); cons=[]
    joined=cp.hstack([beta[i]+db[i] for i in range(count)])
    if len(E):
        norm=np.linalg.norm(E,axis=1)
        if np.any(norm == 0):raise ValueError('ZERO_THETA_EQUALITY')
        cons.append((E/norm[:,None])@joined==f/norm)
    objective=0.
    for i,p in enumerate(profiles):
        B=np.asarray(p['basis'],float); ref=np.asarray(p['reference'],float)
        if B.ndim!=2 or B.shape[1]!=18 or ref.shape!=(len(B),):raise ValueError('INVALID_THETA_REFERENCE')
        b=beta[i]+db[i]; difference=B@b-ref
        cons += [difference<=np.pi/2, difference>=-np.pi/2,
                 db[i]<=.05*trust_factor,db[i]>=-.05*trust_factor,
                 b-initial[i]<=.1,b-initial[i]>=-.1]
        objective+=cp.sum_squares(db[i]/.05)
    for i,m in enumerate(moves):
        k=int(m['profile']); c=np.asarray(m['controls'],float); c0=np.asarray(m['initial'],float)
        J=np.asarray(m['J'],float); d=np.asarray(m['d'],float); scale=float(m['scale'])
        P=np.asarray(m['P'],float); A=np.asarray(m['A'],float); K=np.asarray(m['K'],float)
        ref=np.asarray(m['reference'],float); end=np.asarray(m['endpoint'],float);N=len(P)
        if not (0<=k<count and c.shape==(14,) and c0.shape==(14,) and J.ndim==2 and J.shape[1]==14 and d.shape==(len(J),) and scale>0 and np.isfinite(scale) and P.shape==(N,2) and A.shape==(N,2,14) and K.shape==(N,2,18) and ref.shape==(N,2) and end.shape==(2,)):
            raise ValueError('INVALID_MOVE')
        newc=c+dc[i]
        cons += [newc>=0, dc[i]<=.5*scale*trust_factor,dc[i]>=-.5*scale*trust_factor,
                 newc-c0<=scale,newc-c0>=-scale]
        if len(J):
            norm=np.linalg.norm(J,axis=1)
            if np.any(norm==0):raise ValueError('ZERO_JET_ROW')
            cons.append((J/norm[:,None])@newc==d/norm)
        pred=cp.vstack([P[:,axis]+A[:,axis,:]@dc[i]+K[:,axis,:]@db[k] for axis in range(2)]).T
        cons += [pred[-1]-end<=alpha*TUBE,pred[-1]-end>=-alpha*TUBE,
                 pred-ref<=TUBE*(1+alpha),pred-ref>=-TUBE*(1+alpha)]
        objective+=cp.sum_squares(dc[i]/scale)
    first=cp.Problem(cp.Minimize(alpha),cons); status=_solve(first)
    if status not in ('optimal','optimal_inaccurate') or alpha.value is None:
        return dict(status='SOLVER_INCONCLUSIVE',solver_status=status)
    best=float(alpha.value)
    second=cp.Problem(cp.Minimize(objective),cons+[alpha<=best+1e-10]); status2=_solve(second)
    if status2 not in ('optimal','optimal_inaccurate') or any(v.value is None for v in db+dc):
        return dict(status='SOLVER_INCONCLUSIVE',solver_status=status2,alpha_min=best)
    # Diagnostic only: caller must independently check unscaled equalities,
    # true integrals, all original QPs and complete geometry before use.
    return dict(status='DIAGNOSTIC_PROPOSAL',solver_status=status2,alpha_min=best,
                delta_theta=[np.asarray(v.value).copy() for v in db],
                delta_rho=[np.asarray(v.value).copy() for v in dc])
