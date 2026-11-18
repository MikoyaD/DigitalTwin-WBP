"""Independent TM all-explicit candidate: global profile QP + per-MOVE rho QP.

This conservative candidate is deliberately finite and may return infeasible;
no source point, reference tube or physical bound is relaxed.
"""
import time
import numpy as np
import cvxpy as cp
from .tm_modes import classify
from .unified_chart import ExplicitChart
from .unified_block import MotionBlock, _reference_at
from ..core.math_tools import QuinticSpline, quintic_basis
from ..core.contracts import require


def _basis(x,n,k=0): return quintic_basis(np.asarray(x),n,k)

def _rho_matrix(x, theta, n, length):
    """Gauss integration, split on both independent spline knot sets."""
    nodes,weights=np.polynomial.legendre.leggauss(24)
    knots=np.unique(np.r_[QuinticSpline(np.zeros((n,1))).knots,theta.knots])
    out=np.zeros((len(x),2,n))
    for i,end in enumerate(x):
        cuts=np.r_[0.,knots[(knots>0)&(knots<end)],end]
        for a,b in zip(cuts[:-1],cuts[1:]):
            if b<=a: continue
            v=(a+b)/2+(b-a)*nodes/2
            th=theta.evaluate(v)[:,0]
            e=np.column_stack([np.cos(th),np.sin(th)])
            out[i]+=length*(b-a)/2*np.einsum('i,ij,ik->jk',weights,e,_basis(v,n))
    return out


def build(path,limits):
    started=time.perf_counter();q,u,source,modes,eps=classify(path)
    m=18; cs=[cp.Variable((m,7)) for _ in modes]; obj=0; cons=[]
    lo=limits.arm_lower.value; hi=limits.arm_upper.value
    for k,mode in enumerate(modes):
        a,b=mode['start'],mode['end'];h=mode['u1']-mode['u0']
        points=np.unique(np.r_[u[a:b+1],np.linspace(mode['u0'],mode['u1'],101)])
        x=(points-mode['u0'])/h; B=_basis(x,m); c=cs[k];target=_reference_at(u,q,points)[:,2:]
        cons += [B@c-target<=np.tile(np.r_[np.pi/2,np.full(6,.0999)],(len(x),1)),
                 B@c-target>=-np.tile(np.r_[np.pi/2,np.full(6,.0999)],(len(x),1)),
                 c[:,1:]>=np.tile(lo,(m,1)),c[:,1:]<=np.tile(hi,(m,1))]
        obj+=cp.sum_squares(B@c-target)/len(x)
        d3=_basis(np.linspace(0,1,41),m,3)
        obj+=1e-8/h**5*cp.sum_squares(d3@c)/41
        if k:
            prev=modes[k-1];hp=prev['u1']-prev['u0'];c0=cs[k-1]
            for order in range(3):
                # Row scaling is algebraically equivalent and avoids 1/h^2 blow-up.
                scale=min(h,hp)**order
                cons += [(_basis([1.],m,order)@c0)*(scale/hp**order)==
                         (_basis([0.],m,order)@c)*(scale/h**order)]
    cons += [cs[0][0,:]==q[0,2:],cs[-1][-1,:]==q[-1,2:]]
    problem=cp.Problem(cp.Minimize(obj),cons)
    problem.solve(solver='CLARABEL',max_iter=200,tol_gap_abs=1e-9,tol_feas=1e-9)
    require(problem.status in ('optimal','optimal_inaccurate') and all(c.value is not None for c in cs),
            'TM_PROFILE_QP',str(problem.status))
    charts=[]
    for k,mode in enumerate(modes):
        a,b=mode['start'],mode['end'];s0,s1=mode['u0'],mode['u1'];h=s1-s0
        fitted=np.asarray(cs[k].value);theta=QuinticSpline(fitted[:,:1]);arm=QuinticSpline(fitted[:,1:])
        if mode['kind']=='ADJUST':
            require(np.max(np.ptp(fitted,axis=0))>1e-10,'ADJUST_NO_MOTION','profile constant')
            chart=ExplicitChart(s0,s1,np.asarray(mode['anchor']),QuinticSpline(np.zeros((6,1))),theta,arm,True)
        else:
            n=14; points=np.unique(np.r_[u[a:b+1],np.linspace(s0,s1,101)]);x=(points-s0)/h; A=_rho_matrix(x,theta,n,h)
            start=np.asarray(modes[k-1]['anchor']) if k and modes[k-1]['kind']=='ADJUST' else q[a,:2]
            end=np.asarray(modes[k+1]['anchor']) if k+1<len(modes) and modes[k+1]['kind']=='ADJUST' else q[b,:2]
            c=cp.Variable(n); pred=cp.hstack([cp.reshape(A[:,d,:]@c,(len(x),1),order='C') for d in range(2)])
            ref=_reference_at(u,q,points)[:,:2]-start
            conditions=[c>=0,pred[-1,:]==end-start,pred-ref<=.0999,pred-ref>=-.0999]
            if k and modes[k-1]['kind']=='ADJUST': conditions += [c[0]==0,c[1]==0]
            if k+1<len(modes) and modes[k+1]['kind']=='ADJUST': conditions += [c[-1]==0,c[-2]==0]
            d2=_basis(np.linspace(0,1,41),n,2)/h**2
            opt=cp.Problem(cp.Minimize(cp.sum_squares(pred-ref)/len(x)+1e-6*h*cp.sum_squares(d2@c)/41),conditions)
            opt.solve(solver='CLARABEL',max_iter=200,tol_gap_abs=1e-10,tol_feas=1e-10)
            require(opt.status in ('optimal','optimal_inaccurate') and c.value is not None,'TM_RHO_QP',str(opt.status)+' mode='+str(k))
            v=np.asarray(c.value);require(np.min(v)>=-1e-9,'TM_RHO_NEGATIVE',str(np.min(v)))
            # Only solver roundoff is projected; all constraints are subsequently independently rechecked.
            v=np.maximum(v,0.)
            if k and modes[k-1]['kind']=='ADJUST': v[:2]=0.
            if k+1<len(modes) and modes[k+1]['kind']=='ADJUST': v[-2:]=0.
            rho=QuinticSpline(v[:,None]);grid=np.unique(np.r_[rho.knots,(rho.knots[:-1]+rho.knots[1:])/2]);grid=grid[(grid>0)&(grid<1)]
            require(np.min(rho.evaluate(grid)[:,0])>0,'MOVE_INTERIOR_STATIONARY','rho vanishes inside MOVE')
            chart=ExplicitChart(s0,s1,start,rho,theta,arm,False)
            require(np.max(abs(chart.evaluate_global([s1]).q[0,:2]-end))<1e-6,'TM_XY_ENDPOINT','rho integral')
            probe=np.linspace(0,1,5)
            require(np.max(abs(chart.positions(probe)-np.vstack([chart._xy(float(x)) for x in probe])))<1e-8,
                    'TM_QUADRATURE','Gauss versus quad')
        charts.append(chart)
    block=MotionBlock('tm_000',tuple(charts),path.schema)
    require(np.max(abs(block.start-q[0]))<1e-6 and np.max(abs(block.end-q[-1]))<1e-6,'TM_ENDPOINT','parent endpoint')
    worst=np.zeros(3)
    for chart in charts:
        grid=np.unique(np.r_[np.linspace(chart.s0,chart.s1,1001),u[(u>=chart.s0)&(u<=chart.s1)]])
        jet=chart.evaluate_global(grid);ref=_reference_at(u,q,grid);dev=np.max(abs(jet.q-ref),axis=0)
        worst=np.maximum(worst,[np.max(dev[:2]),dev[2],np.max(dev[3:])])
        lat=-np.sin(jet.q[:,2])*jet.qs[:,0]+np.cos(jet.q[:,2])*jet.qs[:,1]
        forward=np.cos(jet.q[:,2])*jet.qs[:,0]+np.sin(jet.q[:,2])*jet.qs[:,1]
        require(np.max(abs(lat))<=1e-8 and np.min(forward)>=-1e-9,'TM_NONHOLONOMIC','geometry')
        require(np.all(jet.q[:,3:]>=lo-1e-6) and np.all(jet.q[:,3:]<=hi+1e-6),'TM_POSITION_LIMIT','arm')
    require(worst[0]<=.1 and worst[1]<=np.pi/2 and worst[2]<=.1,'TM_REFERENCE_TUBE',repr(worst.tolist()))
    return block,dict(modes=modes,source_indices=source.tolist(),progress=u.tolist(),epsilon_xy=eps,
                      reference_tube_max=worst.tolist(),h_build_s=time.perf_counter()-started,
                      candidate='all_explicit',scope='sampled; collision not checked')


def build_with_tail_fallback(path,limits):
    """Bounded geometry candidates, plus an explicitly identified compatible Fast tail."""
    started=time.perf_counter();attempts=[]
    try:
        block,decision=build(path,limits)
        decision['attempts']=[dict(candidate='all_explicit',status='PASS')]
        decision['h_build_s']=time.perf_counter()-started
        return block,decision
    except Exception as first:
        attempts.append(dict(candidate='all_explicit',status='FAIL',error=str(first)))
    from .tm_local import build_local
    for fraction in (1/16,1/8,1/4,1/2):
        for seed in (0,1):
            candidate='local_regular_%g_seed%d'%(fraction,seed)
            try:
                block,decision=build_local(path,limits,fraction,seed)
                attempts.append(dict(candidate=candidate,status='PASS'))
                decision['attempts']=attempts;decision['h_build_s']=time.perf_counter()-started
                return block,decision
            except Exception as exc:
                attempts.append(dict(candidate=candidate,status='FAIL',error=str(exc)))
    _,u,_,modes,_=classify(path)
    if [m['kind'] for m in modes] == ['MOVE','ADJUST']:
        from .unified_block import build_unified
        try:
            block,decision=build_unified(path,limits)
            require(abs(block.charts[-1].s0-modes[1]['u0'])<1e-12,'TM_FALLBACK_MODE_MISMATCH','tail')
            attempts.append(dict(candidate='fast_tail_reuse',status='PASS'))
            return block,dict(modes=modes,progress=u.tolist(),candidate='fast_tail_reuse',attempts=attempts,
                              h_build_s=time.perf_counter()-started,fast_decision=decision,
                              scope='reused Fast geometry; sampled, collision not checked')
        except Exception as exc:
            attempts.append(dict(candidate='fast_tail_reuse',status='FAIL',error=str(exc)))
    raise ValueError('TM_CANDIDATES_EXHAUSTED: '+repr(attempts))
