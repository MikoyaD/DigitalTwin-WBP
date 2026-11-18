"""Seam-aware numerical integration audit for frozen TM feedback geometry."""
import numpy as np
from numpy.polynomial.legendre import leggauss
from ..core.math_tools import quintic_time_jet,chain_rule
from ..timing.unified_g import _inverse_quintic
from ..timing.tm_g import seams


def integrate(block,T,extra=(),*,max_spans=20000,convergence=1e-8):
    if not np.isfinite(T) or T<=0:raise ValueError('INVALID_DURATION')
    qerr=np.zeros(9);verr=np.zeros(9);local_v=np.zeros(9);local_a=np.zeros(9)
    accum_v=np.zeros(9);accum_a=np.zeros(9);start_v=start_a=None;records=[]; count=0
    seam_results=seams(block,T)
    for ci,c in enumerate(block.charts):
        h=c.s1-c.s0; cuts=[c.s0,c.s1];spl=c.spline if hasattr(c,'spline') else c.rho
        cuts.extend(c.s0+h*spl.knots)
        if hasattr(c,'theta'):
            a,z=c.profile_interval or (c.s0,c.s1)
            for sp in (c.theta,c.arm):cuts.extend(a+(z-a)*sp.knots)
        cuts.extend(x for x in extra if c.s0 < x < c.s1)
        cuts=np.unique(np.clip(cuts,c.s0,c.s1));times=np.array([_inverse_quintic(float(x))*T for x in cuts])
        for si,(t0,t1) in enumerate(zip(times[:-1],times[1:])):
            if t1<=t0:raise ValueError('NONINCREASING_EVENT_TIME')
            count+=1
            if count>max_spans:return dict(status='UNKNOWN',reason='INTEGRATION_BUDGET',spans=count)
            def eval_at(t):
                u,ud,udd,uj=quintic_time_jet(np.asarray(t),T)
                jet=c.evaluate_global(np.clip(u,c.s0,c.s1));v,a,j=chain_rule(jet,ud,udd,uj)
                return jet.q,v,a,j
            q,v,a,j=eval_at([t0,t1])
            if start_v is None:start_v=v[0].copy();start_a=a[0].copy()
            def quadrature(lo,hi,depth):
                estimates=[]
                for order in (16,32):
                    x,w=leggauss(order); ts=(lo+hi)/2+(hi-lo)*x/2
                    _,_,av,jv=eval_at(ts);w=w*(hi-lo)/2
                    estimates.append((w@av,w@jv))
                dv,da=estimates[-1]
                gap=max(float(np.max(abs(estimates[0][0]-dv))),float(np.max(abs(estimates[0][1]-da))))
                if gap<=convergence:return dv,da,gap
                if depth==4:raise ArithmeticError('QUADRATURE_NOT_CONVERGED')
                mid=(lo+hi)/2; v0,a0,g0=quadrature(lo,mid,depth+1);v1,a1,g1=quadrature(mid,hi,depth+1)
                return v0+v1,a0+a1,max(g0,g1)
            try:dv,da,gap=quadrature(t0,t1,0)
            except ArithmeticError:return dict(status='UNKNOWN',reason='QUADRATURE_NOT_CONVERGED',chart=ci,span=si)
            ev=v[-1]-v[0]-dv; ea=a[-1]-a[0]-da
            local_v=np.maximum(local_v,abs(ev));local_a=np.maximum(local_a,abs(ea));accum_v+=dv;accum_a+=da
            qerr=np.maximum(qerr,abs(v[-1]-start_v-accum_v));verr=np.maximum(verr,abs(a[-1]-start_a-accum_a))
            records.append(dict(chart=ci,span=si,u=[float(cuts[si]),float(cuts[si+1])],time=[float(t0),float(t1)],v_error=ev.tolist(),a_error=ea.tolist(),global_v_error=(v[-1]-start_v-accum_v).tolist(),global_a_error=(a[-1]-start_a-accum_a).tolist(),convergence=gap))
    ok=bool(np.max(local_v)<=1e-3 and np.max(qerr)<=1e-3 and np.max(local_a)<=1e-2 and np.max(verr)<=1e-2)
    return dict(status='NUMERIC_INTEGRATION_PASS' if ok else 'FAIL',local_v=local_v.tolist(),local_a=local_a.tolist(),global_v=qerr.tolist(),global_a=verr.tolist(),spans=count,records=records,seams=seam_results)
