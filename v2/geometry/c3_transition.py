"""Independent experimental C3 geometry; immutable contract-v2 parent input.

This module is not connected to any default planner.  Sampling checks are
engineering checks, not interval certification or collision verification.
"""
import numpy as np
import cvxpy as cp
from scipy.interpolate import insert
from .unified_chart import ExplicitChart, RegularChart, nine_dimensional_progress
from .unified_block import MotionBlock
from ..core.math_tools import QuinticSpline, quintic_basis


def restrict_prefix(old, end):
    """Exact knot insertion/restriction, maintaining global u and heading branch."""
    if not old.s0 < end <= old.s1:
        raise ValueError('INVALID_PREFIX_END')
    if end == old.s1:
        return old
    x=(end-old.s0)/(old.s1-old.s0)
    spline=old.spline
    existing=int(np.count_nonzero(spline.knots == x))
    cols=[]
    for k in range(spline.controls.shape[1]):
        t,c,_=insert(x,(spline.knots,spline.controls[:,k],5),m=5-existing)
        idx=int(np.searchsorted(t,x,side='left'))
        cols.append(c[:idx])
    knots=np.r_[t[:idx]/x,np.ones(6)]
    chart=RegularChart(old.s0,float(end),QuinticSpline(np.array(cols).T,knots),old.heading_anchor)
    check=np.unique(np.r_[np.linspace(old.s0,end,301),old.s0+(old.s1-old.s0)*spline.knots])
    check=check[check<=end]
    for attr in ('q','qs','qss','qsss'):
        diff=np.max(np.abs(getattr(chart.evaluate_global(check),attr)-getattr(old.evaluate_global(check),attr)))
        if diff>1e-6:raise ValueError('PREFIX_MISMATCH %s %.9g'%(attr,diff))
    return chart


def integral_matrix(s, a, b, theta, profile_len, n=14):
    """XY integral operator; 24-point Gauss on union of both knot sets."""
    L=b-a; xi=np.asarray((np.asarray(s)-a)/L); kn=QuinticSpline(np.zeros((n,1))).knots
    knots=np.unique(np.r_[kn,theta.knots*profile_len/L,0.,1.]);knots=knots[(knots>=0)&(knots<=1)]
    nodes,weights=np.polynomial.legendre.leggauss(24)
    def integrate(lo,hi):
        z=(lo+hi)/2+(hi-lo)*nodes/2
        angle=theta.evaluate(z*L/profile_len)[:,0]
        return L*(hi-lo)/2*np.einsum('i,ij,ik->jk',weights,np.column_stack((np.cos(angle),np.sin(angle))),quintic_basis(z,n))
    out=np.zeros((len(xi),2,n))
    for i,x in enumerate(xi):
        cuts=np.r_[0.,knots[(knots>0)&(knots<x)],x]
        for lo,hi in zip(cuts[:-1],cuts[1:]):
            if hi>lo:out[i]+=integrate(lo,hi)
    return out


def physical_rho_operators(R,a,b,theta,H):
    """Unit-time G acceleration/jerk operators at fixed heading."""
    from scipy.optimize import brentq
    R=np.asarray(R);L=b-a
    tau=np.array([brentq(lambda t:t**3*(10-15*t+6*t*t)-u,0,1) if 0<u<1 else u for u in R])
    g=30*tau**2*(1-tau)**2;h=60*tau*(1-tau)*(1-2*tau);k=60*(1-6*tau+6*tau*tau)
    Xr=(R-a)/L;B0=quintic_basis(Xr,14);B1=quintic_basis(Xr,14,1)/L;B2=quintic_basis(Xr,14,2)/L**2
    xp=(R-a)/H;th=theta.evaluate(xp)[:,0];th1=theta.evaluate(xp,1)[:,0]/H;th2=theta.evaluate(xp,2)[:,0]/H**2
    E=np.column_stack((np.cos(th),np.sin(th)));EP=np.column_stack((-np.sin(th),np.cos(th)))
    F=(g*g)[:,None]*B1+h[:,None]*B0
    Axy=[E[:,j,None]*F+EP[:,j,None]*(th1*g*g)[:,None]*B0 for j in range(2)]
    Jpar=(g**3)[:,None]*(B2-(th1**2)[:,None]*B0)+(3*g*h)[:,None]*B1+k[:,None]*B0
    Jper=(g**3)[:,None]*(2*th1[:,None]*B1+th2[:,None]*B0)+(3*g*h*th1)[:,None]*B0
    Jxy=[E[:,j,None]*Jpar+EP[:,j,None]*Jper for j in range(2)]
    return F,Axy,Jxy,th2*g*g+th1*h


def _interp(s,q,u):
    return np.column_stack([np.interp(u,s,q[:,k]) for k in range(9)])


def _fixed_controls(m, values, order):
    mat=np.vstack([quintic_basis([0.],m,k)[0,:order] for k in range(order)])
    return np.linalg.solve(mat,np.asarray(values))


def make_candidate(path,limits,original,entry,mode='A',weight=.1,audit=None):
    """Build one bounded candidate; raises on any unresolved geometry condition."""
    old_regular,old_moving,old_stationary=original.charts
    audit={} if audit is None else audit
    audit.update(mode=mode,weight=float(weight),entry=float(entry),exit=float(old_moving.s1),regular_source_interval=[old_regular.s0,old_regular.s1],rho_control_count=14,profile_control_count=18,rho_knots=QuinticSpline(np.zeros((14,1))).knots.tolist(),profile_knots=QuinticSpline(np.zeros((18,1))).knots.tolist())
    q,s,_=nine_dimensional_progress(path.q)
    a=float(entry);b=float(old_moving.s1); L=b-a; H=1-a
    reg=restrict_prefix(old_regular,a);st=reg.evaluate_global([a]);
    rho0=float(np.linalg.norm(st.qs[0,:2]));e=st.qs[0,:2]/rho0;ep=np.array([-e[1],e[0]])
    ru=float(e@st.qss[0,:2]);ruu=float(e@st.qsss[0,:2]+rho0*st.qs[0,2]**2)
    if abs(ep@st.qss[0,:2]-rho0*st.qs[0,2])>1e-6 or abs(ep@st.qsss[0,:2]-2*ru*st.qs[0,2]-rho0*st.qss[0,2])>1e-6:raise ValueError('ENTRY_NORMAL_MISMATCH')
    # Eliminate exact endpoint controls instead of clipping post-solve.
    fvalues=np.vstack([getattr(st,k)[0,2:]*H**i for i,k in enumerate(('q','qs','qss','qsss'))])
    first=_fixed_controls(18,fvalues,4)
    U=np.unique(np.r_[np.linspace(a,1.,501),s[s>=a]])
    X=(U-a)/H;B=quintic_basis(X,18);old=original.evaluate(U).q[:,2:];ref=_interp(s,q,U)[:,2:]
    fixed_end=np.asarray(path.q[-1,2:]);free=cp.Variable((13,7));C=cp.vstack([first,free,fixed_end.reshape(1,7)])
    cap=np.tile(np.r_[np.pi/2,np.full(6,.1)],(len(U),1))
    con=[C[:,1:]>=np.tile(limits.arm_lower.value,(18,1)),C[:,1:]<=np.tile(limits.arm_upper.value,(18,1)),B@C-ref<=cap,ref-B@C<=cap]
    Q=B.T@B/len(U); profile_target=ref if mode=='B' else old
    linear=B.T@profile_target/len(U); scales=np.r_[np.pi/2,np.full(6,.1)] if mode=='B' else np.ones(7)
    obj=sum((cp.quad_form(C[:,i],cp.psd_wrap(Q))-2*linear[:,i]@C[:,i])/scales[i]**2 for i in range(7))
    if mode=='B':
        from scipy.optimize import brentq
        tau=np.array([brentq(lambda t:t**3*(10-15*t+6*t*t)-u,0,1) if 0<u<1 else u for u in U]);g=30*tau**2*(1-tau)**2;h=60*tau*(1-tau)*(1-2*tau);k=60*(1-6*tau+6*tau*tau)
        D1=quintic_basis(X,18,1)/H;D2=quintic_basis(X,18,2)/H**2;D3=quintic_basis(X,18,3)/H**3
        Jop=(g**3)[:,None]*D3+(3*g*h)[:,None]*D2+k[:,None]*D1
        Qj=Jop.T@Jop/(len(U)*73.27969652156213**6)
        obj+=weight*sum(cp.quad_form(C[:,i],cp.psd_wrap(Qj)) for i in range(7))
    prob=cp.Problem(cp.Minimize(obj),con);prob.solve(solver='CLARABEL',max_iter=200,tol_feas=1e-10,tol_gap_abs=1e-10)
    audit.update(profile_status=prob.status,profile_iterations=prob.solver_stats.num_iters)
    if prob.status not in ('optimal','optimal_inaccurate') or free.value is None:raise ValueError('PROFILE_QP_'+str(prob.status))
    cf=np.vstack([first,free.value,fixed_end]);audit['profile_controls']=cf.tolist()
    audit['profile_constraint_violation_max']=float(max(np.max(np.asarray(c.violation())) for c in con))
    theta=QuinticSpline(cf[:,:1]);arm=QuinticSpline(cf[:,1:])
    R=np.unique(np.r_[np.linspace(a,b,501),s[(s>=a)&(s<=b)]]);A=integral_matrix(R,a,b,theta,H); rfirst=_fixed_controls(14,np.array([rho0,L*ru,L*L*ruu]),3)
    audit['rho_entry_controls']=rfirst.tolist()
    if np.min(rfirst)<0:raise ValueError('ENTRY_NONNEGATIVE_CONFLICT')
    x=cp.Variable(8,nonneg=True);rc=cp.hstack([rfirst,x,np.zeros(3)]);pred=cp.hstack([cp.reshape(A[:,k,:]@rc,(len(R),1),order='C') for k in range(2)]);p0=st.q[0,:2]; refxy=_interp(s,q,R)[:,:2]-p0;target=old_stationary.p0-p0
    cons=[pred-refxy<=.1,refxy-pred<=.1,pred[-1,:]==target]
    oldxy=original.evaluate(R).q[:,:2]-p0
    Qr=sum(A[:,k,:].T@A[:,k,:] for k in range(2))/len(R)
    rho_target=refxy if mode=='B' else oldxy
    lr=sum(A[:,k,:].T@rho_target[:,k] for k in range(2))/len(R)
    objective=cp.quad_form(rc,cp.psd_wrap(Qr))-2*lr@rc
    if mode=='B':
        F,Axy,Jxy,alpha=physical_rho_operators(R,a,b,theta,H)
        T=73.27969652156213; qa=sum(m.T@m for m in Axy)/(len(R)*T**4*.5**2);qj=sum(m.T@m for m in Jxy)/(len(R)*T**6)
        z=cp.Variable(nonneg=True);wheelcap=np.asarray(limits.wheel_acceleration.value)
        for sign,j in ((-1,0),(1,1)):
            wheel=(F@rc+sign*(float(limits.wheel_separation.value)/2)*alpha)/float(limits.wheel_radius.value)
            cons.extend([wheel<=wheelcap[j]*T**2*z,-wheel<=wheelcap[j]*T**2*z])
        for j in range(2):
            cons.extend([Axy[j]@rc<=limits.xy_component_acceleration.value[j]*T**2*z,-Axy[j]@rc<=limits.xy_component_acceleration.value[j]*T**2*z])
        cons.extend([F@rc<=float(limits.tangential_acceleration.value)*T**2*z,-F@rc<=float(limits.tangential_acceleration.value)*T**2*z])
        objective=z+.1*(cp.quad_form(rc,cp.psd_wrap(Qr))-2*lr@rc)/.1**2+.1*cp.quad_form(rc,cp.psd_wrap(qa))+.1*cp.quad_form(rc,cp.psd_wrap(qj))
    problem=cp.Problem(cp.Minimize(objective),cons);problem.solve(solver='CLARABEL',max_iter=200,tol_feas=1e-10,tol_gap_abs=1e-10)
    audit.update(rho_status=problem.status,rho_iterations=problem.solver_stats.num_iters)
    if problem.status not in ('optimal','optimal_inaccurate') or x.value is None:
        audit['rho_constraint_residual']='UNAVAILABLE_INFEASIBLE_OR_NO_SOLUTION'
        raise ValueError('RHO_QP_'+str(problem.status))
    audit['rho_constraint_violation_max']=float(max(np.max(np.asarray(c.violation())) for c in cons))
    if np.min(x.value)<0:raise ValueError('RHO_NUMERICAL_NEGATIVE '+repr(float(np.min(x.value))))
    rho=QuinticSpline(np.r_[rfirst,x.value,np.zeros(3)][:,None])
    for lo,hi in zip(np.unique(rho.knots)[:-1],np.unique(rho.knots)[1:]):
        mid=(lo+hi)/2; active=quintic_basis([mid],14)[0]>0
        if not np.any(rho.controls[:,0][active]>0):raise ValueError('NEW_STATIONARY_INTERVAL_UNSUPPORTED')
    moving=ExplicitChart(a,b,p0,rho,theta,arm,False,(a,1.));end=moving.evaluate_global([b]).q[0,:2]
    stationary=ExplicitChart(b,1.,end,QuinticSpline(np.zeros((8,1))),theta,arm,True,(a,1.));block=MotionBlock('block_000',(reg,moving,stationary),original.schema)
    errors={};
    for i,(left,right) in enumerate(zip(block.charts[:-1],block.charts[1:])):
        ll=left.evaluate_global([left.s1]);rr=right.evaluate_global([right.s0]);errors[str(i)]={key:float(np.max(np.abs(getattr(ll,key)-getattr(rr,key)))) for key in ('q','qs','qss','qsss')}
        if max(errors[str(i)].values())>1e-6:raise ValueError('C3_SEAM '+repr(errors))
    if np.max(abs(block.start-path.q[0]))>1e-6 or np.max(abs(block.end-path.q[-1]))>1e-6:raise ValueError('ENDPOINT_MISMATCH')
    for chart in block.charts:
        us=np.unique(np.r_[np.linspace(chart.s0,chart.s1,2001),s[(s>=chart.s0)&(s<=chart.s1)]])
        jet=chart.evaluate_global(us);d=np.abs(jet.q-_interp(s,q,us));
        if np.max(d[:,:2])>.1 or np.max(d[:,2])>np.pi/2 or np.max(d[:,3:])>.1:raise ValueError('REFERENCE_TUBE_SAMPLED '+repr(np.max(d,axis=0).tolist()))
        lat=-np.sin(jet.q[:,2])*jet.qs[:,0]+np.cos(jet.q[:,2])*jet.qs[:,1]
        if np.max(abs(lat))>1e-8:raise ValueError('NONHOLONOMIC_RESIDUAL')
        if np.any(np.abs(np.diff(jet.q[:,2]))>=np.pi):raise ValueError('THETA_WINDING')
    # Gap: continuous tube enclosure is mandatory for accepting A.
    return block,{'seams':errors,'profile_status':prob.status,'rho_status':problem.status,'entry':a,'exit':b,'continuous_tube':'UNVERIFIED','profile_objective':float(prob.value),'rho_objective':float(problem.value),'mode':mode,'weight':float(weight),'profile_iterations':int(prob.solver_stats.num_iters),'rho_iterations':int(problem.solver_stats.num_iters)}


def reference_tube_extrema(block,path):
    """Numerical stationary-point search, not a rigorous interval certificate."""
    from scipy.optimize import brentq
    q,s,_=nine_dimensional_progress(path.q)
    maxima=np.zeros(9); roots=0
    for chart in block.charts:
        # Parent interpolation knots plus dense scan; include each chart boundary.
        grid=np.unique(np.r_[np.linspace(chart.s0,chart.s1,4001),s[(s>=chart.s0)&(s<=chart.s1)]])
        jet=chart.evaluate_global(grid);ref=_interp(s,q,grid)
        maxima=np.maximum(maxima,np.max(abs(jet.q-ref),axis=0))
        mid=(grid[:-1]+grid[1:])/2; idx=np.clip(np.searchsorted(s,mid,side='right')-1,0,len(s)-2)
        slopes=(q[idx+1]-q[idx])/(s[idx+1]-s[idx])[:,None]
        # With parent kink endpoints inserted, each interval uses one slope.
        left=jet.qs[:-1]-slopes;right=jet.qs[1:]-slopes
        for i,k in np.argwhere(left*right<0):
            slope=slopes[i,k]
            def derivative(u): return float(chart.evaluate_global([u]).qs[0,k]-slope)
            root=brentq(derivative,grid[i],grid[i+1],xtol=1e-14)
            error=abs(chart.evaluate_global([root]).q[0,k]-np.interp(root,s,q[:,k]))
            maxima[k]=max(maxima[k],error);roots+=1
    caps=np.r_[.1,.1,np.pi/2,np.full(6,.1)]
    if np.any(maxima>caps):raise ValueError('REFERENCE_TUBE_EXTREMA '+repr(maxima.tolist()))
    return {'status':'NUMERICAL_CHECKED_NOT_CERTIFIED','peak':maxima.tolist(),'roots':roots,'grid_per_chart':4001}
