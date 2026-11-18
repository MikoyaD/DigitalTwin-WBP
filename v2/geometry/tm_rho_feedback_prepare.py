"""Independent candidate preparation; adapted from v3 tm_local.py:13-72.

No QP results from this module may be published as an H geometry.
"""
import time
import numpy as np
import cvxpy as cp
from .tm_modes import classify
from .tm_builder import _basis,_rho_matrix
from .unified_chart import ExplicitChart,fit_regular
from .unified_block import MotionBlock,_reference_at
from ..core.math_tools import QuinticSpline
from ..core.contracts import require


def prepare_candidate(path,limits,fraction,seed=0):
    start=time.perf_counter();q,u,source,modes,eps=classify(path)
    segments=[]
    for i,mode in enumerate(modes):
        a,b=mode['u0'],mode['u1'];h=b-a
        if mode['kind']=='ADJUST':
            segments.append(dict(kind='ADJUST',a=a,b=b,mode=i));continue
        left=fraction*h if i>0 and modes[i-1]['kind']=='ADJUST' else 0.
        right=fraction*h if i+1<len(modes) and modes[i+1]['kind']=='ADJUST' else 0.
        if left+right>=h*(1-1e-12):
            segments.append(dict(kind='EXPLICIT',a=a,b=b,mode=i));continue
        if left:segments.append(dict(kind='EXPLICIT',a=a,b=a+left,mode=i))
        segments.append(dict(kind='REGULAR',a=a+left,b=b-right,mode=i))
        if right:segments.append(dict(kind='EXPLICIT',a=b-right,b=b,mode=i))
    # Fit regular interiors only; all source points still participate in the final tube check.
    for seg in segments:
        a,b=seg['a'],seg['b']
        if seg['kind']=='REGULAR':
            s=np.unique(np.r_[a,b,u[(u>a)&(u<b)],np.linspace(a,b,41)])
            seg['chart']=fit_regular(_reference_at(u,q,s),s,limits)
    m=18;obj=0.;cons=[];variables={};lo=limits.arm_lower.value;hi=limits.arm_upper.value
    for i,seg in enumerate(segments):
        if seg['kind']=='REGULAR':continue
        a,b=seg['a'],seg['b'];h=b-a;c=cp.Variable((m,7));variables[i]=c
        points=np.unique(np.r_[np.linspace(a,b,101),u[(u>=a)&(u<=b)]])
        x=(points-a)/h;B=_basis(x,m);reference=_reference_at(u,q,points)[:,2:];target=reference.copy()
        if seed and seg['kind']=='EXPLICIT':
            displacement=_reference_at(u,q,[b])[0,:2]-_reference_at(u,q,[a])[0,:2]
            if np.linalg.norm(displacement)>1e-12:
                chord=np.arctan2(displacement[1],displacement[0]);mid=float(reference[len(reference)//2,0])
                delta=np.arctan2(np.sin(chord-mid),np.cos(chord-mid))
                target[:,0]+=np.clip(delta,-.1,.1)*np.sin(np.pi*x)**2
        bound=np.tile(np.r_[np.pi/2,np.full(6,.0999)],(len(points),1))
        cons += [B@c-reference<=bound,B@c-reference>=-bound,
                 c[:,1:]>=np.tile(lo,(m,1)),c[:,1:]<=np.tile(hi,(m,1))]
        obj+=cp.sum_squares(B@c-target)/len(points)
        # Local-scale regularization keeps very short mode charts numerically conditioned.
        # It is geometry smoothing only, not a physical jerk objective or limit.
        obj+=1e-8*cp.sum_squares(_basis(np.linspace(0,1,41),m,3)@c)/41
        if i==0:cons += [c[0]==q[0,2:]]
        if i==len(segments)-1:cons += [c[-1]==q[-1,2:]]
        if i and segments[i-1]['kind']=='REGULAR':
            jet=segments[i-1]['chart'].evaluate_global([a])
            for order,name in enumerate(('q','qs','qss')):
                cons += [(_basis([0.],m,order)@c)[0]==h**order*getattr(jet,name)[0,2:]]
        if i+1<len(segments) and segments[i+1]['kind']=='REGULAR':
            jet=segments[i+1]['chart'].evaluate_global([b])
            for order,name in enumerate(('q','qs','qss')):
                cons += [(_basis([1.],m,order)@c)[0]==h**order*getattr(jet,name)[0,2:]]
        if i and segments[i-1]['kind']!='REGULAR':
            prev=segments[i-1];hp=prev['b']-prev['a'];cpv=variables[i-1]
            for order in range(3):
                scale=min(h,hp)**order
                cons += [(_basis([1.],m,order)@cpv)*(scale/hp**order)==
                         (_basis([0.],m,order)@c)*(scale/h**order)]
    if variables:
        prob=cp.Problem(cp.Minimize(obj),cons)
        prob.solve(solver='CLARABEL',max_iter=200,tol_gap_abs=1e-9,tol_feas=1e-9)
        require(prob.status in ('optimal','optimal_inaccurate') and all(v.value is not None for v in variables.values()),
                'TM_LOCAL_PROFILE_QP',str(prob.status))
    return dict(q=q, u=u, source=source, modes=modes, eps=eps, segments=segments, profiles={i:np.asarray(v.value).copy() for i,v in variables.items()}, fraction=fraction, seed=seed, prepare_s=time.perf_counter()-start)


def make_problem(prepared):
    """Assemble fixed candidate charts, reference tubes, and exact boundary jets.

    This is an input assembly function, not an H candidate or an acceptance.
    """
    q,u,modes,segs = (prepared[k] for k in ('q','u','modes','segments'))
    profiles=prepared['profiles']; ids=sorted(profiles); order={k:i for i,k in enumerate(ids)}
    theta_profiles=[]; rows=[]; rhs=[]; width=18*len(ids)
    def equation(parts, value):
        row=np.zeros(width)
        for idx,vector in parts: row[18*order[idx]:18*(order[idx]+1)]+=vector
        rows.append(row);rhs.append(value)
    for i in ids:
        seg=segs[i];a,b=seg['a'],seg['b'];h=b-a
        points=np.unique(np.r_[np.linspace(a,b,101),u[(u>=a)&(u<=b)]])
        B=_basis((points-a)/h,18)
        theta_profiles.append(dict(beta=profiles[i][:,0].copy(),initial=profiles[i][:,0].copy(),basis=B,
                                   reference=_reference_at(u,q,points)[:,2].copy()))
        if i==0:equation([(i,_basis([0.],18)[0])],float(q[0,2]))
        if i==len(segs)-1:equation([(i,_basis([1.],18)[0])],float(q[-1,2]))
        if i and segs[i-1]['kind']=='REGULAR':
            jet=segs[i-1]['chart'].evaluate_global([a])
            for r,name in enumerate(('q','qs','qss')):
                equation([(i,_basis([0.],18,r)[0]/h**r)],float(getattr(jet,name)[0,2]))
        if i+1<len(segs) and segs[i+1]['kind']=='REGULAR':
            jet=segs[i+1]['chart'].evaluate_global([b])
            for r,name in enumerate(('q','qs','qss')):
                equation([(i,_basis([1.],18,r)[0]/h**r)],float(getattr(jet,name)[0,2]))
        if i and segs[i-1]['kind']!='REGULAR':
            hp=segs[i-1]['b']-segs[i-1]['a']
            for r in range(3):
                equation([(i-1,_basis([1.],18,r)[0]/hp**r),(i,-_basis([0.],18,r)[0]/h**r)],0.)
    moves=[]
    for i,seg in enumerate(segs):
        if seg['kind']!='EXPLICIT': continue
        a,b=seg['a'],seg['b'];h=b-a
        def boundary(side):
            idx=i-1 if side=='left' else i+1;where=a if side=='left' else b
            if idx<0 or idx>=len(segs):return _reference_at(u,q,[where])[0,:2],None,'TASK'
            other=segs[idx]
            if other['kind']=='REGULAR':
                jet=other['chart'].evaluate_global([where]);return jet.q[0,:2],jet,'REGULAR'
            if other['kind']=='ADJUST':return np.asarray(modes[other['mode']]['anchor']),None,'ADJUST'
            raise ValueError('TM_UNEXPECTED_ADJACENT_EXPLICIT')
        p0,left,leftkind=boundary('left');p1,right,rightkind=boundary('right')
        points=np.unique(np.r_[a,b,np.linspace(a,b,101),u[(u>a)&(u<b)]])
        x=(points-a)/h; theta=QuinticSpline(profiles[i][:,:1]);A=_rho_matrix(x,theta,14,h)
        J=[];d=[]
        for side,jet,kind in sorted(((0.,left,leftkind),(1.,right,rightkind)), key=lambda item: 0 if item[2]=='ADJUST' else 1):
            if kind=='REGULAR':
                p=jet.qs[0,:2];pp=jet.qss[0,:2];speed=np.linalg.norm(p)
                require(speed>1e-9,'TM_REGULAR_TANGENT','zero')
                J.extend([_basis([side],14)[0],_basis([side],14,1)[0]])
                d.extend([speed,h*np.dot(p,pp)/speed])
            elif kind=='ADJUST':
                J.extend([_basis([side],14)[0],_basis([side],14,1)[0]]);d.extend([0.,0.])
        moves.append(dict(segment=i,profile=order[i],x=x,h=h,p0=p0,p1=p1,A=A,
                          ref=_reference_at(u,q,points)[:,:2]-p0,
                          J=np.asarray(J).reshape(-1,14),d=np.asarray(d),
                          neighbours=(leftkind,rightkind)))
    return dict(profiles=theta_profiles,moves=moves,theta_eq=np.asarray(rows).reshape(-1,width),theta_rhs=np.asarray(rhs),profile_ids=ids)
