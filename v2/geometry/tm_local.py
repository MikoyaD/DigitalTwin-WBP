"""Bounded regular-core/explicit-window TM candidate. Fixed parent, no ROS."""
import time
import numpy as np
import cvxpy as cp
from .tm_modes import classify
from .tm_builder import _basis,_rho_matrix
from .unified_chart import ExplicitChart,fit_regular
from .unified_block import MotionBlock,_reference_at
from ..core.math_tools import QuinticSpline
from ..core.contracts import require


def build_local(path,limits,fraction,seed=0):
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
    charts=[]
    for i,seg in enumerate(segments):
        if seg['kind']=='REGULAR':charts.append(seg['chart']);continue
        a,b=seg['a'],seg['b'];h=b-a;v=np.asarray(variables[i].value)
        theta=QuinticSpline(v[:,:1]);arm=QuinticSpline(v[:,1:]);mode=modes[seg['mode']]
        if seg['kind']=='ADJUST':
            require(np.max(np.ptp(v,axis=0))>1e-10,'ADJUST_NO_MOTION','constant')
            charts.append(ExplicitChart(a,b,np.asarray(mode['anchor']),QuinticSpline(np.zeros((6,1))),theta,arm,True));continue
        # Explicit MOVE bridge between immutable regular or stationary anchors.
        def endpoint(side):
            other=i-1 if side=='left' else i+1;coord=a if side=='left' else b
            if other<0 or other>=len(segments):return _reference_at(u,q,[coord])[0,:2],None
            neighbour=segments[other]
            if neighbour['kind']=='REGULAR':
                jet=neighbour['chart'].evaluate_global([coord]);return jet.q[0,:2],jet
            if neighbour['kind']=='ADJUST':return np.asarray(modes[neighbour['mode']]['anchor']),None
            raise ValueError('TM_UNEXPECTED_ADJACENT_EXPLICIT')
        p0,leftjet=endpoint('left');p1,rightjet=endpoint('right')
        n=14;points=np.unique(np.r_[a,b,np.linspace(a,b,101),u[(u>a)&(u<b)]])
        x=(points-a)/h;A=_rho_matrix(x,theta,n,h);c=cp.Variable(n)
        pred=cp.hstack([cp.reshape(A[:,d,:]@c,(len(x),1),order='C') for d in range(2)])
        ref=_reference_at(u,q,points)[:,:2]-p0
        constraints=[c>=0,pred[-1,:]==p1-p0,pred-ref<=.0999,pred-ref>=-.0999]
        for side,jet in (('left',leftjet),('right',rightjet)):
            neighbour=i-1 if side=='left' else i+1
            xend=0. if side=='left' else 1.
            if jet is not None:
                p=jet.qs[0,:2];pp=jet.qss[0,:2];speed=np.linalg.norm(p)
                require(speed>1e-9,'TM_REGULAR_TANGENT','zero')
                constraints += [(_basis([xend],n)@c)[0]==speed,
                                (_basis([xend],n,1)@c)[0]==h*np.dot(p,pp)/speed]
            elif 0<=neighbour<len(segments) and segments[neighbour]['kind']=='ADJUST':
                constraints += [(_basis([xend],n)@c)[0]==0,(_basis([xend],n,1)@c)[0]==0]
        d2=_basis(np.linspace(0,1,41),n,2)
        problem=cp.Problem(cp.Minimize(cp.sum_squares(pred-ref)/len(x)+1e-6*cp.sum_squares(d2@c)/41),constraints)
        problem.solve(solver='CLARABEL',max_iter=200,tol_gap_abs=1e-10,tol_feas=1e-10)
        require(problem.status in ('optimal','optimal_inaccurate') and c.value is not None,
                'TM_LOCAL_RHO_QP',str(problem.status)+' segment='+str(i)+' theta_range='+repr([float(np.min(theta.evaluate(np.linspace(0,1,101))[:,0])),float(np.max(theta.evaluate(np.linspace(0,1,101))[:,0]))])+' displacement='+repr((p1-p0).tolist()))
        rho=np.asarray(c.value);require(np.min(rho)>=-1e-9,'TM_LOCAL_RHO_NEGATIVE',repr(np.min(rho)))
        rho=np.maximum(rho,0.)
        if i and segments[i-1]['kind']=='ADJUST':rho[:2]=0.
        if i+1<len(segments) and segments[i+1]['kind']=='ADJUST':rho[-2:]=0.
        rs=QuinticSpline(rho[:,None]);knot=rs.knots;grid=np.unique(np.r_[knot,(knot[:-1]+knot[1:])/2]);grid=grid[(grid>0)&(grid<1)]
        require(np.min(rs.evaluate(grid)[:,0])>0,'MOVE_INTERIOR_STATIONARY',str(i))
        chart=ExplicitChart(a,b,p0,rs,theta,arm,False)
        require(np.max(abs(chart.evaluate_global([b]).q[0,:2]-p1))<1e-6,'TM_LOCAL_ENDPOINT','integral')
        test=np.linspace(0,1,5)
        require(np.max(abs(chart.positions(test)-np.vstack([chart._xy(float(x)) for x in test])))<1e-8,'TM_QUADRATURE','quad')
        charts.append(chart)
    block=MotionBlock('tm_000',tuple(charts),path.schema)
    require(np.max(abs(block.start-q[0]))<1e-6 and np.max(abs(block.end-q[-1]))<1e-6,'TM_ENDPOINT','parent')
    worst=np.zeros(3)
    for c in charts:
        grid=np.unique(np.r_[np.linspace(c.s0,c.s1,1001),u[(u>=c.s0)&(u<=c.s1)]])
        jet=c.evaluate_global(grid);ref=_reference_at(u,q,grid);d=np.max(abs(jet.q-ref),axis=0)
        worst=np.maximum(worst,[max(d[:2]),d[2],max(d[3:])])
        lat=-np.sin(jet.q[:,2])*jet.qs[:,0]+np.cos(jet.q[:,2])*jet.qs[:,1]
        forward=np.cos(jet.q[:,2])*jet.qs[:,0]+np.sin(jet.q[:,2])*jet.qs[:,1]
        require(max(abs(lat))<=1e-8 and min(forward)>=-1e-9,'TM_NONHOLONOMIC','geometry')
        require(np.all(jet.q[:,3:]>=lo-1e-6) and np.all(jet.q[:,3:]<=hi+1e-6),'TM_POSITION_LIMIT','arm')
    require(worst[0]<=.1 and worst[1]<=np.pi/2 and worst[2]<=.1,'TM_REFERENCE_TUBE',repr(worst.tolist()))
    return block,dict(modes=modes,source_indices=source.tolist(),progress=u.tolist(),epsilon_xy=eps,
        reference_tube_max=worst.tolist(),candidate='local_regular_%g_seed%d'%(fraction,seed),h_build_s=time.perf_counter()-start,
        scope='sampled; collision not checked',regularization='local-scale third derivative')
