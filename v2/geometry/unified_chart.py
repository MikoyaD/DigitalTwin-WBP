"""Contract-v2 nonholonomic charts.  Independent of ROS and legacy phases.

The explicit chart stores rho=|dp/ds| and heading/arm quintic coefficients on
its local coordinate xi.  Position is the integral of rho*heading, never an
endpoint correction.  The v2 contract deliberately records numerical, rather
than interval-certified, quadrature accuracy.
"""
from dataclasses import dataclass
import numpy as np
from scipy.integrate import quad
import cvxpy as cp
from ..core.contracts import PathJet, ContractError, require
from ..core.math_tools import QuinticSpline, quintic_basis
from .spline_builder import LiftedSpline


@dataclass(frozen=True)
class ExplicitChart:
    s0: float
    s1: float
    p0: np.ndarray
    rho: QuinticSpline
    theta: QuinticSpline
    arm: QuinticSpline
    stationary: bool = False
    profile_interval: tuple = None

    def __post_init__(self):
        require(0 <= self.s0 < self.s1 <= 1, 'CHART_RANGE', 'global s interval')
        require(np.asarray(self.p0).shape == (2,), 'CHART_DIMENSION', 'p0')
        require(self.rho.controls.shape[1] == 1 and self.theta.controls.shape[1] == 1
                and self.arm.controls.shape[1] == 6, 'CHART_DIMENSION', 'rho/theta/arm')
        if self.stationary:
            require(np.max(np.abs(self.rho.controls)) == 0, 'STATIONARY_RHO', 'rho must vanish identically')
        else:
            require(np.min(self.rho.controls) >= 0, 'REVERSE_RHO', 'control hull must be nonnegative')

    def _xy(self, xi):
        if self.stationary or xi == 0:
            return np.asarray(self.p0, float).copy()
        def component(k):
            def fun(z):
                speed = self.rho.evaluate(np.array([z]))[0, 0]
                profile=self.profile_interval or (self.s0,self.s1)
                global_s=self.s0+(self.s1-self.s0)*z
                xp=(global_s-profile[0])/(profile[1]-profile[0])
                angle = self.theta.evaluate(np.array([xp]))[0, 0]
                return speed * (np.cos(angle) if k == 0 else np.sin(angle))
            profile=self.profile_interval or (self.s0,self.s1)
            transformed=(profile[0]+self.theta.knots*(profile[1]-profile[0])-self.s0)/(self.s1-self.s0)
            knots = np.unique(np.r_[self.rho.knots, transformed])
            cuts = knots[(knots > 0) & (knots < xi)]
            points = np.r_[0., cuts, xi]
            return sum(quad(fun, a, b, epsabs=2e-11, epsrel=2e-11)[0]
                       for a, b in zip(points[:-1], points[1:]))
        return np.asarray(self.p0, float) + (self.s1-self.s0)*np.array([component(0), component(1)])

    def positions(self, xi):
        """Vectorized 24-point Gauss panels; independent scipy.quad is an audit."""
        xi=np.atleast_1d(np.asarray(xi,float));p0=np.asarray(self.p0,float)
        if self.stationary:return np.tile(p0,(len(xi),1))
        profile=self.profile_interval or (self.s0,self.s1)
        transformed=(profile[0]+self.theta.knots*(profile[1]-profile[0])-self.s0)/(self.s1-self.s0)
        cuts=np.unique(np.r_[self.rho.knots,transformed,0.,1.]);cuts=cuts[(cuts>=0)&(cuts<=1)]
        nodes,weights=np.polynomial.legendre.leggauss(24)
        def integrate(a,b):
            a=np.asarray(a,float);b=np.asarray(b,float);h=b-a
            v=(a+b)[:,None]/2+h[:,None]*nodes[None,:]/2
            local=np.clip(v.ravel(),0,1)
            speed=self.rho.evaluate(local)[:,0].reshape(v.shape)
            xp=(self.s0+(self.s1-self.s0)*local-profile[0])/(profile[1]-profile[0])
            angle=self.theta.evaluate(np.clip(xp,0,1))[:,0].reshape(v.shape)
            directions=np.stack((np.cos(angle),np.sin(angle)),axis=2)
            return h[:,None]/2*np.einsum('j,ij,ijk->ik',weights,speed,directions)
        segment=integrate(cuts[:-1],cuts[1:]);prefix=np.vstack((np.zeros(2),np.cumsum(segment,axis=0)))
        indices=np.searchsorted(cuts,xi,side='right')-1;indices=np.clip(indices,0,len(cuts)-2)
        partial=integrate(cuts[indices],xi)
        return p0+(self.s1-self.s0)*(prefix[indices]+partial)

    def evaluate_global(self, s):
        s = np.atleast_1d(np.asarray(s, float))
        require(np.all(np.isfinite(s)) and np.all((s >= self.s0-1e-14) & (s <= self.s1+1e-14)),
                'CHART_RANGE', 'query outside chart')
        xi = np.clip((s-self.s0)/(self.s1-self.s0), 0., 1.)
        length = self.s1-self.s0
        rho = [self.rho.evaluate(xi, k)[:, 0]/length**k for k in range(3)]
        profile=self.profile_interval or (self.s0,self.s1)
        xp=np.clip((s-profile[0])/(profile[1]-profile[0]),0.,1.)
        lp=profile[1]-profile[0]
        theta = [self.theta.evaluate(xp, k)[:, 0]/lp**k for k in range(4)]
        arm = [self.arm.evaluate(xp, k)/lp**k for k in range(4)]
        e = np.column_stack((np.cos(theta[0]), np.sin(theta[0])))
        ep = np.column_stack((-np.sin(theta[0]), np.cos(theta[0])))
        p = self.positions(xi)
        p1 = rho[0][:, None]*e
        p2 = rho[1][:, None]*e + (rho[0]*theta[1])[:, None]*ep
        p3 = (rho[2]-rho[0]*theta[1]**2)[:, None]*e + (2*rho[1]*theta[1]+rho[0]*theta[2])[:, None]*ep
        return PathJet(np.column_stack((p, theta[0], arm[0])),
                       np.column_stack((p1, theta[1], arm[1])),
                       np.column_stack((p2, theta[2], arm[2])),
                       np.column_stack((p3, theta[3], arm[3])))

    def freeze(self):
        return dict(type='EXPLICIT_HEADING_INTEGRAL', s_interval=[self.s0,self.s1],
                    p0=np.asarray(self.p0).tolist(),stationary=self.stationary,
                    rho_controls=self.rho.controls.tolist(),rho_knots=self.rho.knots.tolist(),
                    theta_controls=self.theta.controls.tolist(),theta_knots=self.theta.knots.tolist(),
                    arm_controls=self.arm.controls.tolist(),arm_knots=self.arm.knots.tolist(),
                    profile_interval=list(self.profile_interval or (self.s0,self.s1)),
                    evaluation='scipy_quad_double_NUMERICAL_CHECKED')

    @classmethod
    def thaw(cls, item):
        require(item['type'] == 'EXPLICIT_HEADING_INTEGRAL', 'CHART_TYPE', 'explicit chart')
        return cls(*item['s_interval'],np.asarray(item['p0'],float),
                   QuinticSpline(item['rho_controls'],item['rho_knots']),
                   QuinticSpline(item['theta_controls'],item['theta_knots']),
                   QuinticSpline(item['arm_controls'],item['arm_knots']),item['stationary'],
                   tuple(item.get('profile_interval',item['s_interval'])))


def nine_dimensional_progress(q):
    """One dimensionless progress for XY, theta, and all six arms."""
    q = np.asarray(q,float)
    require(q.ndim == 2 and q.shape[1] == 9 and len(q)>1 and np.isfinite(q).all(),
            'P_DIMENSION', '(N,9) finite')
    jumps=np.diff(q[:,2])
    require(np.all(np.abs(jumps)<np.pi), 'AMBIGUOUS_WINDING', 'theta steps >=pi need explicit lift')
    keep=np.r_[True,np.any(np.diff(q,axis=0)!=0,axis=1)]
    source_indices=np.flatnonzero(keep)
    states=q[keep]
    require(len(states)>1,'NO_MOTION','single unique state')
    delta=np.linalg.norm(np.diff(states,axis=0),axis=1)
    require(np.all(delta>0),'PROGRESS','strictly positive after duplicate removal')
    progress=np.r_[0.,np.cumsum(delta)]/np.sum(delta)
    progress[-1]=1.
    return states,progress,source_indices


def fit_stationary(q, s, limits, *, controls=10):
    """Convex shared-progress theta+arm fit for a truly fixed-XY source interval.

    This chart may carry simultaneous angular and joint derivatives.  It does
    not assert zero derivatives at its entry; a preceding transition must match
    its actual jets before this chart can be included in a MotionBlock.
    """
    q=np.asarray(q,float);s=np.asarray(s,float)
    require(q.ndim==2 and q.shape[1]==9 and len(q)>=2 and len(s)==len(q),
            'STATIONARY_INPUT','matching nine-state and progress arrays')
    # MoveIt serialization exhibits a few ulps of XY round-off on the fixed
    # tail.  This is not a general near-zero-motion classifier.
    roundoff=4*np.spacing(max(1.,float(np.max(np.abs(q[:,:2])))))
    require(np.max(np.abs(q[:,:2]-q[0,:2]))<=roundoff,'STATIONARY_XY',
            'source XY differs by more than serialization round-off')
    require(np.all(np.diff(s)>0),'PROGRESS','strictly increasing')
    xi=(s-s[0])/(s[-1]-s[0]);m=max(6,int(controls))
    B=quintic_basis(xi,m);D3=quintic_basis(np.linspace(0,1,101),m,3)
    c=cp.Variable((m,7));target=q[:,2:]
    w=np.r_[xi[1]-xi[0],xi[2:]-xi[:-2],xi[-1]-xi[-2]]/2
    objective=cp.sum_squares(cp.multiply(np.sqrt(w)[:,None],B@c-target))
    objective+=1e-4*cp.sum_squares(D3@c)/101
    constraints=[c[0]==target[0],c[-1]==target[-1],
                 c[:,1:]>=np.tile(limits.arm_lower.value,(m,1)),
                 c[:,1:]<=np.tile(limits.arm_upper.value,(m,1)),
                 B@c-target<=.1,B@c-target>=-.1]
    problem=cp.Problem(cp.Minimize(objective),constraints)
    problem.solve(solver='CLARABEL',max_iter=150,tol_gap_abs=1e-10,tol_feas=1e-10)
    require(problem.status in ('optimal','optimal_inaccurate') and c.value is not None,
            'STATIONARY_QP_FAILED',problem.status)
    fitted=np.asarray(c.value);fitted[0]=target[0];fitted[-1]=target[-1]
    require(np.max(np.abs(B@fitted-target))<=.1000001,'STATIONARY_TUBE','node deviation')
    return ExplicitChart(float(s[0]),float(s[-1]),q[0,:2].copy(),
                         QuinticSpline(np.zeros((m,1))),
                         QuinticSpline(fitted[:,:1]),QuinticSpline(fitted[:,1:]),True)


@dataclass(frozen=True)
class RegularChart:
    s0: float
    s1: float
    spline: QuinticSpline
    heading_anchor: float

    def __post_init__(self):
        require(0<=self.s0<self.s1<=1,'CHART_RANGE','regular chart interval')
        object.__setattr__(self,'lift',LiftedSpline(self.spline,self.heading_anchor))

    def evaluate_global(self,s):
        s=np.atleast_1d(np.asarray(s,float))
        require(np.all((s>=self.s0-1e-14)&(s<=self.s1+1e-14)),'CHART_RANGE','regular query')
        xi=np.clip((s-self.s0)/(self.s1-self.s0),0,1)
        jet=self.lift(xi);L=self.s1-self.s0
        return PathJet(jet.q,jet.qs/L,jet.qss/L**2,jet.qsss/L**3)

    def freeze(self):
        return dict(type='REGULAR_XY_QUINTIC',s_interval=[self.s0,self.s1],
                    controls_8d=self.spline.controls.tolist(),knots=self.spline.knots.tolist(),
                    heading_anchor=self.heading_anchor)

    @classmethod
    def thaw(cls,item):
        require(item['type']=='REGULAR_XY_QUINTIC','CHART_TYPE','regular chart')
        return cls(*item['s_interval'],QuinticSpline(item['controls_8d'],item['knots']),item['heading_anchor'])


def fit_regular(q,s,limits,*,controls=32):
    """Shared-progress 8D convex QP.  Direction equalities are not speed guesses."""
    q=np.asarray(q,float);s=np.asarray(s,float)
    require(len(q)==len(s) and len(q)>2 and np.all(np.diff(s)>0),'REGULAR_INPUT','progress')
    xi=(s-s[0])/(s[-1]-s[0]);m=max(8,min(32,controls));B=quintic_basis(xi,m)
    z=q[:,[0,1,3,4,5,6,7,8]];c=cp.Variable((m,8));v0=cp.Variable(nonneg=True);v1=cp.Variable(nonneg=True)
    dirs=np.column_stack((np.cos(q[[0,-1],2]),np.sin(q[[0,-1],2])))
    w=np.r_[xi[1]-xi[0],xi[2:]-xi[:-2],xi[-1]-xi[-2]]/2
    nodes=np.linspace(0,1,201);D3=quintic_basis(nodes,m,3)
    objective=cp.sum_squares(cp.multiply(np.sqrt(w)[:,None],B@c-z))+1e-4*cp.sum_squares(D3@c)/len(nodes)
    # A small inner numerical margin keeps the *actual* interpolated curve
    # inside the unchanged 0.1 tube, which is checked again after solving.
    constraints=[c[0]==z[0],c[-1]==z[-1],B@c-z<=.0998,B@c-z>=-.0998,
                 c[:,2:]>=np.tile(limits.arm_lower.value,(m,1)),
                 c[:,2:]<=np.tile(limits.arm_upper.value,(m,1)),
                 (quintic_basis([0.],m,1)@c)[0,:2]==v0*dirs[0],
                 (quintic_basis([1.],m,1)@c)[0,:2]==v1*dirs[1],v0>=1e-5,v1>=1e-5]
    problem=cp.Problem(cp.Minimize(objective),constraints)
    problem.solve(solver='CLARABEL',max_iter=150,tol_gap_abs=1e-9,tol_feas=1e-9)
    require(problem.status in ('optimal','optimal_inaccurate') and c.value is not None,
            'REGULAR_QP_FAILED',problem.status)
    fitted=np.asarray(c.value);fitted[0]=z[0];fitted[-1]=z[-1]
    require(np.max(np.abs(B@fitted-z))<=.100001,'REGULAR_TUBE','node deviation')
    return RegularChart(float(s[0]),float(s[-1]),QuinticSpline(fitted),float(q[0,2]))


def fit_tail_profile(q,s,regular,limits,*,controls=14,heading_dip=0.,stationary_index=None,smoothing=1e-4):
    """One theta/arm spline across moving-to-stationary behavior; no mode stop."""
    q=np.asarray(q,float);s=np.asarray(s,float)
    start=regular.evaluate_global([s[0]])
    m=controls;xp=(s-s[0])/(s[-1]-s[0]);L=s[-1]-s[0]
    B=quintic_basis(xp,m);c=cp.Variable((m,7));z=q[:,2:].copy()
    if heading_dip:
        require(stationary_index is not None,'HEADING_DIP','stationary index required')
        progress=np.clip((s-s[0])/(s[stationary_index]-s[0]),0,1)
        z[:,0]-=heading_dip*np.sin(np.pi*progress)**2
    w=np.r_[xp[1]-xp[0],xp[2:]-xp[:-2],xp[-1]-xp[-2]]/2
    D3=quintic_basis(np.linspace(0,1,101),m,3)
    obj=cp.sum_squares(cp.multiply(np.sqrt(w)[:,None],B@c-z))+smoothing*cp.sum_squares(D3@c)/101
    constraints=[c[0]==start.q[0,2:],c[-1]==z[-1],
                 (quintic_basis([0.],m,1)@c)[0]==L*start.qs[0,2:],
                 (quintic_basis([0.],m,2)@c)[0]==L**2*start.qss[0,2:],
                 c[:,1:]>=np.tile(limits.arm_lower.value,(m,1)),
                 c[:,1:]<=np.tile(limits.arm_upper.value,(m,1)),
                 B@c-q[:,2:]<=np.tile(np.r_[np.pi/2,np.full(6,.1)],(len(q),1)),
                 B@c-q[:,2:]>=-np.tile(np.r_[np.pi/2,np.full(6,.1)],(len(q),1))]
    problem=cp.Problem(cp.Minimize(obj),constraints)
    problem.solve(solver='CLARABEL',max_iter=150,tol_gap_abs=1e-9,tol_feas=1e-9)
    require(problem.status in ('optimal','optimal_inaccurate') and c.value is not None,
            'TAIL_PROFILE_QP_FAILED',problem.status)
    fitted=np.asarray(c.value);fitted[-1]=z[-1]
    return QuinticSpline(fitted[:,:1]),QuinticSpline(fitted[:,1:])


def _rho_position_matrix(xi, theta, m):
    """XY integral is linear in rho controls when theta is fixed."""
    result=np.zeros((len(xi),2,m))
    # Integrate each knot span separately; deterministic and stable at knots.
    knots=np.unique(theta.knots)
    nodes,weights=np.polynomial.legendre.leggauss(16)
    for k,end in enumerate(xi):
        cuts=np.r_[0.,knots[(knots>0)&(knots<end)],end]
        for a,b in zip(cuts[:-1],cuts[1:]):
            if b<=a:continue
            x=(a+b)/2+(b-a)*nodes/2
            basis=quintic_basis(x,m)
            heading=theta.evaluate(x)[:,0]
            direction=np.column_stack((np.cos(heading),np.sin(heading)))
            result[k]+=(b-a)/2*np.einsum('n,ni,nj->ij',weights,direction,basis)
    return result


def fit_tail_rho(q,s,regular,theta,stationary_index,*,controls=10):
    """Nonnegative local rho QP with exact integrated XY endpoint."""
    q=np.asarray(q,float);s=np.asarray(s,float);m=controls
    last=stationary_index
    L=s[last]-s[0];xi=(s[:last+1]-s[0])/L
    start=regular.evaluate_global([s[0]])
    p1=start.qs[0,:2];p2=start.qss[0,:2];speed=np.linalg.norm(p1)
    require(speed>1e-9,'TAIL_START_TANGENT','regular entry')
    dr=np.dot(p1,p2)/speed
    rho=cp.Variable(m)
    # theta profile lives on [s0, s_goal], whereas this rho chart is shorter.
    class LocalTheta:
        knots=theta.knots*(s[-1]-s[0])/L
        def evaluate(self,x):
            return theta.evaluate(np.asarray(x)*L/(s[-1]-s[0]))
    A=_rho_position_matrix(xi,LocalTheta(),m)
    ref=q[:last+1,:2]-q[0,:2]
    w=np.r_[xi[1]-xi[0],xi[2:]-xi[:-2],xi[-1]-xi[-2]]/2
    predicted=cp.hstack([cp.reshape(A[:,k,:]@rho,(len(xi),1)) for k in range(2)])*L
    D2=quintic_basis(np.linspace(0,1,101),m,2)
    obj=cp.sum_squares(cp.multiply(np.sqrt(w)[:,None],predicted-ref))+1e-6*cp.sum_squares(D2@rho)/101
    cons=[rho>=0,rho[0]==speed,(quintic_basis([0.],m,1)@rho)[0]==L*dr,
          rho[-1]==0,(quintic_basis([1.],m,1)@rho)[0]==0,
          predicted[-1,:]==ref[-1,:],predicted-ref<=.1,predicted-ref>=-.1]
    problem=cp.Problem(cp.Minimize(obj),cons)
    problem.solve(solver='CLARABEL',max_iter=200,tol_gap_abs=1e-10,tol_feas=1e-10)
    require(problem.status in ('optimal','optimal_inaccurate') and rho.value is not None,
            'TAIL_RHO_QP_FAILED',problem.status)
    spline=QuinticSpline(np.asarray(rho.value)[:,None]);p_end=q[0,:2]+L*A[-1].dot(np.asarray(rho.value))
    require(np.max(np.abs(p_end-q[last,:2]))<1e-6,'TAIL_XY_ENDPOINT','integral residual')
    return spline
