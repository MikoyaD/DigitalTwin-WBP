"""Eight-coordinate convex fitting QP; explicit bounded candidate search."""
import numpy as np
import cvxpy as cp
from scipy.interpolate import BSpline
from ..core.contracts import *
from ..core.math_tools import *

Z = [0,1,3,4,5,6,7,8]


class LiftedSpline:
    def __init__(self, spline, anchor):
        self.spline = spline
        reg = xy_regularity(spline)
        require(reg.status == Status.PASS, 'GEOMETRY_REGULARITY', reg.reason)
        # Convex-hull bound on |p_ss| / min |p_s| bounds total angle change.
        d2 = BSpline(spline.knots, spline.controls[:,:2], 5).derivative(2)
        bound = np.max(np.linalg.norm(d2.c, axis=1))/reg.minimum_speed
        n = max(33, int(np.ceil(bound/(np.pi/4)))+1)
        require(n <= 32769, 'HEADING_REFINEMENT_BUDGET', str(n))
        self.grid = np.unique(np.r_[np.linspace(0,1,n),spline.knots])
        p = [spline.evaluate(self.grid,k)[:,:2] for k in range(1,5)]
        self.heading = heading_jet(self.grid,*p,anchor=anchor)[0]
        self.regularity = reg

    def __call__(self, s):
        s=np.asarray(s); z=[self.spline.evaluate(s,k) for k in range(5)]
        # Single-point analytic jets avoid imposing query order on the lift.
        p1,p2,p3,p4=[v[:,:2] for v in z[1:]]
        cross=lambda a,b:a[:,0]*b[:,1]-a[:,1]*b[:,0]
        D=np.sum(p1*p1,axis=1); C=cross(p1,p2); C1=cross(p1,p3)
        C2=cross(p2,p3)+cross(p1,p4); D1=2*np.sum(p1*p2,axis=1)
        D2=2*np.sum(p2*p2+p1*p3,axis=1)
        raw=np.arctan2(p1[:,1],p1[:,0])
        theta=raw+2*np.pi*np.round((np.interp(s,self.grid,self.heading)-raw)/(2*np.pi))
        hs=[theta,C/D,C1/D-C*D1/D**2,C2/D-2*C1*D1/D**2-C*D2/D**2+2*C*D1**2/D**3]
        return PathJet(*(np.column_stack((z[k][:,:2],hs[k],z[k][:,2:])) for k in range(4)))


def fit(q, limits, directions=None, zero_curvature=True, smoothing=1e-4):
    param=whole_body_parameter(q); q=q[param.retained_indices]; s=param.s; z=q[:,Z]
    m=max(8,min(32,len(q)+4)); C=cp.Variable((m,8))
    B=quintic_basis(s,m); w=np.r_[s[1]-s[0],s[2:]-s[:-2],s[-1]-s[-2]]/2
    # Explicit normalization: 1 m XY and 1 rad per arm joint.
    scale=np.ones(8); objective=cp.sum_squares(cp.multiply(np.sqrt(w)[:,None],(B@C-z)@np.diag(1/scale)))
    knots=quintic_knots(m); nodes,weights=np.polynomial.legendre.leggauss(3)
    for lo,hi in zip(np.unique(knots)[:-1],np.unique(knots)[1:]):
        x=(lo+hi)/2+(hi-lo)*nodes/2
        objective+=smoothing*cp.sum_squares(cp.multiply(np.sqrt(weights*(hi-lo)/2)[:,None],quintic_basis(x,m,3)@C@np.diag(1/scale)))
    constraints=[B@C-z<=.1,B@C-z>=-.1,C[0]==z[0],C[-1]==z[-1],C[:,2:]>=np.tile(limits.arm_lower.value,(m,1)),C[:,2:]<=np.tile(limits.arm_upper.value,(m,1))]
    if directions is not None:
        speeds=[np.linalg.norm(q[1,:2]-q[0,:2])/(s[1]-s[0]),np.linalg.norm(q[-1,:2]-q[-2,:2])/(s[-1]-s[-2])]
        for i,edge in enumerate((0.,1.)):
            constraints.append((quintic_basis([edge],m,1)@C)[0,:2]==speeds[i]*np.asarray(directions[i]))
            if zero_curvature: constraints.append((quintic_basis([edge],m,2)@C)[0,:2]==0)
    problem=cp.Problem(cp.Minimize(objective),constraints)
    try: problem.solve(solver='CLARABEL',max_iter=150,tol_gap_abs=1e-10,tol_feas=1e-10)
    except cp.error.SolverError as e: raise ContractError('GEOMETRY_SOLVER',str(e))
    require(problem.status in ('optimal','optimal_inaccurate') and C.value is not None,'GEOMETRY_INFEASIBLE',problem.status)
    controls=C.value.copy()
    require(np.max(np.abs(controls[[0,-1]]-z[[0,-1]]))<1e-8,'GEOMETRY_ENDPOINT','solver residual')
    controls[0]=z[0];controls[-1]=z[-1] # exact equality cleanup, bounded above
    require(np.all(controls[:,2:]>=limits.arm_lower.value-1e-8) and np.all(controls[:,2:]<=limits.arm_upper.value+1e-8),'POSITION_LIMIT','control convex hull')
    require(np.max(np.abs(B.dot(controls)-z))<=.10000001,'GEOMETRY_FIT_RESIDUAL','waypoint box')
    if directions is not None:
        for i,edge in enumerate((0.,1.)):
            require(np.max(np.abs(quintic_basis([edge],m,1).dot(controls)[0,:2]-speeds[i]*np.asarray(directions[i])))<1e-7,'TANGENT_RESIDUAL','QP equality')
            if zero_curvature:require(np.max(np.abs(quintic_basis([edge],m,2).dot(controls)[0,:2]))<1e-7,'CURVATURE_RESIDUAL','QP equality')
    return QuinticSpline(controls)


def candidates(q,limits):
    chord=[q[1,:2]-q[0,:2],q[-1,:2]-q[-2,:2]]
    chord=[d/np.linalg.norm(d) for d in chord]
    headings=[np.array([np.cos(q[i,2]),np.sin(q[i,2])]) for i in (0,-1)]
    failures=[]
    for choice in range(4):
        dirs=[headings[i] if choice & (1<<i) else chord[i] for i in range(2)]
        for retry in range(2):
            try:
                spline=fit(q,limits,dirs,zero_curvature=retry==0)
                lift=LiftedSpline(spline,q[0,2])
                yield lift,dict(candidate=choice,retry=retry,normalization=[1.]*8,lambda_g=1e-4,waypoint_box_normalized=.1)
                break
            except ContractError as e: failures.append((choice,retry,e.code))
    if len(failures)==8: raise ContractError('GEOMETRY_CANDIDATES_EXHAUSTED',str(failures))
