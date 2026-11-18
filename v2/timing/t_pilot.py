"""Fixed-v4-H experimental convex time allocation. No geometry changes."""
import time
import numpy as np
import cvxpy as cp
from scipy.interpolate import BSpline
from numpy.polynomial.legendre import leggauss


def basis(u, knots, order=0):
    n=len(knots)-4
    return BSpline(knots,np.eye(n),3,extrapolate=False)(np.asarray(u),nu=order)


def weight(u):
    u=np.asarray(u); return (u*(1-u))**(4./3.)

def weight_d(u):
    u=np.asarray(u); return (4./3.)*(u*(1-u))**(1./3.)*(1-2*u)

def matrices(u,knots):
    B=basis(u,knots); return weight(u)[:,None]*B, weight_d(u)[:,None]*B+weight(u)[:,None]*basis(u,knots,1)


def mesh(block,n):
    return np.unique(np.r_[np.linspace(0,1,n+1),[c.s0 for c in block.charts],[c.s1 for c in block.charts]])

def solve(block,n=80,solver='CLARABEL'):
    started=time.monotonic(); grid=mesh(block,n)
    knots=np.r_[[0.]*4,grid[1:-1],[1.]*4]; nc=len(knots)-4
    # Constraints sampled on each span; two one-sided interior probes near each chart seam.
    uv=np.unique(np.r_[np.concatenate([a+(b-a)*np.array([.00001,.25,.5,.75,.99999]) for a,b in zip(grid[:-1],grid[1:])]),grid[1:-1]])
    A,D=matrices(uv,knots);jet=block.evaluate(uv)
    # Scale b=0.001*w*B*c for solver conditioning.
    c=cp.Variable(nc,nonneg=True);bv=.001*(A@c);bd=.001*(D@c)
    constraints=[bv>=1e-13]
    for k in range(9):
        V=.001*jet.qs[:,k,None]**2*A
        R=.001*(jet.qss[:,k,None]*A+.5*jet.qs[:,k,None]*D)
        vs=np.maximum(1.,np.max(np.abs(V),axis=1));rs=np.maximum(1.,np.max(np.abs(R),axis=1))
        constraints.extend([cp.Constant(V/vs[:,None])@c<=.25/vs,cp.abs(cp.Constant(R/rs[:,None])@c)<=1./rs])
    # Gauss-Legendre integral of b^-1/2; cubic substitutions remove endpoint singularities.
    x,w=leggauss(6);z=(x+1)/2;w=w/2;nodes=[];weights=[]
    for i,(a,b) in enumerate(zip(grid[:-1],grid[1:])):
        if i==0: u=a+(b-a)*z**3; ww=w*3*(b-a)*z**2
        elif i==len(grid)-2: u=b-(b-a)*z**3; ww=w*3*(b-a)*z**2
        else: u=a+(b-a)*z;ww=w*(b-a)
        nodes.extend(u);weights.extend(ww)
    Q,_=matrices(np.asarray(nodes),knots); qb=.001*(Q@c)
    prob=cp.Problem(cp.Minimize(100.*(np.asarray(weights)@cp.power(qb/1e-4,-.5))),constraints)
    result={'solver':solver,'grid_intervals':len(grid)-1,'coefficients_count':nc,'constraint_samples':len(uv),'quadrature_nodes':len(nodes)}
    try:
        data=prob.get_problem_data(solver)[0];result['cone_dimensions']=str(data['dims'])
        prob.solve(solver=solver,verbose=False,**({'max_iter':200} if solver=='CLARABEL' else {'max_iters':20000}))
        result.update(status=str(prob.status),objective_s=None if prob.value is None else float(prob.value),solve_wall_s=time.monotonic()-started,solver_stats=dict(solve_time=prob.solver_stats.solve_time,num_iters=prob.solver_stats.num_iters))
        if c.value is not None:result.update(coefficients=np.asarray(c.value).tolist(),knots=knots.tolist(),grid=grid.tolist())
    except Exception as e:result.update(status='SOLVER_EXCEPTION',error=repr(e),solve_wall_s=time.monotonic()-started)
    return result
