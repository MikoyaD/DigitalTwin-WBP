"""Build signed frozen charts from a parent using the validated fixed-profile QPs.

This implementation covers MOVE charts bounded by ADJUST charts or task ends.
Adjacent MOVE direction changes require a separate boundary model and are rejected.
"""
import time
import numpy as np
import cvxpy as cp
from numpy.polynomial.legendre import leggauss
from scipy.sparse import csr_matrix
from scipy.integrate import quad
from scipy.interpolate import BSpline, PPoly
from scipy.optimize import minimize_scalar
from ..core.math_tools import quintic_basis,quintic_knots
from .tm_modes import classify
from .unified_block import _reference_at
from .signed_frozen import Geometry


def basis(x,n,d=0):return quintic_basis(np.asarray(x,dtype=float),n,d)


def mode_directions(q,modes):
 """Mirror PlanarJointModel DIFF_DRIVE turn-drive-turn interpolation.

 Sample the actual piecewise interpolation with 256 points, as the original
 direction probe did. Its min translation setting is 0.01; no ROS is needed.
 """
 wrap=lambda x: np.arctan2(np.sin(x),np.cos(x))
 stats=[]
 for a,b in zip(q[:-1],q[1:]):
  dx,dy=b[:2]-a[:2];length=float(np.hypot(dx,dy));straight=float(wrap(np.arctan2(dy,dx)-a[2])) if length>.01 else 0.
  backward=float(wrap(straight-np.pi));cost=lambda x: abs(x)+abs(float(wrap(b[2]-a[2]-x)))
  turn=straight if cost(straight)<=cost(backward) else backward;drive=a[2]+turn;last=float(wrap(b[2]-drive))
  # Match the historical probe's default angular distance weight of 1.0.
  total=abs(turn)+length+abs(last);prev=a[:3].copy();fwd=rev=0.
  if total<np.finfo(np.float32).eps:
   stats.append((length,0.,0.));continue
  i_frac=abs(turn)/total;d_frac=length/total
  for t in np.arange(1,257)/256.:
   if t<=i_frac:
    nex=np.array([a[0],a[1],a[2]+turn*t/i_frac])
   elif t<=i_frac+d_frac:
    z=(t-i_frac)/d_frac;nex=np.array([a[0]+dx*z,a[1]+dy*z,drive])
   else:
    z=(t-i_frac-d_frac)/(abs(last)/total);nex=np.array([b[0],b[1],drive+last*z])
   dd=nex[:2]-prev[:2];dist=np.linalg.norm(dd)
   if dist>1e-12:
    yaw=np.arctan2(np.sin(prev[2])+np.sin(nex[2]),np.cos(prev[2])+np.cos(nex[2]));f=dd[0]*np.cos(yaw)+dd[1]*np.sin(yaw)
    if f < -1e-12:rev+=dist
    else:fwd+=dist
   prev=nex
  stats.append((length,fwd,rev))
 stats=np.asarray(stats);charts=[]
 for m in modes:
  i,j=m['start'],m['end']
  if m['kind']=='ADJUST':charts.append((i,j,0));continue
  for k in range(i,j):
   if stats[k,1]>1e-9 and stats[k,2]>1e-9:raise ValueError('MIXED_DIRECTION_PARENT_EDGE %d'%k)
  signs=np.where(stats[:,2]>1e-10,-1,1);start=i
  for k in range(i+1,j+1):
   if k==j or signs[k]!=signs[start]:charts.append((start,k,int(signs[start])));start=k
 if any(a[2] and b[2] for a,b in zip(charts,charts[1:])):raise ValueError('ADJACENT_MOVE_CHARTS_NOT_SUPPORTED')
 return charts,stats


def design(x,h,theta,n):
 cuts=np.unique(np.r_[quintic_knots(18),quintic_knots(n)]);z,w=leggauss(16);nodes=[];weights=[];rows=[]
 for i,t in enumerate(x):
  edges=np.r_[0,cuts[(cuts>0)&(cuts<t)],t]
  for a,b in zip(edges[:-1],edges[1:]):
   if b>a:nodes.extend((.5*(a+b)+.5*(b-a)*z).tolist());weights.extend((h*.5*(b-a)*w).tolist());rows.extend([i]*len(z))
 M=csr_matrix((weights,(rows,np.arange(len(rows)))),shape=(len(x),len(rows)));th=basis(nodes,18)@theta;rho=basis(nodes,n)
 return np.stack([M@(np.cos(th)[:,None]*rho),M@(np.sin(th)[:,None]*rho)],axis=1)


def solve(path,limits):
 start=time.perf_counter();q,u,src,modes,_=classify(path);charts,stats=mode_directions(q,modes)
 grids=[];variables=[];constraints=[];objective=0
 for i,j,s in charts:
  h=u[j]-u[i];uu=np.unique(np.r_[u[i:j+1],np.linspace(u[i],u[j],1001)]);x=np.clip((uu-u[i])/h,0,1);ref=_reference_at(u,q,uu);mat=basis(x,18);c=cp.Variable((18,7));tol=np.tile(np.r_[np.pi/2,np.full(6,.1)],(len(x),1))
  constraints.extend([mat@c-ref[:,2:]<=tol,mat@c-ref[:,2:]>=-tol,c[:,1:]>=np.tile(limits.arm_lower.value,(18,1)),c[:,1:]<=np.tile(limits.arm_upper.value,(18,1))]);objective+=cp.sum_squares(mat@c-ref[:,2:])/len(x)+1e-6*cp.sum_squares(basis(np.linspace(0,1,41),18,2)@c)/41;grids.append((h,uu,x,ref));variables.append(c)
 constraints.extend([variables[0][0,:]==q[0,2:],variables[-1][-1,:]==q[-1,2:]])
 for k in range(len(charts)-1):
  h1,h2=grids[k][0],grids[k+1][0]
  for d in range(3):constraints.append(min(h1,h2)**d*basis([1],18,d)@variables[k]/h1**d==min(h1,h2)**d*basis([0],18,d)@variables[k+1]/h2**d)
 profile=cp.Problem(cp.Minimize(objective),constraints);profile.solve(solver='CLARABEL',max_iter=300,tol_feas=1e-9,tol_gap_abs=1e-9)
 if profile.status not in ('optimal','optimal_inaccurate') or any(c.value is None for c in variables):raise RuntimeError('PROFILE_QP_'+profile.status)
 profiles=[np.asarray(v.value) for v in variables];n=22;rhos={k:cp.Variable(n) for k,(_,_,s) in enumerate(charts) if s};cs=[];origin=cp.Constant(q[0,:2]);obj=0;edges=np.array([u[charts[0][0]]]+[u[j] for i,j,s in charts])
 for k,(i,j,s) in enumerate(charts):
  h=edges[k+1]-edges[k];uu=np.unique(np.r_[np.linspace(edges[k],edges[k+1],2001),u[(u>=edges[k])&(u<=edges[k+1])],edges[k]+h*quintic_knots(n),edges[k]+h*quintic_knots(18)]);x=np.clip((uu-edges[k])/h,0,1);ref=_reference_at(u,q,uu)
  if s:
   A=design(x,h,profiles[k][:,0],n);rr=rhos[k];xy=cp.hstack([cp.reshape(origin[d]+s*A[:,d,:]@rr,(len(x),1),order='C') for d in range(2)]);origin=xy[-1,:]
   cs.extend([rr>=0,rr[0]==0,rr[1]==0,rr[-1]==0,rr[-2]==0]);obj+=cp.sum_squares(np.diff(np.eye(n),2,axis=0)@rr)*1e-4
  else:xy=np.ones((len(x),1))@cp.reshape(origin,(1,2),order='C')
  cs.extend([xy-ref[:,:2]<=.0999,xy-ref[:,:2]>=-.0999])
 cs.append(origin==q[-1,:2]);problem=cp.Problem(cp.Minimize(obj),cs);problem.solve(solver='CLARABEL',max_iter=300,tol_feas=1e-9,tol_gap_abs=1e-9,tol_gap_rel=1e-9)
 if problem.status not in ('optimal','optimal_inaccurate') or any(v.value is None for v in rhos.values()):raise RuntimeError('RHO_QP_'+problem.status)
 data={'global_u':edges,'directions':np.array([s for i,j,s in charts]),'source_indices':np.array([src[charts[0][0]]]+[src[j] for i,j,s in charts]),'start_xy':q[0,:2],'knots_theta':quintic_knots(18),'knots_rho':quintic_knots(n)}
 for k,v in enumerate(profiles):data['profile_%d'%k]=v
 for k,v in rhos.items():data['rho_%d'%k]=np.asarray(v.value)
 return data,{'wall_s':time.perf_counter()-start,'profile_status':profile.status,'rho_status':problem.status,'parent_states':len(path.q),'charts':[{'start':int(src[i]),'end':int(src[j]),'direction':int(s)} for i,j,s in charts],'xy_length':float(stats[:,0].sum()),'forward_length':float(stats[:,1].sum()),'reverse_length':float(stats[:,2].sum())},(q,u)


def verify(data,q,u,limits):
 """Independent dense geometry/limits and knotwise quadrature check."""
 geom=Geometry(data);aud=[];jets=[];quaderr=0.;margin=float('inf');rho_min=float('inf')
 for k,c in enumerate(geom.cs):
  grid=np.unique(np.r_[np.linspace(c.a,c.b,5001),u[(u>=c.a)&(u<=c.b)],c.a+c.h*c.knots]);v=c.evaluate(grid);ref=_reference_at(u,q,grid);delta=abs(v[0]-ref);xy=np.max(delta[:,:2],axis=0)
  for dim in range(2):
   a=delta[:,dim];peaks=np.flatnonzero((a[1:-1]>=a[:-2])&(a[1:-1]>=a[2:])&(a[1:-1]>.095))+1
   for i in peaks:
    f=lambda s:-abs(float(c.evaluate(np.array([s]))[0][0,dim]-_reference_at(u,q,np.array([s]))[0,dim]));r=minimize_scalar(f,bounds=(grid[i-1],grid[i+1]),method='bounded',options={'xatol':1e-13});xy[dim]=max(xy[dim],-r.fun)
  for j in range(6):
   pp=PPoly.from_spline((c.pknots,c.p[:,j+1],5));crit=pp.derivative().roots(extrapolate=False);crit=crit[np.isfinite(crit)&(crit>0)&(crit<1)];vals=BSpline(c.pknots,c.p[:,j+1],5)(np.r_[0.,1.,crit]);margin=min(margin,float(min(np.min(vals)-limits.arm_lower.value[j],limits.arm_upper.value[j]-np.max(vals))))
  lat=-np.sin(v[0][:,2])*v[1][:,0]+np.cos(v[0][:,2])*v[1][:,1];lon=np.cos(v[0][:,2])*v[1][:,0]+np.sin(v[0][:,2])*v[1][:,1]
  if c.sign:
   rho_min=min(rho_min,float(min(c.r)));spl=BSpline(c.rknots,c.r,5);integral=[]
   for dim in range(2):
    def f(t):
     th=float(basis([t],len(c.p))@c.p[:,0]);return c.h*float(spl(t))*(np.cos(th) if dim==0 else np.sin(th))
    integral.append(sum(quad(f,float(l),float(r),epsabs=1e-11,epsrel=1e-11)[0] for l,r in zip(c.knots[:-1],c.knots[1:]) if r>l))
   quaderr=max(quaderr,float(np.max(abs((v[0][-1,:2]-c.anchor)/c.sign-integral))))
  aud.append({'chart':k,'direction':c.sign,'xy_max':float(max(xy)),'theta_max':float(max(delta[:,2])),'arm_max':float(np.max(delta[:,3:])),'lateral_max':float(max(abs(lat))),'signed_longitudinal_min':float(min(c.sign*lon)) if c.sign else 0.});jets.append(v)
 seams=[ [float(np.max(abs(jets[k][d][-1]-jets[k+1][d][0]))) for d in range(3)] for k in range(len(jets)-1)]
 maxima={'xy':max(x['xy_max'] for x in aud),'theta':max(x['theta_max'] for x in aud),'arm':max(x['arm_max'] for x in aud),'lateral':max(x['lateral_max'] for x in aud),'seams':np.max(seams,axis=0).tolist() if seams else [0,0,0],'endpoint':float(max(np.max(abs(jets[0][0][0]-q[0])),np.max(abs(jets[-1][0][-1]-q[-1])))),'quadrature':quaderr,'joint_margin':margin,'rho_min':rho_min}
 ok=maxima['xy']<=.1 and maxima['theta']<=np.pi/2 and maxima['arm']<=.1 and maxima['lateral']<1e-8 and max(maxima['seams'])<=1e-6 and maxima['endpoint']<=1e-6 and quaderr<=1e-8 and margin>=-1e-8 and rho_min>=-1e-9 and all(x['signed_longitudinal_min']>=-1e-9 for x in aud)
 return {'status':'PASS' if ok else 'FAIL','aggregate':maxima,'charts':aud,'seams':seams,'scope':'discrete geometry plus numerical extrema; not collision or continuous certificate'}
