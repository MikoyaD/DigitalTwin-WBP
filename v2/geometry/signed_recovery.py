"""Bounded signed-R feasibility recovery for an ADJUST/MOVE/ADJUST parent.

Only an exact fixed-profile rho-QP infeasibility enables this optional fallback.
Every returned candidate must pass the unchanged signed geometry verifier.
"""
import time
import numpy as np
import cvxpy as cp
from scipy.sparse import csr_matrix
from numpy.polynomial.legendre import leggauss
from .tm_modes import classify
from .unified_block import _reference_at
from .signed_reconstruction import basis,design,mode_directions,fit_profiles,verify
from ..core.math_tools import quintic_knots

class Quad:
 def __init__(self,x,h):
  cuts=np.unique(np.r_[quintic_knots(18),quintic_knots(36)]);z,w=leggauss(12);nodes=[];weights=[];row=[]
  for i,t in enumerate(x):
   c=cuts[(cuts>0)&(cuts<t)]
   for a,b in zip(np.r_[0,c],np.r_[c,t]):
    if b>a:nodes.extend((.5*(a+b)+.5*(b-a)*z).tolist());weights.extend((h*.5*(b-a)*w).tolist());row.extend([i]*len(z))
  self.M=csr_matrix((weights,(row,np.arange(len(row)))),shape=(len(x),len(row)));self.B=basis(nodes,18);self.R=basis(nodes,36)
 def evaluate(self,th,r):
  angle=self.B@th;rad=self.R@r;e=np.column_stack((np.cos(angle),np.sin(angle)));ep=np.column_stack((-np.sin(angle),np.cos(angle)))
  I=np.column_stack([self.M@(rad*e[:,j]) for j in range(2)]);A=np.stack([self.M@(e[:,j,None]*self.R) for j in range(2)],axis=1);K=np.stack([self.M@((rad*ep[:,j])[:,None]*self.B) for j in range(2)],axis=1);return I,A,K

def solve(path,limits,max_iterations=12):
 started=time.perf_counter();q,u,src,modes,_=classify(path);charts,stats=mode_directions(q,modes);signs=[s for _,_,s in charts]
 if len(signs)!=3 or signs[0]!=0 or signs[2]!=0 or signs[1] not in (-1,1):raise ValueError('SIGNED_RECOVERY_REQUIRES_ADJUST_MOVE_ADJUST')
 sign=signs[1];profiles,profile_status=fit_profiles(q,u,charts,limits);edges=np.array([u[charts[0][0]]]+[u[j] for _,j,_ in charts]);h=edges[2]-edges[1];start=q[0,:2];target=q[-1,:2]
 # Diagnostic elastic tube finds a deterministic, nonnegative, closed seed.
 uu=np.unique(np.r_[np.linspace(edges[1],edges[2],2001),u[(u>=edges[1])&(u<=edges[2])],edges[1]+h*quintic_knots(36),edges[1]+h*quintic_knots(18)]);x=(uu-edges[1])/h;ref=_reference_at(u,q,uu);mat=design(x,h,profiles[1][:,0],36);r=cp.Variable(36);z=cp.Variable(nonneg=True);xy=cp.hstack([cp.reshape(start[d]+sign*mat[:,d,:]@r,(len(x),1),order='C') for d in range(2)]);cs=[r>=0,r[0]==0,r[1]==0,r[-1]==0,r[-2]==0,xy[-1,:]==target,xy-ref[:,:2]<=z,ref[:,:2]-xy<=z]
 # Include stationary charts in the same diagnostic tube, as in formal R.
 for k in (0,2):
  a,b,_=charts[k];rr=_reference_at(u,q,np.unique(np.r_[u[a:b+1],np.linspace(u[a],u[b],2001)]))[:,:2];anchor=start if k==0 else target;cs.extend([anchor-rr<=z,rr-anchor<=z])
 prob=cp.Problem(cp.Minimize(z),cs);prob.solve(solver='ECOS',max_iters=1000,abstol=1e-8,reltol=1e-8,feastol=1e-8)
 if prob.status not in ('optimal','optimal_inaccurate') or r.value is None:raise RuntimeError('RECOVERY_INITIALIZATION_'+str(prob.status))
 initial_tube=float(z.value);rho=np.asarray(r.value).copy();th=profiles[1][:,0].copy();original_theta=th.copy()
 uu=np.unique(np.r_[np.linspace(edges[1],edges[2],1001),u[(u>=edges[1])&(u<=edges[2])],edges[1]+h*quintic_knots(18),edges[1]+h*quintic_knots(36)]);x=(uu-edges[1])/h;ref=_reference_at(u,q,uu);quad=Quad(x,h);inner=.0995
 def score(th,rho):
  I,_,_=quad.evaluate(th,rho);pts=start+sign*I;maximum=float(np.max(abs(pts-ref[:,:2])));closure=float(np.max(abs(pts[-1]-target)));return {'xy_m':maximum,'closure_m':closure,'violation_m':max(0.,maximum-inner,closure)}
 before=score(th,rho);iterations=[];trust=.12
 for it in range(1,max_iterations+1):
  I,A,K=quad.evaluate(th,rho);dt=cp.Variable(18);dr=cp.Variable(36);alpha=cp.Variable(nonneg=True);pred=cp.hstack([cp.reshape(start[d]+sign*(I[:,d]+A[:,d,:]@dr+K[:,d,:]@dt),(len(x),1),order='C') for d in range(2)]);newr=rho+dr;newt=th+dt;cs=[newr>=0,newr[0]==0,newr[1]==0,newr[-1]==0,newr[-2]==0,cp.abs(dt)<=trust,cp.abs(dr)<=max(1.,.5*np.max(abs(rho))),basis(x,18)@newt-ref[:,2]<=np.pi/2,basis(x,18)@newt-ref[:,2]>=-np.pi/2,pred-ref[:,:2]<=inner+alpha,ref[:,:2]-pred<=inner+alpha,pred[-1,:]-target<=alpha,target-pred[-1,:]<=alpha]
  for d in range(3):cs.extend([basis([0],18,d)@dt==0,basis([1],18,d)@dt==0])
  qp=cp.Problem(cp.Minimize(alpha+1e-7*cp.sum_squares(dt)+1e-9*cp.sum_squares(dr)),cs);it0=time.perf_counter();qp.solve(solver='CLARABEL',max_iter=250,tol_feas=1e-7,tol_gap_abs=1e-7,tol_gap_rel=1e-7);entry={'iteration':it,'status':qp.status,'wall_s':time.perf_counter()-it0,'before':before,'elastic_m':float(alpha.value) if alpha.value is not None else None};iterations.append(entry)
  if dt.value is None or dr.value is None:raise RuntimeError('RECOVERY_JOINT_QP_'+str(qp.status))
  choices=[]
  for f in (1.,.5,.25,.125,.0625):
   t=th+f*dt.value;r=rho+f*dr.value;v=score(t,r);choices.append((v['violation_m'],f,t,r,v))
  choice=min(choices,key=lambda v:v[0]);entry['step']=choice[1];entry['after']=choice[4]
  if choice[0]>=before['violation_m']-1e-10:trust*=.5
  else:th,rho,before=choice[2],choice[3],choice[4]
  if before['xy_m']<=.09998 and before['closure_m']<=1e-7:break
  if trust<.003:break
 # Exact nonlinear endpoint closure reduces to a linear rho QP once theta is fixed.
 uu=np.unique(np.r_[np.linspace(edges[1],edges[2],2001),u[(u>=edges[1])&(u<=edges[2])],edges[1]+h*quintic_knots(18),edges[1]+h*quintic_knots(36)]);x=(uu-edges[1])/h;ref=_reference_at(u,q,uu);A=design(x,h,th,36);r=cp.Variable(36);xy=cp.hstack([cp.reshape(start[d]+sign*A[:,d,:]@r,(len(x),1),order='C') for d in range(2)]);cs=[r>=0,r[0]==0,r[1]==0,r[-1]==0,r[-2]==0,xy-ref[:,:2]<=.0999,ref[:,:2]-xy<=.0999,xy[-1,:]==target];last=cp.Problem(cp.Minimize(cp.sum_squares(r-rho)),cs);last.solve(solver='CLARABEL',max_iter=300,tol_feas=1e-9,tol_gap_abs=1e-9,tol_gap_rel=1e-9)
 if last.status not in ('optimal','optimal_inaccurate') or r.value is None:raise RuntimeError('RECOVERY_EXACT_CLOSURE_'+str(last.status))
 data={'global_u':edges,'directions':np.asarray(signs),'source_indices':np.array([src[charts[0][0]]]+[src[j] for _,j,_ in charts]),'start_xy':start,'knots_theta':quintic_knots(18),'knots_rho':quintic_knots(36)}
 for k,v in enumerate(profiles):data['profile_%d'%k]=v.copy()
 data['profile_1'][:,0]=th;data['rho_1']=np.asarray(r.value);check=verify(data,q,u,limits)
 if check['status']!='PASS':raise RuntimeError('RECOVERY_GEOMETRY_NOT_PASS '+str(check['aggregate']))
 info={'method':'theta_rho_scp','wall_s':time.perf_counter()-started,'profile_status':profile_status,'initial_tube_m':initial_tube,'iterations':iterations,'closure_status':last.status,'theta_control_change':float(np.max(abs(th-original_theta))),'rho_controls':36,'parent_states':len(path.q),'charts':[{'start':int(src[i]),'end':int(src[j]),'direction':int(s)} for i,j,s in charts],'xy_length':float(stats[:,0].sum()),'forward_length':float(stats[:,1].sum()),'reverse_length':float(stats[:,2].sum()),'independent_verification':check}
 return data,info,(q,u)
