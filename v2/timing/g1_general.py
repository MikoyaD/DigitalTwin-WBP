"""Independent G1: finite time -> jerk -> optional time-recovery search on frozen H."""
import time,numpy as np
from scipy.optimize import minimize
from ..core.math_tools import quintic_time_jet,chain_rule
from ..timing.unified_g import _inverse_quintic
from ..scripts import g1_p01_finalize as base

def matrices(s,k):
 idx=np.clip(np.searchsorted(k,s,side='right')-1,0,len(k)-2);h=k[idx+1]-k[idx];z=np.clip((s-k[idx])/h,0,1);S=z**3*(10-15*z+6*z*z);D=30*z*z*(1-z)**2/h;E=60*z*(1-z)*(1-2*z)/h**2
 B=np.zeros((len(s),len(k)));B1=B.copy();B2=B.copy();r=np.arange(len(s));B[r,idx]=1-S;B[r,idx+1]=S;B1[r,idx]=-D;B1[r,idx+1]=D;B2[r,idx]=-E;B2[r,idx+1]=E
 return B,B1,B2

def raw_cache(block,geo,T,n,base_law=None):
 base.T0=T; rows=[]
 for ci,c in enumerate(block.charts):
  if base_law is None:
   cuts=base.cuts_for(c,geo['construction']['progress'],np.array([0.,T]));s=np.unique(np.concatenate([np.linspace(a,b,n) for a,b in zip(cuts[:-1],cuts[1:])]));u,ud,udd,uj=quintic_time_jet(s,T);jet=c.evaluate_global(np.clip(u,c.s0,c.s1));v,a,j=chain_rule(jet,ud,udd,uj)
  else:
   # Parameter r is the C density's original 0..40 reference time, not physical t0.
   from .g0_ac_geometry import chart_events
   cuts=np.r_[chart_events(c),geo['construction']['progress'],c.s0,c.s1];cuts=np.unique(np.clip(cuts,c.s0,c.s1));r=np.array([_inverse_quintic(float(u))*base_law.end for u in cuts]);r=np.unique(np.r_[r,base_law.density.k[(base_law.density.k>r[0])&(base_law.density.k<r[-1])]]);r=np.unique(np.concatenate([np.linspace(a,b,n) for a,b in zip(r[:-1],r[1:])]));s,u,q,v,a,j=base_law.from_parameter(c,r)
  rows.append((s,np.full(len(s),ci),v,a,j))
 return dict(zip(('s','chart','v','a','j'),(np.concatenate([r[i] for r in rows]) for i in range(5))))

def control_grid(d,T):
 risk=np.max(np.maximum(np.sqrt(np.abs(d['a'])),np.cbrt(abs(d['j'])/5)),axis=1);local=[]
 for ci in np.unique(d['chart']):
  ix=np.flatnonzero(d['chart']==ci);q=risk[ix];good=np.r_[True,(q[1:-1]>=q[:-2])&(q[1:-1]>=q[2:]),True];local.extend(ix[good].tolist())
 peaks=[]
 for j in np.array(local)[np.argsort(-risk[local],kind='mergesort')]:
  s=float(d['s'][j]);
  if all(abs(s-z)>=4 for z in peaks):peaks.append(s)
  if len(peaks)==4:break
 cand=[(0.,0),(T,0)]+[(x,1) for x in peaks]+[(np.clip(x+z,0,T),2) for x in peaks for z in (-2,2)]+[(x,3) for x in np.linspace(0,T,13)[1:-1]];kept=[]
 for s,p in sorted(cand,key=lambda x:(x[1],x[0])):
  if all(abs(s-y[0])>=1 for y in kept):kept.append((s,p))
 return np.array(sorted(s for s,p in kept))

def prep(d,k):
 d=d.copy();d['B'],d['D'],d['E']=matrices(d['s'],k);h=np.diff(k);d['weights']=np.r_[h[0]/2,(h[:-1]+h[1:])/2,h[-1]/2];d['k']=k;return d

def evaluate(x,d):
 L=d['B']@x;D=d['D']@x;E=d['E']@x;V,A,J=d['v'],d['a'],d['j'];l=L[:,None];dd=D[:,None];ee=E[:,None]
 v=V/l;a=A/l**2-V*dd/l**3;j=J/l**3-3*A*dd/l**4-V*ee/l**4+3*V*dd**2/l**5
 return v,a,j,L,D,E

def constraints(x,d,jerk_limit=None):
 v,a,j,*_=evaluate(x,d);z=np.r_[.5-np.max(abs(v),axis=0),1-np.max(abs(a),axis=0)];return np.r_[z,jerk_limit-np.max(abs(j),axis=0)] if jerk_limit is not None else z

def jac(x,d,guard=False):
 v,a,j,l,ls,lss=evaluate(x,d);B,D,E=d['B'],d['D'],d['E'];V,A,J=d['v'],d['a'],d['j'];rows=[]
 for k in range(9):
  i=np.argmax(abs(v[:,k]));rows.append(np.sign(v[i,k])*V[i,k]*B[i]/l[i]**2)
 for k in range(9):
  i=np.argmax(abs(a[:,k]));ll=l[i];vv=V[i,k];aa=A[i,k];der=-2*aa*B[i]/ll**3-vv*D[i]/ll**3+3*vv*ls[i]*B[i]/ll**4;rows.append(-np.sign(a[i,k])*der)
 if guard:
  for k in range(9):
   i=np.argmax(abs(j[:,k]));ll=l[i];vv=V[i,k];aa=A[i,k];jj=J[i,k];ss=ls[i];s2=lss[i];der=-3*jj*B[i]/ll**4-3*aa*D[i]/ll**4+12*aa*ss*B[i]/ll**5-vv*E[i]/ll**4+4*vv*s2*B[i]/ll**5+6*vv*ss*D[i]/ll**5-15*vv*ss**2*B[i]/ll**6;rows.append(-np.sign(j[i,k])*der)
 return np.array(rows)

def evaluate_candidate(x,d):
 v,a,j,*_=evaluate(x,d);return dict(T=float(d['weights']@x),v=np.max(abs(v),axis=0).tolist(),a=np.max(abs(a),axis=0).tolist(),jerk=np.max(abs(j),axis=0).tolist())

def time_search(x,d,jerk_guard=False):
 start=time.perf_counter();guard=5. if jerk_guard else None
 r=minimize(lambda q:float(d['weights']@q),x,jac=lambda q:d['weights'],method='SLSQP',bounds=[(.25,4)]*len(x),constraints=[dict(type='ineq',fun=lambda q:constraints(q,d,guard),jac=lambda q:jac(q,d,jerk_guard))],options={'maxiter':160,'ftol':1e-8});return r,dict(stage='time_recovery' if jerk_guard else 'time',success=bool(r.success),status=int(r.status),message=str(r.message),iterations=int(r.nit),elapsed_s=time.perf_counter()-start)

def smooth_search(x,d,reference_T):
 start=time.perf_counter();j=evaluate(x,d)[2];y0=np.r_[x,float(np.max(abs(j))/5)];n=len(x)
 # A soft search envelope, relative to the actual task, not a legality limit.
 budget=1.25*reference_T
 def fn(y):
  v,a,j,*_=evaluate(y[:-1],d);return np.r_[.5-np.max(abs(v),axis=0),1-np.max(abs(a),axis=0),y[-1]-np.max(abs(j),axis=0)/5,(budget-d['weights']@y[:-1])/max(budget,1)]
 def df(y):
  A=jac(y[:-1],d,True);return np.vstack([np.c_[A[:18],np.zeros(18)],np.c_[A[18:]/5,np.ones(9)],np.r_[-d['weights']/max(budget,1),0]])
 obj=lambda y:y[-1]+.02*(d['weights']@y[:-1])/max(reference_T,1)
 g=lambda y:np.r_[.02*d['weights']/max(reference_T,1),1.]
 r=minimize(obj,y0,jac=g,method='SLSQP',bounds=[(.25,4)]*n+[(0,None)],constraints=[dict(type='ineq',fun=fn,jac=df)],options={'maxiter':160,'ftol':1e-8});return r,dict(stage='smooth',success=bool(r.success),status=int(r.status),message=str(r.message),iterations=int(r.nit),elapsed_s=time.perf_counter()-start,search_time_envelope_s=budget)

def scale_safe(x,d,jerk_guard=False):
 v,a,j,*_=evaluate(x,d);factor=max(1.,float(np.max(abs(v))/.5),float(np.sqrt(np.max(abs(a)))) ,float(np.cbrt(np.max(abs(j))/5)) if jerk_guard else 1.)*1.002
 y=np.minimum(x*factor,4.)  # bounded time-density proposal; physical derivatives never clipped
 if np.min(y)<.25:return None,dict(reason='SCALE_EXCEEDS_NODE_BOUNDS',factor=factor)
 result=evaluate_candidate(y,d)
 if max(result['v'])>.5+1e-12 or max(result['a'])>1+1e-12 or (jerk_guard and max(result['jerk'])>5+1e-9):return None,dict(reason='DENSE_LIMIT_FAILURE',factor=factor,metrics=result)
 return y,dict(reason='DENSE_PASS',factor=factor,metrics=result)
