"""Numerical export of the existing G1 lambda search on a signed frozen H.

The base parameter r belongs to C; lambda's independent variable is C's
physical time. Geometry and both time transformations are evaluated directly.
"""
import json
from pathlib import Path
import numpy as np
from scipy.optimize import brentq
from scipy.special import roots_legendre
from ..timing.unified_g import _inverse_quintic
from .g1_p01_finalize import law
from ..timing.g0_ac_geometry import chart_events


def export_numeric(out,block,path,geo,base,k,x,limits):
 out=Path(out);out.mkdir(parents=True,exist_ok=False)
 def state(c,r):
  t0,u,q,v0,a0,j0=base.from_parameter(c,np.asarray(r));t,L,D,E=law(t0,k,x);l=L[:,None];d=D[:,None];e=E[:,None]
  return t,u,q,v0/l,a0/l**2-v0*d/l**3,j0/l**3-3*a0*d/l**4-v0*e/l**4+3*v0*d*d/l**5
 def r_for_t(t): return brentq(lambda r:base.density.integrate(np.array([r]))[0]-t,0.,base.end,xtol=2e-12)
 rknots=np.array([0. if t<=0 else base.end if t>=base.T else r_for_t(t) for t in k]); pieces=[];charts=[];seams=[];quaderr=np.zeros(3);intabs=np.zeros(9)
 gn,gw=roots_legendre(48)
 for ci,c in enumerate(block.charts):
  values=np.r_[geo['construction']['progress'],chart_events(c),c.s0,c.s1];values=np.unique(np.clip(values,c.s0,c.s1));cuts=np.array([_inverse_quintic(float(u))*base.end for u in values]);cuts=np.unique(np.r_[cuts,base.density.k[(base.density.k>cuts[0])&(base.density.k<cuts[-1])],rknots[(rknots>cuts[0])&(rknots<cuts[-1])]])
  direction=getattr(c,'direction',0 if getattr(c,'stationary',False) else 1)
  r=np.unique(np.r_[np.linspace(cuts[0],cuts[-1],10001),np.concatenate([np.linspace(a,b,65) for a,b in zip(cuts[:-1],cuts[1:])])]); t,u,q,v,a,j=state(c,r); pieces.append((t,u,q,v,a,j,np.full(len(r),ci),np.full(len(r),direction)))
  # Independent physical-time quadrature on all geometry and time-law spans.
  rr=((cuts[:-1,None]+cuts[1:,None])/2+(cuts[1:,None]-cuts[:-1,None])*gn/2).ravel();tt,uu,qq,vv,aa,jj=state(c,rr);t0=base.density.integrate(rr);L=law(t0,k,x)[1];w=(cuts[1:]-cuts[:-1])[:,None]*gw[None,:]/2;dt=(base.density.jets(rr)[0]*L).reshape(len(cuts)-1,-1);end=state(c,cuts)
  for z,(vfield,endpoint) in enumerate(((vv,end[2]),(aa,end[3]),(jj,end[4]))):
   integ=np.einsum('ij,ijk->ik',w*dt,vfield.reshape(len(cuts)-1,-1,9));quaderr[z]=max(quaderr[z],float(np.max(abs(endpoint[1:]-endpoint[:-1]-integ))))
  intabs+=np.sum(np.einsum('ij,ijk->ik',w*dt,np.abs(jj).reshape(len(cuts)-1,-1,9)),axis=0)
  charts.append({'chart':ci,'direction':direction,'duration_s':float(t[-1]-t[0])})
  if ci:
   left=block.charts[ci-1];l=state(left,np.array([cuts[0]]));right=state(c,np.array([cuts[0]]));seams.append([float(np.max(abs(l[i]-right[i]))) for i in (2,3,4)])
 data=[np.concatenate([p[i] for p in pieces]) for i in range(8)]; t,u,q,v,a,j,ci,direction=data; keep=np.r_[True,np.diff(t)>1e-12];t,u,q,v,a,j,ci,direction=[a0[keep] for a0 in data];lat=-np.sin(q[:,2])*v[:,0]+np.cos(q[:,2])*v[:,1];lon=np.cos(q[:,2])*v[:,0]+np.sin(q[:,2])*v[:,1];endpoint=float(max(np.max(abs(q[0]-block.start)),np.max(abs(q[-1]-block.end))));vpeak=np.max(abs(v),axis=0);apeak=np.max(abs(a),axis=0);jpeak=np.max(abs(j),axis=0);dir_ok=bool(np.all(direction[direction!=0]*lon[direction!=0]>=-1e-10));adjust_ok=bool(np.max(abs(v[direction==0,:2]))<1e-9); valid=bool(np.all(np.diff(t)>0) and np.max(vpeak)<=.5+1e-9 and np.max(apeak)<=1+1e-9 and np.max(abs(lat))<1e-8 and dir_ok and adjust_ok and endpoint<1e-6 and max(max(row) for row in seams)<1e-6 and np.max(abs(v[[0,-1]]))<1e-8 and np.max(abs(a[[0,-1]]))<1e-8 and np.all(quaderr<np.array([1e-4,1e-3,1e-2])))
 margin=float(min(np.min(q[:,3:]-limits.arm_lower.value),np.min(limits.arm_upper.value-q[:,3:])))
 valid=valid and margin>=-1e-8
 np.savez_compressed(out/'trajectory.npz',t=t,u=u,q=q,velocity=v,acceleration=a,jerk=j,chart=ci,direction=direction,nodes=k,lambda_values=x)
 result={'status':'PASS' if valid else 'FAIL','duration_s':float(t[-1]),'velocity_peak':vpeak.tolist(),'acceleration_peak':apeak.tolist(),'jerk_peak':jpeak.tolist(),'jerk_mean':(intabs/t[-1]).tolist(),'lateral_peak':float(np.max(abs(lat))),'endpoint_max':endpoint,'joint_position_margin':margin,'seams_q_v_a':seams,'direction_ok':dir_ok,'adjust_ok':adjust_ok,'chart':charts,'quadrature_span_errors_q_v_a':quaderr.tolist(),'samples':len(t),'collision':'NOT_ATTEMPTED','scope':'event-aware discrete physical-time numerical validation'}
 (out/'result.json').write_text(json.dumps(result,indent=2));return result
