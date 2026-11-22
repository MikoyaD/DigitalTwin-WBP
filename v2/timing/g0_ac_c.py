#!/usr/bin/env python3
"""Uniform A continuation and nested C, starting from independently obtained A/smooth."""
import json,sys,time,argparse,os
from pathlib import Path
import numpy as np
from v2.core import array_store as store
from scipy.optimize import minimize
from scipy.special import roots_legendre
O=Path(os.environ['V2_G0_AC_OUTPUT']).resolve(); E=O; PREV=O
from v2.timing import g0_ac_geometry as geo
from v2.timing import g1_general as calc
from v2.scripts.stage_v2 import load_parent,limits_for
from v2.geometry.tm_modes import classify

def uniq(x):
 x=sorted(float(v) for v in x); o=[]
 for y in x:
  if not o or y-o[-1]>8*np.finfo(float).eps*max(1.,abs(y),abs(o[-1])):o.append(y)
 return np.array(o)
def bm(x,k):
 # bump supports are distinct original A intervals; support and first two derivatives vanish on endpoints.
 idx=np.clip(np.searchsorted(k,x,side='right')-1,0,len(k)-2); h=k[idx+1]-k[idx]; t=np.clip((x-k[idx])/h,0,1)
 p=64*t**3*(1-t)**3; p1=192*t**2*(1-t)**2*(1-2*t)/h; p2=384*t*(1-t)*(1-5*t+5*t*t)/h**2
 return idx,p,p1,p2
class Density:
 def __init__(self,k,x,active,beta):self.k=k;self.x=x;self.active=active;self.beta=beta;self.index=np.full(len(k)-1,-1,int);self.index[active]=np.arange(len(active))
 def jets(self,r):
  i,p,p1,p2=bm(r,self.k); sel=self.index[i]; b=np.where(sel>=0,self.beta[np.maximum(sel,0)] if len(self.beta) else 0.,0.); h=self.k[i+1]-self.k[i];t=np.clip((r-self.k[i])/h,0,1);delta=self.x[i+1]-self.x[i];v=self.x[i]+delta*t**3*(10+t*(-15+6*t));d=delta*30*t*t*(1-t)**2/h;e=delta*60*t*(1-t)*(1-2*t)/h**2;f=b*p;f1=b*p1;f2=b*p2;ef=np.exp(np.clip(f,-50,50));return v*ef,(d+v*f1)*ef,(e+2*d*f1+v*(f2+f1*f1))*ef
 def integrate(self,points,n=24):
  pts=np.asarray(points); ns,w=roots_legendre(n);h=np.diff(self.k);xx=(self.k[:-1,None]+self.k[1:,None])/2+h[:,None]*ns/2;prefix=np.r_[0,np.cumsum(h/2*np.sum(self.jets(xx.ravel())[0].reshape(xx.shape)*w,axis=1))]; i=np.clip(np.searchsorted(self.k,pts,side='right')-1,0,len(h)-1);dx=pts-self.k[i];xx=self.k[i,None]+dx[:,None]*(ns+1)/2;return prefix[i]+dx/2*np.sum(self.jets(xx.ravel())[0].reshape(xx.shape)*w,axis=1)
 def at(self,r):
  d,d1,d2=self.jets(r);return d,d1,d2,self.integrate(r)
class Problem:
 def __init__(self,grid,k,active,base,quadrature):
  self.s=grid;self.k=k;self.active=active;self.B,self.D,self.E=calc.matrices(grid,k); self.V,self.A,self.J=base; idx,p,p1,p2=bm(grid,k); W=np.zeros((len(grid),len(active)));W1=W.copy();W2=W.copy(); amap=np.full(len(k)-1,-1,int);amap[active]=np.arange(len(active));selected=amap[idx];good=selected>=0;r=np.where(good)[0];W[r,selected[good]]=p[good];W1[r,selected[good]]=p1[good];W2[r,selected[good]]=p2[good];self.W,self.W1,self.W2=W,W1,W2
  cuts=k;gn,gw=roots_legendre(quadrature); xx=((cuts[:-1,None]+cuts[1:,None])/2+(cuts[1:,None]-cuts[:-1,None])*gn/2).ravel(); self.Q,self.Q1,self.Q2=calc.matrices(xx,k);ii,pp,_,_=bm(xx,k);vv=amap[ii];self.QW=np.zeros((len(xx),len(active)));rr=np.where(vv>=0)[0];self.QW[rr,vv[rr]]=pp[rr];self.qw=np.repeat(np.diff(cuts)/2,quadrature)*np.tile(gw,len(cuts)-1)
 def calculate(self,y,with_derivs=False):
  n=len(self.k);x=y[:n];beta=y[n:n+len(self.active)];F=self.W@beta;F1=self.W1@beta;F2=self.W2@beta;ef=np.exp(np.clip(F,-50,50));b=self.B@x;b1=self.D@x;b2=self.E@x;L=b*ef;D=(b1+b*F1)*ef;E=(b2+2*b1*F1+b*(F2+F1**2))*ef;l=L[:,None];d=D[:,None];e=E[:,None];V,A,J=self.V,self.A,self.J;v=V/l;a=A/l**2-V*d/l**3;j=J/l**3-3*A*d/l**4-V*e/l**4+3*V*d**2/l**5
  if not with_derivs:return v,a,j
  LX=self.B*ef[:,None];DX=(self.D+self.B*F1[:,None])*ef[:,None];EX=(self.E+2*self.D*F1[:,None]+self.B*(F2+F1**2)[:,None])*ef[:,None];LB=L[:,None]*self.W;DB=D[:,None]*self.W+L[:,None]*self.W1;EB=E[:,None]*self.W+2*D[:,None]*self.W1+L[:,None]*self.W2;Lv=np.c_[LX,LB];Dv=np.c_[DX,DB];Ev=np.c_[EX,EB]; rows=[]
  for z in range(9):
   ii=np.argmax(abs(v[:,z]));rows.append(np.sign(v[ii,z])*V[ii,z]*Lv[ii]/L[ii]**2)
  for z in range(9):
   ii=np.argmax(abs(a[:,z]));ll=L[ii];vv=V[ii,z];aa=A[ii,z];der=-2*aa*Lv[ii]/ll**3-vv*Dv[ii]/ll**3+3*vv*D[ii]*Lv[ii]/ll**4;rows.append(-np.sign(a[ii,z])*der)
  for z in range(9):
   ii=np.argmax(abs(j[:,z]));ll=L[ii];vv=V[ii,z];aa=A[ii,z];jj=J[ii,z];ss=D[ii];s2=E[ii];der=-3*jj*Lv[ii]/ll**4-3*aa*Dv[ii]/ll**4+12*aa*ss*Lv[ii]/ll**5-vv*Ev[ii]/ll**4+4*vv*s2*Lv[ii]/ll**5+6*vv*ss*Dv[ii]/ll**5-15*vv*ss**2*Lv[ii]/ll**6;rows.append(-np.sign(j[ii,z])*der)
  return np.array(rows)
 def time(self,y,grad=False):
  n=len(self.k);x=y[:n];beta=y[n:n+len(self.active)];e=np.exp(np.clip(self.QW@beta,-50,50));b=self.Q@x;weights=self.qw*e;T=weights@b
  if grad:return np.r_[self.Q.T@weights,self.QW.T@(weights*b)]
  return float(T)
 def constraints(self,y):
  v,a,j=self.calculate(y);return np.r_[.5-np.max(abs(v),axis=0),1-np.max(abs(a),axis=0),y[-1]-np.max(abs(j),axis=0)/5]
 def jac(self,y):
  j=self.calculate(y,True);return np.vstack([np.c_[j[:18],np.zeros(18)],np.c_[j[18:]/5,np.ones(9)]])

def main(seg,mode):
 begin=time.perf_counter();case=json.loads((O/'solver/cases.json').read_text())[str(seg)];out=O/'cases'/f'seg{seg:02}'/mode;out.mkdir(parents=True,exist_ok=True)
 hp=PREV/'cases'/f'seg{seg:02}'/'geometry_adapter.arrays' if case.get('rho_index')=='move_ordinal' else E/case['H']; zz=store.load(hp);z={k:zz[k] for k in zz.files};h=geo.Geometry(z); src=store.load(PREV/'cases'/f'seg{seg:02}'/'A'/'smooth.arrays'); k=src['g1_density_knots']; x0=src['g1_density_values']; nodes=k/40.;bounds=np.array([geo.phi_inv(float(q)) for q in h.edges]); active=[]
 if mode=='C':
  for b in bounds[1:-1]:
   j=int(np.argmin(abs(nodes-b))); assert abs(nodes[j]-b)<1e-12;active.extend([j-1,j])
 active=np.array(sorted(set(active)),int); source_law=geo.Law(k,x0);candidate=Density(k,x0,active,np.zeros(len(active)));test=np.unique(np.r_[np.linspace(0,40,20001),k]);a=source_law.at(test);b=candidate.jets(test);replay=[float(np.max(abs(a[i]-b[i]))) for i in range(3)];tt=candidate.integrate(test[::20]);replay.append(float(np.max(abs(tt-source_law.at(test[::20])[3]))))
 parent,path=load_parent(Path(case['parent']) if case['parent'].startswith('/') else E/case['parent']);lim=limits_for(parent);assert lim.metadata['constraint_profile']['identity']=='V2_NINE_DOF_VA_ONLY'; _,up,*_=classify(path);events=list(up)
 for ch in h.cs:events.extend(ch.a+ch.h*ch.knots)
 cuts=uniq(np.r_[[geo.phi_inv(float(u)) for u in np.unique(events)],nodes,bounds]);gn,_=roots_legendre(6);s=uniq(np.r_[cuts,((cuts[:-1,None]+cuts[1:,None])/2+(cuts[1:,None]-cuts[:-1,None])*gn/2).ravel()]);unit=geo.Law(np.array([0.,1.]),np.array([40.,40.]));_,_,v,a,j,_,ci=geo.timeline(h,s,unit);P=Problem(s*40,k,active,(v,a,j),16)
 init=np.r_[x0,np.zeros(len(active)),max(float(np.max(abs(P.calculate(np.r_[x0,np.zeros(len(active))])[2]))/5)*1.01,1e-10)]; meta={'source':'A/smooth independent H-only optimization','mode':mode,'nodes_s':nodes.tolist(),'chart_boundaries_s':bounds.tolist(),'support_intervals_s':[[float(nodes[i]),float(nodes[i+1])] for i in active],'zero_beta_replay_max_abs_density_first_second_time':replay,'training_points':len(s),'iteration_budget':180,'geometry':str(hp)};(out/'metadata.json').write_text(json.dumps(meta,indent=2))
 trace=[]
 def cb(y):
  trace.append([P.time(y),float(y[-1]),float(np.min(P.constraints(y)))])
 optstart=time.perf_counter(); upper=max(20.,float(np.max(x0))*2);r=minimize(lambda y:P.time(y)+8*y[-1],init,jac=lambda y:np.r_[P.time(y,True),8.],method='SLSQP',bounds=[(.0025,upper)]*len(x0)+[(-3.,3.)]*len(active)+[(0,None)],constraints=[{'type':'ineq','fun':P.constraints,'jac':P.jac}],callback=cb,options={'maxiter':180,'ftol':1e-8});elapsed=time.perf_counter()-optstart
 raw=r.x.copy();dense=uniq(np.r_[np.linspace(0,1,70001),np.concatenate([np.linspace(a,b,65) for a,b in zip(cuts[:-1],cuts[1:])]),bounds]);u,q,V,A,J,_,c=geo.timeline(h,dense,unit); F=Problem(dense*40,k,active,(V,A,J),16);v,a,j=F.calculate(raw);vr=float(np.max(abs(v))/.5);ar=float(np.max(abs(a)));factor=(max(vr,np.sqrt(ar))*1.004 if max(vr,np.sqrt(ar))>1 else 1.); x=raw[:len(k)]*factor;beta=raw[len(k):-1];adj=np.r_[x,beta,raw[-1]];v,a,j=F.calculate(adj);L=Density(k,x,active,beta);ts=L.integrate(dense*40);keep=np.r_[True,np.diff(ts)>0]
 # Actual preliminary raw/post objective uses sampled jerk rather than epigraph state.
 raw_obj=P.time(raw)+8*np.max(abs(P.calculate(raw)[2]))/5;post_obj=P.time(adj)+8*np.max(abs(P.calculate(adj)[2]))/5
 store.save(out/'candidate.arrays',t=ts[keep],u=u[keep],q=q[keep],velocity=v[keep],acceleration=a[keep],jerk=j[keep],chart=c[keep],direction=z['directions'][c[keep]],base_s=dense[keep],normalized_nodes=nodes,density_x=x,density_beta=beta,active_intervals=active,g0_density_knots=np.array([0.,1.]),g0_density_values=np.array([40.,40.]),g1_density_knots=k,g1_density_values=x)
 record={'success':bool(r.success),'status':int(r.status),'message':str(r.message),'nit':int(r.nit),'wall_s':elapsed,'total_wall_s':time.perf_counter()-begin,'initial_actual_objective':float(P.time(init)+8*np.max(abs(P.calculate(init)[2]))/5),'raw_actual_objective':float(raw_obj),'post_actual_objective':float(post_obj),'solver_epigraph':float(raw[-1]),'raw_training_min_constraint':float(np.min(P.constraints(raw)[:18])),'raw_dense_v_ratio':vr,'raw_dense_a_ratio':ar,'raw_dense_max_violation':max(0.,vr*.5-.5,ar-1.),'post_dense_v_ratio':float(np.max(abs(v))/.5),'post_dense_a_ratio':float(np.max(abs(a))),'safe_scale':float(factor),'raw_duration_s':P.time(raw),'post_duration_s':P.time(adj),'beta':beta.tolist(),'trace':trace};(out/'solver.json').write_text(json.dumps(record,indent=2,allow_nan=False));print(json.dumps({'seg':seg,'mode':mode,**{key:record[key] for key in ['status','nit','wall_s','raw_duration_s','post_duration_s','raw_dense_max_violation','safe_scale','post_dense_v_ratio','post_dense_a_ratio','post_actual_objective']}}),flush=True)
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('seg',type=int,choices=range(1,6));p.add_argument('mode',choices=['Acontinue','C']);args=p.parse_args();main(args.seg,args.mode)
