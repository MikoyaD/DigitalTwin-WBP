#!/usr/bin/env python3
"""One cold-start G0 algorithm, A/B event knots for all frozen-H segments."""
import argparse,json,sys,time,traceback,os
from pathlib import Path
import numpy as np
from scipy.optimize import minimize
from scipy.special import roots_legendre
O=Path(os.environ['V2_G0_AC_OUTPUT']).resolve(); E=O
from v2.timing import g0_ac_geometry as geo
from v2.timing import g1_general as calc
from v2.scripts.stage_v2 import load_parent,limits_for
from v2.geometry.tm_modes import classify
ap=argparse.ArgumentParser();ap.add_argument('segment',type=int,choices=range(1,6));ap.add_argument('scheme',choices=('A','B')); args=ap.parse_args();case=json.loads((O/'solver/cases.json').read_text())[str(args.segment)];dest=O/'cases'/f'seg{args.segment:02}'/args.scheme;dest.mkdir(parents=True,exist_ok=True)
def unique_machine(x):
 x=sorted(float(v) for v in x);out=[]
 for y in x:
  if not out or y-out[-1]>8*np.finfo(float).eps*max(1.,abs(y),abs(out[-1])):out.append(y)
 return np.asarray(out)
def setup_h():
 z0=np.load(E/case['H']);z={k:z0[k] for k in z0.files}
 if case.get('rho_index')=='move_ordinal':
  old={k:z[k] for k in z if k.startswith('rho_')};count=0
  for i,sign in enumerate(z['directions']):
   if sign:z[f'rho_{i}']=old[f'rho_{count}'];count+=1
  np.savez_compressed(O/'cases'/f'seg{args.segment:02}'/'geometry_adapter.npz',**z)
 return geo.Geometry(z),z
start=time.perf_counter();h,z=setup_h();parent,path=load_parent(Path(case['parent']) if case['parent'].startswith('/') else E/case['parent']);lim=limits_for(parent);vmax=np.r_[lim.xy_component_velocity.value,lim.angular_velocity.value,lim.arm_velocity.value];amax=np.r_[lim.xy_component_acceleration.value,lim.angular_acceleration.value,lim.arm_acceleration.value];assert np.all(vmax==.5) and np.all(amax==1.) and lim.metadata['constraint_profile']['identity']=='V2_NINE_DOF_VA_ONLY'
_,up,*_=classify(path);bounds=np.array([geo.phi_inv(float(x)) for x in h.edges]);A=unique_machine(np.r_[np.linspace(0,1,27),bounds]);added=[]
if args.scheme=='B':
 for b in bounds[1:-1]:
  i=int(np.searchsorted(A,b));i=min(i,len(A)-1)
  if i==0 or abs(A[i]-b)>1e-14:raise RuntimeError('missing boundary')
  added.extend(((A[i-1]+b)/2,(b+A[i+1])/2))
nodes=unique_machine(np.r_[A,added]);S=40.;knots=nodes*S;events=list(up)
for ch in h.cs:events.extend(ch.a+ch.h*ch.knots)
cuts=unique_machine(np.r_[[geo.phi_inv(float(u)) for u in np.unique(events)],nodes,bounds]);gn,_=roots_legendre(6);inner=((cuts[:-1,None]+cuts[1:,None])/2+(cuts[1:,None]-cuts[:-1,None])*gn/2).ravel();s=unique_machine(np.r_[cuts,inner]);unit=geo.Law(np.array([0.,1.]),np.array([S,S]));_,_,v,a,j,_,ci=geo.timeline(h,s,unit);train=calc.prep({'s':s*S,'chart':ci,'v':v,'a':a,'j':j},knots)
r0=max(float(np.max(abs(v)/vmax)),float(np.sqrt(np.max(abs(a)/amax))),.1)*1.02;init=np.full(len(nodes),r0);upper=max(20.,float(r0)*2);bd=[(.0025,upper)]*len(nodes)
# Same event-sensitive candidate samples and same post-scaling policy in all ten cases.
dense=unique_machine(np.r_[np.linspace(0,1,70001),np.concatenate([np.linspace(a,b,65) for a,b in zip(cuts[:-1],cuts[1:])]),bounds]);u,q,V,A2,J,_,c=geo.timeline(h,dense,unit);full=calc.prep({'s':dense*S,'chart':c,'v':V,'a':A2,'j':J},knots)
def save(x,name,info,scale=True):
 v,a,j,*_=calc.evaluate(x,full);raw_v=float(np.max(abs(v)/vmax));raw_a=float(np.max(abs(a)/amax));factor=max(1.,raw_v,np.sqrt(raw_a))*1.004 if scale else 1.;x=x*factor;v,a,j,*_=calc.evaluate(x,full);law=geo.Law(knots,x);tt=law.at(dense*S)[3];keep=np.r_[True,np.diff(tt)>0];np.savez_compressed(dest/(name+'.npz'),t=tt[keep],u=u[keep],q=q[keep],velocity=v[keep],acceleration=a[keep],jerk=j[keep],chart=c[keep],direction=z['directions'][c[keep]],base_s=dense[keep],g0_density_knots=np.array([0.,1.]),g0_density_values=np.array([S,S]),g1_density_knots=knots,g1_density_values=x,normalized_nodes=nodes,physical_density=x*S)
 record={'duration_s':float(tt[-1]),'raw_v_ratio':raw_v,'raw_a_ratio':raw_a,'safe_scale':float(factor),'solver':info,'training_min_constraint':float(np.min(calc.constraints(x,train)))};(dest/(name+'_solver.json')).write_text(json.dumps(record,indent=2,allow_nan=False));return x
geo_sample=np.unique(np.r_[np.linspace(0,1,10001),bounds]);gj,_=h.evaluate(geo.phi(geo_sample));meta={'segment':args.segment,'scheme':args.scheme,'source_H':str(E/case['H']),'parent':case['parent'],'profile':lim.metadata['constraint_profile']['identity'],'source_evidence':case['geometry_evidence'],'initial_density_s':S*r0,'nodes':nodes.tolist(),'base_nodes':A.tolist(),'inserted_nodes':unique_machine(added).tolist(),'chart_boundaries_s':bounds.tolist(),'chart_boundaries_u':h.edges.tolist(),'directions':z['directions'].tolist(),'train_samples':len(s),'candidate_samples':len(dense),'geometric_jet_max_by_q_and_order':[np.max(abs(x),axis=0).tolist() for x in gj[1:]],'initial_source':'H jets and effective v/a only','solver_upper_x':upper,'geometry_index_adapter':case.get('rho_index','chart_index')};(dest/'metadata.json').write_text(json.dumps(meta,indent=2,allow_nan=False))
save(init,'initial',{'method':'H-only constant density'},scale=False)
trace=[]
def cb(x):trace.append([float(train['weights']@x),float(np.min(calc.constraints(x,train)))])
st=time.perf_counter();f=minimize(lambda x:float(train['weights']@x),init,jac=lambda x:train['weights'],method='SLSQP',bounds=bd,constraints=[{'type':'ineq','fun':lambda x:calc.constraints(x,train),'jac':lambda x:calc.jac(x,train)}],callback=cb,options={'maxiter':160,'ftol':1e-8});fw=time.perf_counter()-st
info=lambda r,w:{'success':bool(r.success),'status':int(r.status),'message':str(r.message),'nit':int(r.nit),'wall_s':w}
x=save(f.x,'fast',{**info(f,fw),'trace':trace});_,_,js,*_=calc.evaluate(x,train);y=np.r_[x,max(float(np.max(abs(js)))/5*1.01,1e-10)]
def cfun(y):
 _,_,j,*_=calc.evaluate(y[:-1],train);return np.r_[calc.constraints(y[:-1],train),y[-1]-np.max(abs(j),axis=0)/5]
def cjac(y):
 a=calc.jac(y[:-1],train,True);return np.vstack([np.c_[a[:18],np.zeros(18)],np.c_[a[18:]/5,np.ones(9)]])
tr=[]
def cb2(y):tr.append([float(train['weights']@y[:-1]),float(y[-1]),float(np.min(cfun(y)))])
st=time.perf_counter();sm=minimize(lambda y:float(train['weights']@y[:-1]+8*y[-1]),y,jac=lambda y:np.r_[train['weights'],8.],method='SLSQP',bounds=bd+[(0,None)],constraints=[{'type':'ineq','fun':cfun,'jac':cjac}],callback=cb2,options={'maxiter':180,'ftol':1e-8});sw=time.perf_counter()-st;save(sm.x[:-1],'smooth',{**info(sm,sw),'trace':tr,'initialization':'this run own fast G0','objective':'T + 8 s * max_i(|jerk_i|/5), soft, not hard limit'})
print(json.dumps({'segment':args.segment,'scheme':args.scheme,'wall_s':time.perf_counter()-start,'fast':info(f,fw),'smooth':info(sm,sw),'initial_time_s':S*r0,'nodes':len(nodes)}),flush=True)
