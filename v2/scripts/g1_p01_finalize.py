"""P01 fixed-H timewarp export and independent piecewise numerical validation."""
import argparse,json,sys,time,traceback
from pathlib import Path
import numpy as np
from numpy.polynomial.legendre import leggauss
from ..scripts.stage_v2 import load_parent,limits_for,HEADER
from ..geometry.tm_rho_feedback_frozen import thaw
from ..geometry.tm_frozen import verify_geometry
from ..timing.unified_g import _inverse_quintic
from ..core.math_tools import quintic_time_jet,chain_rule,differential_kinematics
from ..core.contracts import TrajectoryRecord,PhaseType,Status
from ..validation.numerical import validate
from ..scripts.tm_feedback_hg import check_workspace
from ..scripts.tm_feedback_piecewise_run import nonintegration_failures

ROOT=Path(__file__).resolve().parents[2]
PROJECT=ROOT.parents[2]
H=ROOT/'v2/outputs/V2-20261030-000922-single_goal_05_TM_RHO_FEEDBACK-RRTstar'
EVID=PROJECT/'research_management/evidence/G1_P01_FEASIBILITY_PILOT_20261104/refined'
T0=140.31872051876778

def law(s,k,x):
 s=np.atleast_1d(s);i=np.clip(np.searchsorted(k,s,side='right')-1,0,len(k)-2);h=k[i+1]-k[i];z=np.clip((s-k[i])/h,0,1);dx=x[i+1]-x[i]
 L=x[i]+dx*z**3*(10-15*z+6*z*z);D=dx*30*z*z*(1-z)**2/h;E=dx*60*z*(1-z)*(1-2*z)/h**2
 acc=np.r_[0,np.cumsum(np.diff(k)*(x[:-1]+x[1:])/2)]
 t=acc[i]+h*(x[i]*z+dx*(2.5*z**4-3*z**5+z**6))
 return t,L,D,E

def eval_chart(c,s,k,x):
 t,L,D,E=law(s,k,x);u,ud,udd,uj=quintic_time_jet(np.asarray(s),T0);u=np.clip(u,c.s0,c.s1);q=c.evaluate_global(u);v0,a0,j0=chain_rule(q,ud,udd,uj);L=L[:,None];D=D[:,None];E=E[:,None]
 v=v0/L;a=a0/L**2-v0*D/L**3;j=j0/L**3-3*a0*D/L**4-v0*E/L**4+3*v0*D**2/L**5
 return t,u,q.q,v,a,j

def cuts_for(c,extra,k):
 h=c.s1-c.s0;cuts=[c.s0,c.s1];spl=c.spline if hasattr(c,'spline') else c.rho;cuts.extend(c.s0+h*spl.knots)
 if hasattr(c,'theta'):
  a,b=c.profile_interval or (c.s0,c.s1)
  for sp in (c.theta,c.arm):cuts.extend(a+(b-a)*sp.knots)
 cuts.extend(p for p in extra if c.s0<p<c.s1);cuts=np.unique(np.clip(cuts,c.s0,c.s1));s=np.array([_inverse_quintic(float(p))*T0 for p in cuts]);s=np.unique(np.r_[s,k[(k>s[0])&(k<s[-1])]])
 return s

def quad(c,cuts,k,x):
 pieces=[]
 for n in (16,32):
  z,w=leggauss(n);lo=cuts[:-1];hi=cuts[1:];ss=((lo+hi)[:,None]+(hi-lo)[:,None]*z)/2;weights=(hi-lo)[:,None]*w/2
  _,_,q,v,a,j=eval_chart(c,ss.ravel(),k,x);_,L,_,_=law(ss.ravel(),k,x);L=L.reshape(ss.shape);vals=[]
  for zval in (v,a,j,abs(j),j*j):vals.append(np.einsum('ij,ijk->ik',weights*L,zval.reshape(len(lo),n,9)))
  pieces.append(vals)
 gap=max(float(np.max(abs(a-b))) for a,b in zip(pieces[0][:3],pieces[1][:3]));return pieces[1],gap

def main():
 global H,T0
 p=argparse.ArgumentParser();p.add_argument('--out',required=True);p.add_argument('--scale',type=float,default=1.);p.add_argument('--law-file');p.add_argument('--geometry',default=str(H));p.add_argument('--g0-duration',type=float,default=T0);a=p.parse_args();H=Path(a.geometry);T0=a.g0_duration;out=Path(a.out);out.mkdir(parents=True,exist_ok=False);log={'status':'FAIL','collision':'NOT_ATTEMPTED'};started=time.perf_counter()
 try:
  parent,path=load_parent(H);geo=json.loads((H/'geometry.json').read_text());lim=limits_for(parent);block=thaw(geo);verify_geometry(block,path,geo['construction'])
  
  if a.law_file:
   chosen=json.loads(Path(a.law_file).read_text());k=np.array(chosen['nodes']);x=np.array(chosen['lambda_values'])*a.scale
  else:
   p0=json.loads((EVID/'search_results.json').read_text());pilot=json.loads((EVID/'detailed_audit.json').read_text());k=np.array(p0['control_nodes']);x=np.array(pilot['cases']['continuous_with_jerk_uniform_scaled']['lambda_nodes'])*a.scale
  if geo['contract_version']!=4 or np.any(x<.25) or np.any(x>4):raise ValueError('INPUT_OR_LAMBDA_INVALID')
  t_start=time.perf_counter();records=[];allcuts=[];seam=[]
  for ci,c in enumerate(block.charts):
   cuts=cuts_for(c,geo['construction']['progress'],k);allcuts.append(cuts)
   ss=np.unique(np.concatenate([np.linspace(lo,hi,129) for lo,hi in zip(cuts[:-1],cuts[1:])]))
   t,u,q,v,acc,j=eval_chart(c,ss,k,x);records.append((ss,t,u,q,v,acc,j,np.full(len(ss),ci,int)))
  s,t,u,q,v,acc,j,ci=[np.concatenate([r[z] for r in records]) for z in range(8)]
  order=np.argsort(t,kind='stable');s,t,u,q,v,acc,j,ci=[z[order] for z in (s,t,u,q,v,acc,j,ci)];_,ridx=np.unique(t[::-1],return_index=True);idx=np.sort(len(t)-1-ridx);s,t,u,q,v,acc,j,ci=[z[idx] for z in (s,t,u,q,v,acc,j,ci)]
  if not np.all(np.diff(t)>0):raise ValueError('TIME_NOT_INCREASING')
  for left,right in zip(block.charts[:-1],block.charts[1:]):
   ss=_inverse_quintic(left.s1)*T0;L=eval_chart(left,[ss],k,x);R=eval_chart(right,[ss],k,x);err=[float(np.max(abs(L[z]-R[z]))) for z in (2,3,4)];seam.append(dict(u=left.s1,s=ss,t=float(L[0][0]),errors=err,jerk_left=L[5][0].tolist(),jerk_right=R[5][0].tolist()))
  if max(max(z['errors']) for z in seam)>1e-6:raise ValueError('C2_SEAM_FAILURE')
  mode=np.array([next(i for i,m in enumerate(geo['construction']['modes']) if m['u0']-1e-14<=(block.charts[int(c)].s0+block.charts[int(c)].s1)/2<=m['u1']+1e-14) for c in ci]);
  rec=TrajectoryRecord(path.trial_id,block.block_id,PhaseType.TRANSLATE,path.planner_id,path.parent_path_hash,path.schema,t,q,v,acc,j,'G1 saved density')
  check=validate(rec,lim,block.start,block.end);codes=nonintegration_failures(check.failure_codes);check_workspace(parent,rec)
  if codes:raise ValueError('NUMERIC_CHECK_FAIL '+repr(codes))
  if np.any(np.max(abs(v),axis=0)>.5+1e-12) or np.any(np.max(abs(acc),axis=0)>1+1e-12):raise ValueError('NINE_DOF_LIMIT_FAIL')
  kin=differential_kinematics(q,v,acc,float(lim.wheel_radius.value),float(lim.wheel_separation.value));lateral=float(np.max(abs(kin['lateral'])))
  for ix,c in enumerate(block.charts):
   mask=(ci==ix)&(u>c.s0+1e-12)&(u<c.s1-1e-12)
   if getattr(c,'stationary',False):
    if np.max(abs(v[ci==ix,:2]))>1e-10:raise ValueError('ADJUST_XY_DRIFT')
   elif np.any(kin['forward'][mask]<=0):raise ValueError('MOVE_FORWARD_FAIL')
  # Re-derive the saved jet via u derivatives, an independent chain-rule path.
  _,L,D,E=law(s,k,x);_,ud,udd,uj=quintic_time_jet(s,T0);uc=ud/L;ucc=udd/L**2-ud*D/L**3;uccc=uj/L**3-3*udd*D/L**4-ud*E/L**4+3*ud*D**2/L**5
  checkjet=block.evaluate(u);v2,a2,j2=chain_rule(checkjet,uc,ucc,uccc);cross=[float(np.max(abs(z-z2))) for z,z2 in zip((v,acc,j),(v2,a2,j2))]
  if max(cross)>1e-8:raise ValueError('CHAIN_RULE_MISMATCH '+repr(cross))
  compute_s=time.perf_counter()-t_start
  # Independent split quadrature, all chart/parent/geometry/time-law events.
  vstart=accstart=qstart=None;sumq=np.zeros(9);sumv=np.zeros(9);suma=np.zeros(9);worst=np.zeros(3);gap=0.;span_count=0;jerk_abs=np.zeros(9);jerk_sq=np.zeros(9)
  for c,cuts in zip(block.charts,allcuts):
   vals,g=quad(c,cuts,k,x);gap=max(gap,g)
   _,_,qe,ve,ae,je=eval_chart(c,cuts,k,x)
   if qstart is None:qstart=qe[0];vstart=ve[0];accstart=ae[0]
   for i in range(len(cuts)-1):
    iq,iv,ia,ij,ij2=(z[i] for z in vals);local=[qe[i+1]-qe[i]-iq,ve[i+1]-ve[i]-iv,ae[i+1]-ae[i]-ia];sumq+=iq;sumv+=iv;suma+=ia
    errors=local+[qe[i+1]-qstart-sumq,ve[i+1]-vstart-sumv,ae[i+1]-accstart-suma];worst=np.maximum(worst,[max(np.max(abs(errors[z])),np.max(abs(errors[z+3]))) for z in range(3)])
    jerk_abs+=ij;jerk_sq+=ij2;span_count+=1
  if gap>1e-8:raise ValueError('QUADRATURE_UNRESOLVED '+str(gap))
  if np.any(worst>np.array([1e-3,1e-3,1e-2])):raise ValueError('PIECEWISE_INTEGRATION_FAIL '+repr(worst.tolist()))
  validation_s=time.perf_counter()-t_start-compute_s
  result=dict(status='INDEPENDENT_NUMERICAL_PASS',collision='NOT_ATTEMPTED',duration_s=float(t[-1]),scale=a.scale,velocity_peak=np.max(abs(v),axis=0).tolist(),acceleration_peak=np.max(abs(acc),axis=0).tolist(),jerk_peak=np.maximum(np.max(abs(j),axis=0),np.max(np.abs(np.array([z[k] for z in seam for k in ('jerk_left','jerk_right')])),axis=0)).tolist(),jerk_mean=(jerk_abs/t[-1]).tolist(),jerk_rms=np.sqrt(jerk_sq/t[-1]).tolist(),lateral_peak=lateral,chain_rule_error=cross,seams=seam,piecewise_errors=worst.tolist(),quadrature_gap=gap,spans=span_count,samples=len(t),old_validator_nonintegration_failures=codes,compute_s=compute_s,numerical_validation_s=validation_s)
  es=time.perf_counter();matrix=np.column_stack([t,q,v,acc,j,u,s,ci,mode]);np.savetxt(out/'trajectory.csv',matrix,delimiter=',',fmt='%.17g',header=','.join(HEADER)+',u,g0_time_s,chart_index,mode_index',comments='');(out/'time_law.json').write_text(json.dumps(dict(g0_duration_s=T0,nodes=k.tolist(),lambda_values=x.tolist(),scale=a.scale,modes=geo['construction']['modes']),indent=2));result['export_s']=time.perf_counter()-es
  # output itself may be consumed by independent numerical/collision runner
  (out/'result.json').write_text(json.dumps(result,indent=2));print('NUMERIC_PASS',str(out),'T',t[-1],'samples',len(t),'errors',worst,'quad',gap,flush=True);return 0
 except Exception as e:
  log.update(error=repr(e),elapsed_s=time.perf_counter()-started);(out/'error.log').write_text(traceback.format_exc());(out/'result.json').write_text(json.dumps(log,indent=2));print('FAIL',repr(e),flush=True);return 1
if __name__=='__main__':sys.exit(main())
