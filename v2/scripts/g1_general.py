"""Generic fixed-H G1 time/jerk search. Numeric candidates require new collision PASS."""
import argparse,datetime,json,os,subprocess,sys,time,traceback
from pathlib import Path
import numpy as np
from ..timing import g1_general as opt
from ..scripts import g1_p01_finalize as fin
from ..scripts.stage_v2 import load_parent,limits_for
from ..geometry.tm_rho_feedback_frozen import thaw
from ..geometry.tm_frozen import verify_geometry

ROOT=Path(__file__).resolve().parents[2]
EVID=ROOT.parents[2]/'research_management/evidence/R5_V2_TM_G_PIECEWISE'

def checked_g0(h,g0,allow_deferred_collision=False):
 if (h/'geometry.json').read_bytes()!=(g0.parent/'geometry.json').read_bytes():raise ValueError('G0_GEOMETRY_MISMATCH')
 r=json.loads((g0/'result.json').read_text());T=float(r['duration_s'])
 if r.get('status')!='FULL_G_PASS' and not (allow_deferred_collision and r.get('status')=='NUMERICAL_PASS_COLLISION_PENDING' and r.get('full_g')=='PENDING_COLLISION'):
  prefix=g0.parent.name;ni=EVID/(prefix+'_independent.json');co=EVID/(prefix+'_collision.json')
  if not ni.is_file() or not co.is_file():raise ValueError('G0_INDEPENDENT_EVIDENCE_MISSING')
  if json.loads(ni.read_text()).get('status') not in ('NUMERIC_INDEPENDENT_PASS','PASS') or json.loads(co.read_text()).get('status')!='PASS':raise ValueError('G0_NOT_VALIDATED')
 return T,r

def run_process(cmd):
 p=subprocess.run(cmd,cwd=str(ROOT),capture_output=True,text=True);print(p.stdout,flush=True)
 if p.returncode:print(p.stderr,flush=True)
 return dict(command=cmd,returncode=p.returncode,stdout=p.stdout,stderr=p.stderr)

def high_jerk(csv,result):
 data=np.loadtxt(str(csv),delimiter=',',skiprows=1,usecols=[0,*range(28,37)]); t=data[:,0];j=data[:,1:];out=[]
 # Include one-sided seam jets at the same timestamp; no bridge across jumps.
 seams=result.get('seams',[])
 for z in seams:
  tt=z['time_s'] if 'time_s' in z else z['t'];i=np.searchsorted(t,tt);t=np.insert(t,i,tt);j=np.insert(j,i,np.array(z['jerk_left']),axis=0)
 for k in range(9):
  intervals=[]
  for i in range(len(t)-1):
   dt=t[i+1]-t[i]
   if dt<=0:continue
   a=j[i,k];b=j[i+1,k];cuts=[0.,1.]
   if a!=b:
    for threshold in (-4.,4.):
     z=(threshold-a)/(b-a)
     if 0<z<1:cuts.append(float(z))
   cuts.sort()
   for q0,q1 in zip(cuts[:-1],cuts[1:]):
    if abs(a+(b-a)*(q0+q1)/2)<4:continue
    lo=t[i]+q0*dt;hi=t[i]+q1*dt
    if intervals and lo-intervals[-1][1]<1e-9:intervals[-1][1]=hi
    else:intervals.append([float(lo),float(hi)])
  duration=sum(b-a for a,b in intervals);out.append(dict(duration_s=duration,time_fraction=duration/t[-1],clusters=len(intervals)))
 return out

def stage_all(args):
 h=Path(args.geometry).resolve();g0=Path(args.g0).resolve();T,g0r=checked_g0(h,g0,getattr(args,'allow_deferred_g0_collision',False));parent,path=load_parent(h);geo=json.loads((h/'geometry.json').read_text());block=thaw(geo);verify_geometry(block,path,geo['construction']);lim=limits_for(parent)
 base_law=None
 if g0r.get('time_law_type')=='NESTED_C_V1':
  from ..timing.nested_time import NestedBase
  base_law=NestedBase(g0r.get('time_law_arrays',g0r.get('time_law_npz')))
  if abs(base_law.T-T)>1e-6:raise ValueError('G0_TIME_LAW_DURATION_MISMATCH')
 profile=lim.metadata.get('constraint_profile',{}); assert profile.get('identity')=='V2_NINE_DOF_VA_ONLY','G1_LIMIT_CONTRACT'
 out=Path(args.out).resolve();out.mkdir(parents=True,exist_ok=False);summary=dict(status='SEARCH_STARTED',geometry=str(h),g0=str(g0),g0_time_s=T,search=[],stages={},fallback='G0',h_build_s=geo['construction'].get('h_build_s'),g0_compute_s=g0r.get('compute_s'));started=time.perf_counter()
 try:
  opt.base.T0=T;raw=opt.raw_cache(block,geo,T,5,base_law);k=opt.control_grid(raw,T);d=opt.prep(raw,k);fine=opt.prep(opt.raw_cache(block,geo,T,33,base_law),k);ones=np.ones(len(k));
  # A starts at exact G0; solver failures do not replace the baseline.
  afit,alog=opt.time_search(ones,d);summary['search'].append(alog);ax,adiag=opt.scale_safe(np.asarray(afit.x),fine);summary['stages']['A']=adiag
  if ax is None or float(d['weights']@ax)>=T:
   ax=ones.copy();summary['stages']['A']['fallback']='G0'
  choices=[('G0',ones.copy(),opt.evaluate_candidate(ones,fine)),('A',ax.copy(),opt.evaluate_candidate(ax,fine))]
  # B jointly trades jerk and time; per-task envelope is a search heuristic only.
  bfit,blog=opt.smooth_search(ax,d,float(d['weights']@ax));summary['search'].append(blog);bx,bdiag=opt.scale_safe(np.asarray(bfit.x[:-1]),fine,False);summary['stages']['B']=bdiag
  if bx is None:bx=ax.copy();summary['stages']['B']['fallback']='A'
  bm=opt.evaluate_candidate(bx,fine);choices.append(('B',bx.copy(),bm))
  # Exactly one time-recovery optimization, only once sampled jerk target is met.
  if max(bm['jerk'])<=5:
   cfit,clog=opt.time_search(bx,d,True);summary['search'].append(clog);cx,cdiag=opt.scale_safe(np.asarray(cfit.x),fine,True);summary['stages']['C']=cdiag
   if cx is not None and float(d['weights']@cx)+.1<float(d['weights']@bx):choices.append(('C',cx.copy(),opt.evaluate_candidate(cx,fine)))
  else:summary['stages']['C']={'reason':'JERK_REFERENCE_NOT_MET'}
  fastest=min(choices,key=lambda p:p[2]['T']);good=[p for p in choices if max(p[2]['jerk'])<=5]
  if good:
   near=[p for p in good if p[2]['T']<=min(q[2]['T'] for q in good)*1.005]
   def quality(p):
    _,_,j,L,_,_=opt.evaluate(p[1],fine);order=np.argsort(fine['s'],kind='stable');ss=fine['s'][order];JJ=abs(j[order]);dt=np.diff(ss)*(L[order][:-1]+L[order][1:])/2;means=np.sum(np.diff(ss)[:,None]*(JJ[:-1]*L[order][:-1,None]+JJ[1:]*L[order][1:,None])/2,axis=0)/p[2]['T'];high=np.sum(dt[:,None]*(JJ[:-1]>=4),axis=0)/p[2]['T'];return (float(np.mean(means/5)),float(np.max(high)),p[2]['T'])
   recommended=min(near,key=quality)
  else:recommended=min(choices,key=lambda p:(max(p[2]['jerk'])/5,p[2]['T']))
  summary['selected_sources']=dict(fast=fastest[0],recommended=recommended[0]);summary['search_total_s']=time.perf_counter()-started;summary['knots']=k.tolist()
  for name,x in [('fast',fastest[1]),('recommended',recommended[1])]:
   law=out/(name+'_law.json');law.write_text(json.dumps(dict(nodes=k.tolist(),lambda_values=x.tolist(),g0_duration_s=T),indent=2))
   if base_law is not None:
    from .g1_signed_output import export_numeric
    details=export_numeric(out/name,block,path,geo,base_law,k,x,lim)
    summary['stages'][name+'_numeric']=details['status'];summary['stages'][name]=details
    continue
   cmd=[sys.executable,'-B','-m','v2.scripts.g1_p01_finalize','--out',str(out/name),'--geometry',str(h),'--g0-duration',str(T),'--law-file',str(law)]
   result=run_process(cmd);summary['stages'][name]=result
   if result['returncode']==0:
    ir=run_process([sys.executable,'-B','-m','v2.scripts.g1_p01_independent','--input',str(out/name),'--geometry',str(h)]);summary['stages'][name+'_independent']=ir
    if ir['returncode']==0:
     r=json.loads((out/name/'result.json').read_text());r['high_jerk']=high_jerk(out/name/'trajectory.csv',r);(out/name/'high_jerk.json').write_text(json.dumps(r['high_jerk'],indent=2));summary['stages'][name+'_numeric']='PASS'
  summary['status']=('NUMERIC_ONLY_COLLISION_PENDING'
                     if all(summary['stages'].get(n+'_numeric')=='PASS' for n in ('fast','recommended'))
                     else 'PARTIAL_NUMERIC_FAILURE')
 except Exception as e:summary.update(status='FAIL',error=repr(e));(out/'error.log').write_text(traceback.format_exc())
 (out/'summary.json').write_text(json.dumps(summary,indent=2));return out,summary

def collision(out,h):
 result={}
 for name in ('fast','recommended'):
  p=out/name
  if not (p/'independent_export.json').exists():continue
  cmd=[sys.executable,'-B','-m','v2.scripts.g1_p01_collision','--input',str(p),'--geometry',str(h)]
  result[name]=run_process(cmd)
 return result

def main():
 p=argparse.ArgumentParser(description='Frozen-H G1: finite time + smooth search; no replanning')
 p.add_argument('--geometry',required=True);p.add_argument('--g0',required=True);p.add_argument('--out',required=True);p.add_argument('--scene-snapshot',default=str(ROOT/'v2/outputs/runs/run_20261023T165050Z_548ef707/preflight_recovered/scene_snapshot.bin'));p.add_argument('--numeric-only',action='store_true');p.add_argument('--allow-deferred-g0-collision',action='store_true');a=p.parse_args()
 if a.allow_deferred_g0_collision and not a.numeric_only:p.error('--allow-deferred-g0-collision requires --numeric-only')
 if not a.numeric_only and json.loads((Path(a.g0)/'result.json').read_text()).get('time_law_type')=='NESTED_C_V1':p.error('NESTED_C_V1 currently supports --numeric-only; collision/export integration is not yet implemented')
 out,s=stage_all(a)
 if s['status']!='NUMERIC_ONLY_COLLISION_PENDING':return 1
 if a.numeric_only:print('COLLISION_PENDING',out);return 0
 # Reuse owned runtime only; scripts refuse unowned/incompatible services.
 status=run_process([str(ROOT/'v2/scripts/status_v2.sh')]);state=json.loads(status['stdout'].strip().splitlines()[-1])['status'];owned=False
 try:
  if state=='STOPPED':
   x=run_process([str(ROOT/'v2/scripts/start_v2.sh'),'--config',str(ROOT/'v2/configs/v2_workspace_single_goal_9d_va_only.yaml')]);owned=True
   if x['returncode']:raise RuntimeError('ROS_START_FAILED')
  elif state!='READY':raise RuntimeError('ROS_RUNTIME_'+state)
  # subprocess inherits explicit clean Noetic environment via existing setup script.
  command='source v2/scripts/env_noetic.sh >/dev/null; exec "$@"'
  env=dict(os.environ,V2_MASTER_URI='http://127.0.0.1:11411')
  for name in ('fast','recommended'):
   if s['stages'].get(name+'_numeric')!='PASS':continue
   cmd=['bash','--noprofile','--norc','-c',command,'_',sys.executable,'-B','-m','v2.scripts.g1_p01_collision','--input',str(out/name),'--geometry',str(Path(a.geometry).resolve()),'--scene-snapshot',str(Path(a.scene_snapshot).resolve())]
   q=subprocess.run(cmd,cwd=str(ROOT),env=env,capture_output=True,text=True);print(q.stdout,flush=True);s['stages'][name+'_collision']=dict(returncode=q.returncode,stdout=q.stdout,stderr=q.stderr)
  s['status']='PASS' if all(s['stages'].get(n+'_collision',{}).get('returncode')==0 for n in ('fast','recommended')) else 'PARTIAL_COLLISION_OR_NUMERIC_FAILURE'
 except Exception as e:s.update(status='COLLISION_UNKNOWN',collision_error=repr(e))
 finally:
  if owned:s['runtime_stop']=run_process([str(ROOT/'v2/scripts/stop_v2.sh')])
 (out/'summary.json').write_text(json.dumps(s,indent=2));print('G1_RESULT',out,s['status'],flush=True);return 0 if s['status']=='PASS' else 1
if __name__=='__main__':raise SystemExit(main())
