"""预求五目标并对前两段执行现有 PHG1；仅生成参考轨迹。"""
import argparse,copy,csv,datetime,json,math,os,sys,time,traceback,shutil
from pathlib import Path
from unittest.mock import patch
import numpy as np
import yaml
from . import retry_phg1,stage_v2
from .joint_ik import JointIK

ROOT=Path(__file__).resolve().parents[2]
NAMES=stage_v2.NAMES

def write(path,obj):Path(path).write_text(json.dumps(obj,indent=2,ensure_ascii=False))
def read_csv(path):
 with Path(path).open(newline='') as f:
  r=csv.DictReader(f);header=r.fieldnames;rows=list(r)
 if not rows or not header:raise ValueError('EMPTY_TRAJECTORY')
 return header,rows

def qva(row):
 return [np.array([float(row[p+n]) for n in NAMES]) for p in ('q_','v_','a_')]

def combine(first,second,dest,dwell=1.):
 h,a=read_csv(first);h2,b=read_csv(second)
 if h!=h2:raise ValueError('CSV_SCHEMA_MISMATCH')
 qa,va,aa=qva(a[-1]);qb,vb,ab=qva(b[0]);dq=qa-qb;dq[2]=math.remainder(dq[2],2*math.pi)
 seam=dict(q_error=dq.tolist(),v_before=va.tolist(),v_after=vb.tolist(),a_before=aa.tolist(),a_after=ab.tolist(),position_max=float(max(abs(dq))),velocity_max=float(max(np.max(abs(va)),np.max(abs(vb)))),acceleration_max=float(max(np.max(abs(aa)),np.max(abs(ab)))))
 if seam['position_max']>1e-8 or seam['velocity_max']>1e-8 or seam['acceleration_max']>1e-8:raise ValueError('STOP_SEAM_NOT_PASS '+json.dumps(seam))
 if abs(float(a[0]['time_s']))>1e-9 or abs(float(b[0]['time_s']))>1e-9:raise ValueError('TIME_ORIGIN')
 ta=float(a[-1]['time_s']);offset=ta+dwell
 # A single hold midpoint and the original second segment start delimit a 1 s dwell;
 # both motion segments retain every q/v/a/jerk field, only time is offset.
 hold=dict(a[-1]);hold['time_s']=format(ta+dwell/2,'.17g')
 for prefix in ('v_','a_','jerk_'):
  for n in NAMES:hold[prefix+n]='0'
 out=a+[hold]
 for row in b:
  row=dict(row);row['time_s']=format(float(row['time_s'])+offset,'.17g');out.append(row)
 t=np.array([float(r['time_s']) for r in out]);
 if np.any(np.diff(t)<=0):raise ValueError('TIME_NOT_INCREASING')
 for rows in (a,b):
  for row in rows:
   q,v,acc=qva(row); lateral=-math.sin(q[2])*v[0]+math.cos(q[2])*v[1];forward=math.cos(q[2])*v[0]+math.sin(q[2])*v[1]
   if np.max(abs(v))>.5+1e-9 or np.max(abs(acc))>1+1e-9 or abs(lateral)>1e-6 or forward < -1e-8:raise ValueError('COMBINED_SAMPLE_LIMIT_OR_NONHOLONOMIC')
 with Path(dest).open('x',newline='') as f:
  w=csv.DictWriter(f,fieldnames=h);w.writeheader();w.writerows(out)
 return dict(status='PASS',seam=seam,dwell_s=dwell,rows=len(out),duration_s=float(t[-1]),bytes=Path(dest).stat().st_size,motion_rows_preserved=True,transition_gap_s=2e-9,scope='display only; holds are constant for 1s, post-hold tolerance gap is not a continuously verified trajectory; dense per-segment collision evidence remains authoritative')

def combine_all(paths,dest,dwell=1.):
 allrows=[];seams=[];intervals=[];previous=None;offset=0.;header=None
 for i,path in enumerate(paths):
  h,rows=read_csv(path)
  if header is None:header=h
  if h!=header:raise ValueError('CSV_SCHEMA_MISMATCH')
  if abs(float(rows[0]['time_s']))>1e-9:raise ValueError('TIME_ORIGIN')
  q0,v0,a0=qva(rows[0]);qe,ve,ae=qva(rows[-1])
  if max(np.max(abs(v0)),np.max(abs(a0)),np.max(abs(ve)),np.max(abs(ae)))>1e-8:raise ValueError('STOP_BOUNDARY_NOT_PASS segment=%d'%(i+1))
  if previous is not None:
   qp,vp,ap=previous;dq=qp-q0;dq[2]=math.remainder(dq[2],2*math.pi)
   seam=dict(after_segment=i,q_diff=dq.tolist(),q_max=float(np.max(abs(dq))),v_diff=(vp-v0).tolist(),a_diff=(ap-a0).tolist(),v_max=float(max(np.max(abs(vp)),np.max(abs(v0)))),a_max=float(max(np.max(abs(ap)),np.max(abs(a0)))))
   seam['tolerance_xy_m']=1e-4;seam['tolerance_angular_rad']=1e-4
   if np.any(abs(dq[:2])>1e-4) or np.any(abs(dq[2:])>1e-4):raise ValueError('POSITION_SEAM_NOT_PASS '+repr(seam))
   seams.append(seam)
  start=offset;end=start+float(rows[-1]['time_s'])
  for original in rows:
   q,v,acc=qva(original);lateral=-math.sin(q[2])*v[0]+math.cos(q[2])*v[1];forward=math.cos(q[2])*v[0]+math.sin(q[2])*v[1]
   if np.max(abs(v))>.5+1e-9 or np.max(abs(acc))>1+1e-9 or abs(lateral)>1e-6 or forward < -1e-8:raise ValueError('COMBINED_SAMPLE_LIMIT_OR_NONHOLONOMIC')
   r=dict(original);r['time_s']=format(float(r['time_s'])+offset,'.17g');allrows.append(r)
  # Retain all original motion rows and add a stationary sample. Final hold
  # ends at +1 s; between segments next motion begins at +1 s.
  hold=dict(rows[-1]);hold['time_s']=format(end+dwell,'.17g')
  for prefix in ('v_','a_','jerk_'):
   for n in NAMES:hold[prefix+n]='0'
  allrows.append(hold);intervals.append(dict(segment=i+1,motion_start_s=start,motion_end_s=end,dwell_start_s=end,dwell_end_s=end+dwell));offset=end+dwell+(2e-9 if i<len(paths)-1 else 0.);previous=(qe,ve,ae)
 t=np.array([float(r['time_s']) for r in allrows])
 if np.any(np.diff(t)<=0):raise ValueError('TIME_NOT_INCREASING')
 with Path(dest).open('x',newline='') as f:
  w=csv.DictWriter(f,fieldnames=header);w.writeheader();w.writerows(allrows)
 return dict(status='PASS',seams=seams,intervals=intervals,rows=len(allrows),duration_s=float(t[-1]),bytes=Path(dest).stat().st_size,motion_rows_preserved=True,transition_gap_s=2e-9,scope='display only; holds are constant for 1s, post-hold tolerance gap is not a continuously verified trajectory; dense per-segment collision evidence remains authoritative')

def main():
 p=argparse.ArgumentParser();p.add_argument('--config',default='v2/configs/multi_5goal.yaml');p.add_argument('--resume',help='previous two-segment multi task');p.add_argument('--third-attempts',type=int,default=3,choices=range(1,11));a=p.parse_args();cfg=yaml.safe_load(Path(a.config).read_text());spec=cfg['multi_goal'];base=cfg['tasks'][0]
 if len(spec['targets'])!=5 or spec['dwell_s']!=1 or spec['max_attempts']!=3 or spec['position_tolerance_m']!=.01 or spec['orientation_tolerance_rad']!=.01:raise ValueError('UNSUPPORTED_MULTI_CONTRACT')
 dt=datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).strftime('%Y%m%d-%H%M%S');root=ROOT/'v2/outputs'/('V2-'+dt+'-MULTI-IK-PHG1-RRTstar')
 for i in range(1,1000):
  try:out=Path(str(root)+(('-%02d'%i) if i>1 else ''));out.mkdir();break
  except FileExistsError:continue
 else:raise ValueError('OUTPUT_EXISTS')
 started=time.perf_counter();result={'status':'FAIL','phase':'INITIAL','targets':[],'segments':[]};write(out/'config.json',cfg);(out/'IK').mkdir()
 try:
  expected=stage_v2.context(cfg);start=base['start_q'];solvers=[]
  if a.resume:
   old=Path(a.resume).resolve(strict=True);oldcfg=json.loads((old/'config.json').read_text());oldresult=json.loads((old/'result.json').read_text())
   if oldcfg!=cfg:raise ValueError('RESUME_CONFIG_MISMATCH')
   for i in (1,2):
    seg=old/('segment_%02d'%i);summary=json.loads((seg/'summary.json').read_text());record=json.loads((seg/'result.json').read_text());parent=json.loads((seg/'P/parent_path.json').read_text());collision=json.loads((seg/'g1/recommended/collision.json').read_text())
    if summary.get('status')!='PASS' or record.get('status')!='PASS' or collision.get('status')!='PASS' or parent['scene']['scene_sha256']!=expected['scene_sha256']:raise ValueError('RESUME_SEGMENT_OR_SCENE_MISMATCH')
    shutil.copytree(seg,out/seg.name);result['segments'].append(dict(segment=i,status='PASS',reused=True,source=str(seg),attempts=summary['total_attempts'],failures=summary['failures']))
   _,rows=read_csv(old/'segment_02/g1/recommended/trajectory.csv');start=qva(rows[-1])[0].tolist()
  for target in spec['targets']:
   result['phase']='IK_'+target['id'];print('PRE_IK',target['id'],flush=True);solver=JointIK(cfg,target)
   if a.resume:
    r=json.loads((old/'IK'/('target_'+target['id']+'.json')).read_text());check=solver.check(r['q'])
    if not r.get('ik_feasible') or not check['ik_feasible'] or r.get('target')!=solver.target():raise ValueError('RESUME_IK_MISMATCH_OR_INVALID')
    r['resume_revalidation']=check
   else:r=solver.solve(start,1);r['target']=solver.target()
   write(out/'IK'/('target_'+target['id']+'.json'),r);result['targets'].append(dict(id=target['id'],ik_feasible=r['ik_feasible'],position_error_m=r['position_error_m'],orientation_error_rad=r['orientation_error_rad']))
   if not r['ik_feasible']:raise ValueError('IK_NOT_FEASIBLE')
   solvers.append((solver,r))
  if a.resume:
   # Do not spend new planning attempts if retained samples cannot define
   # an exactly stationary hold without editing an existing motion row.
   _,left=read_csv(out/'segment_01/g1/recommended/trajectory.csv');_,right=read_csv(out/'segment_02/g1/recommended/trajectory.csv')
   q1,v1,a1=qva(left[-1]);q2,v2,a2=qva(right[0]);delta=q1-q2;delta[2]=math.remainder(delta[2],2*math.pi)
   seamcheck=dict(q_diff=delta.tolist(),tolerance_xy_m=1e-4,tolerance_angular_rad=1e-4,max_xy_m=float(np.max(abs(delta[:2]))),max_angular_rad=float(np.max(abs(delta[2:]))),velocity_max=float(max(np.max(abs(v1)),np.max(abs(v2)))),acceleration_max=float(max(np.max(abs(a1)),np.max(abs(a2)))))
   result['reused_seam_check']=seamcheck
   if seamcheck['max_xy_m']>1e-4 or seamcheck['max_angular_rad']>1e-4 or seamcheck['velocity_max']>1e-8 or seamcheck['acceleration_max']>1e-8:
    result['phase']='REUSED_HOLD_PRECHECK';raise ValueError('REUSED_HOLD_NOT_CONTINUOUS '+repr(seamcheck))
  for i,(solver,pre) in enumerate(solvers,1):
   if a.resume and i<=2:continue
   if stage_v2.context(cfg)['scene_sha256']!=expected['scene_sha256']:raise ValueError('SCENE_CHANGED_SINCE_PRE_IK')
   task=copy.deepcopy(base);task.update(id='multi_goal_%d'%i,start_q=list(start),pose_goal=solver.target());task['pose_goal'].pop('position_tolerance_m',None);task['pose_goal'].pop('orientation_tolerance_rad',None)
   class FrozenIK:
    def __init__(self,*args):pass
    def solve(self,*args):
     r=copy.deepcopy(pre);r.update(solver.check(pre['q']));r['source']='PRECOMPUTED_IK_REVALIDATED';return r
    def check(self,q):return solver.check(q)
    def target(self):return solver.target()
   def new_dir(config,task,suffix=''):
    dest=out/('segment_%02d'%i);dest.mkdir();return dest
   result['phase']='SEGMENT_%d'%i;print('SEGMENT_START',i,flush=True);t=time.perf_counter()
   with patch.object(stage_v2,'new_dir',new_dir),patch('v2.scripts.joint_ik.JointIK',FrozenIK):seg=retry_phg1.run(cfg,task,['plan_v2_multi'],max_attempts=(a.third_attempts if i==3 else 3))
   s=json.loads((seg/'summary.json').read_text());entry=dict(segment=i,status=s['status'],attempts=s['total_attempts'],failures=s['failures'],wall_s=time.perf_counter()-t);result['segments'].append(entry);write(out/'result.json',result)
   if s['status']!='PASS':result['phase']='SEGMENT_%d'%i;raise ValueError('SEGMENT_NOT_PASS')
   record=json.loads((seg/'result.json').read_text());entry['final_pose_check']=record.get('final_pose_check');_,rows=read_csv(seg/'g1/recommended/trajectory.csv');start=qva(rows[-1])[0].tolist();entry['actual_end_q']=start
  result['phase']='COMBINE';folder=out/'task';folder.mkdir();result['combined']=combine_all([out/('segment_%02d'%i)/'g1/recommended/trajectory.csv' for i in range(1,6)],folder/'trajectory.csv',spec['dwell_s']);write(folder/'result.json',{'status':'TASK_PASS','validation_scope':'display only; per-segment dense trajectory acceptance','source_segment':'segment_01'});result.update(status='TASK_PASS',phase='COMPLETE')
 except Exception as e:
  result['error']=repr(e);(out/'error.log').write_text(traceback.format_exc());print('MULTI_FAIL',repr(e),flush=True)
 finally:
  result['wall_s']=time.perf_counter()-started;write(out/'result.json',result);print('MULTI_RESULT',out,result['status'],flush=True)
 return 0 if result['status']=='TASK_PASS' else 1
if __name__=='__main__':sys.exit(main())
