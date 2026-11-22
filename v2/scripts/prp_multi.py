"""Sequential five-goal orchestration using the existing single-goal PRP runner."""
import copy,datetime,json,math,os,time,traceback,shutil
from pathlib import Path
import numpy as np
import yaml
from . import prp_global,prp_compact

def retryable_failure(status,error,collision):
 """Only a confirmed collision failure can justify replanning for collision."""
 if status=='COLLISION_NOT_PASS' and collision.get('status')!='FAIL':return False
 return status in ('R_FAIL','COLLISION_NOT_PASS','DOWNSTREAM_FAIL','FINAL_TARGET_FK_FAIL') or any(x in error for x in ('IK_POSTERIOR_NOT_FEASIBLE','P_ENDPOINT_TASK_FK_NOT_PASS','PLANNING_FAILED','MOVEIT_CAPTURE_FAILED'))

def write_task_csv(data,dest):
 from .stage_v2 import HEADER
 from .g1_display_sampling import select_storage_rows
 t=data['t'];q=data['q'];v=data['velocity'];a=data['acceleration'];j=data['jerk']
 if len(t)<2 or np.any(np.diff(t)<=0):raise ValueError('TASK_CSV_TIME_NOT_INCREASING')
 matrix=np.column_stack((t,q,v,a,j,data['segment'],data['chart'],data['direction']))
 # Preserve both sides of segment, chart and direction changes, including
 # arrival and departure at each hold. Rows remain the original dense samples.
 changes=np.flatnonzero(np.any(matrix[1:,37:40]!=matrix[:-1,37:40],axis=1))+1
 event_ix=np.unique(np.r_[0,len(t)-1,changes,np.maximum(changes-1,0)])
 events=np.unique(np.r_[t[event_ix],data.get('storage_events',[])])
 selector=np.column_stack((matrix[:,:37],np.zeros(len(t)),matrix[:,38],matrix[:,37],matrix[:,39]))
 ix=select_storage_rows(selector,events)
 np.savetxt(dest,matrix[ix],delimiter=',',fmt='%.17g',header=','.join(HEADER+['segment','chart_index','direction']),comments='')
 return len(ix)


def run(config_path,c):
 task=c['task'];goals=task.get('targets');start=task.get('start_q');dwell=task.get('dwell_s')
 if c['input'].get('stage')!='new' or c['input'].get('until')!='g1' or task.get('source')!='configured_goal' or not isinstance(goals,list) or len(goals)!=5 or dwell!=1.0:raise ValueError('MULTI_REQUIRES_FIVE_NEW_GOALS_AND_ONE_SECOND_DWELL')
 if not isinstance(start,list) or len(start)!=9 or not all(isinstance(x,(int,float)) and math.isfinite(x) for x in start):raise ValueError('INVALID_MULTI_START')
 for g in goals:
  if set(g)!={'id','type','frame','link','position_m','quaternion_xyzw'} or g['type']!='link_pose' or g['frame']!='world' or g['link']!='Link6':raise ValueError('UNSUPPORTED_MULTI_GOAL')
 reuse=task.get('reuse_segments',[])
 if not isinstance(reuse,list) or len(reuse)>4 or any(not isinstance(x,dict) or set(x)!={'directory','fk_record','target_record'} for x in reuse):raise ValueError('INVALID_REUSED_SEGMENTS')
 base=config_path.parent;root=prp_global._path(c['output']['root'],base);root.mkdir(parents=True,exist_ok=True);run=root/prp_global.experiment_name('multi',True,c['planner']['id']);run.mkdir();(run/'segments').mkdir()
 (run/'effective_config.yaml').write_text(yaml.safe_dump(c,allow_unicode=True,sort_keys=False));summary={'status':'PARTIAL','segments':[],'total_wall_s':None,'input_config':str(config_path),'goals':goals,'rho_policy':'22 then 36 fixed profile; bounded theta/rho recovery on exact infeasibility','dwell_s':1.0};t0=time.perf_counter();segments=[]
 def save():summary['total_wall_s']=time.perf_counter()-t0;(run/'result.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))
 try:
  for i,g in enumerate(goals,1):
   print('MULTI_SEGMENT_START',i,flush=True);st=time.perf_counter();descriptor=reuse[i-1] if i<=len(reuse) else None
   if descriptor:
    folder=prp_global._path(descriptor['directory'],base).resolve(strict=True);code=0
    # Reuse is allowed only for an explicitly selected, previously accepted
    # trajectory in the same current PlanningScene and for the same Link6 goal.
    saved_goal=json.loads(prp_global._path(descriptor['target_record'],base).read_text());expected=(g['position_m'],g['quaternion_xyzw'])
    actual=(saved_goal['position'],saved_goal['orientation_xyzw'])
    if not np.allclose(actual[0],expected[0],rtol=0,atol=1e-12) or min(np.max(np.abs(np.array(actual[1])-expected[1])),np.max(np.abs(np.array(actual[1])+expected[1])))>1e-12:raise RuntimeError('REUSE_TARGET_MISMATCH')
    snapshot=os.environ.get('V2_PRP_SCENE_SNAPSHOT');previous=folder/'result/PRP/scene_snapshot.bin'
    if not snapshot or not previous.is_file() or previous.read_bytes()!=Path(snapshot).read_bytes():raise RuntimeError('REUSE_SCENE_MISMATCH')
    entry={'segment':i,'target':g,'directory':str(folder),'wall_s':time.perf_counter()-st,'exit_code':0,'reused':True}
   else:
    cc=copy.deepcopy(c);cc['task']={'mode':'single','source':'configured_goal','start_q':start,'goal':{k:v for k,v in g.items() if k!='id'}};cc['output']['root']=str(run/'segments');cc['output']['plot']=False
    # Resolve paths relative to the original YAML, not the generated YAML.
    if not cc['scene']['octomap'].startswith('package://'):cc['scene']['octomap']=str(prp_global._path(cc['scene']['octomap'],base))
    if cc['scene'].get('snapshot'):cc['scene']['snapshot']=str(prp_global._path(cc['scene']['snapshot'],base))
    temp=run/('segment_%02d_config.yaml'%i);temp.write_text(yaml.safe_dump(cc,allow_unicode=True,sort_keys=False));attempts=[]
    for attempt in range(1,int(os.environ.get('V2_PRP_MAX_ATTEMPTS','6'))+1):
     ctl={'rho_fallback':36,'signed_recovery':True,'run_dir_override':str(run/'segments'/('segment_%02d'%i)/('attempt_%02d'%attempt))};code=prp_global.run(temp,ctl);folder=Path(ctl['run_dir'])
     result_file=folder/'result/result.json';data=json.loads(result_file.read_text()) if result_file.is_file() else {};error_file=folder/'error.log';error=error_file.read_text()[-3000:] if error_file.is_file() else ''
     status=data.get('status','NO_RESULT');attempts.append({'attempt':attempt,'directory':str(folder),'exit_code':code,'status':status,'error':data.get('error') or error})
     # Segment budget is not the task budget: retain a numerically and
     # collision-accepted candidate even when its CSV makes it exceed 30 MiB.
     sf=folder/'storage.json'
     if code and status=='FIXED_SIGNED_R_SINGLE_PASS' and sf.is_file() and json.loads(sf.read_text()).get('status')=='OUTPUT_BUDGET_EXCEEDED':
      attempts[-1]['segment_storage']='OUTPUT_BUDGET_EXCEEDED';code=0
     if code==0 and status=='FIXED_SIGNED_R_SINGLE_PASS':break
     # Retry only a new genuinely randomized P for candidate-dependent failures;
     # scene, runtime and interface errors must stop instead of being masked.
     collision_file=folder/'result/PRP/collision.json'
     collision=json.loads(collision_file.read_text()) if collision_file.is_file() else {}
     # An UNKNOWN check is a resource/scene/service failure, not evidence that
     # another random parent path will help. Keep this candidate and stop.
     if status=='COLLISION_NOT_PASS':attempts[-1]['collision_status']=collision.get('status','UNKNOWN')
     retry=retryable_failure(status,error,collision)
     if not retry:break
    entry={'segment':i,'target':g,'directory':str(folder),'wall_s':time.perf_counter()-st,'exit_code':code,'reused':False,'attempts':attempts}
   summary['segments'].append(entry)
   result=folder/'result/result.json';entry['result']=json.loads(result.read_text()) if result.is_file() else None
   pfile=folder/'result/P/parent_path.json'
   if pfile.is_file():
    parent=json.loads(pfile.read_text());entry['P']={'states':len(parent['path']['q']),'planning':parent['planning'],'start_q':parent['path']['q'][0],'end_q':parent['path']['q'][-1]}
   if code or not entry['result'] or entry['result'].get('status')!='FIXED_SIGNED_R_SINGLE_PASS':
    entry['status']='FAIL';summary['failure_segment']=i;summary['failure_stage_or_reason']=(entry['result'] or {}).get('error','single-stage failure; see error.log');save();break
   trajectory=folder/'result/PRP/P1/recommended/trajectory.npz';numeric=folder/'result/PRP/P1/recommended/result.json';fk=prp_global._path(descriptor['fk_record'],base) if descriptor else folder/'result/IK/final_fk.json';collision=folder/'result/PRP/collision.json'
   if not all(x.is_file() for x in (trajectory,numeric,fk,collision)):raise RuntimeError('MISSING_SEGMENT_FINAL_EVIDENCE')
   fkr=json.loads(fk.read_text());fkr=fkr.get('P1_final_fk',fkr);cr=json.loads(collision.read_text());nr=json.loads(numeric.read_text())
   if not fkr.get('ik_feasible') or cr.get('status')!='PASS' or nr.get('status')!='PASS':raise RuntimeError('SEGMENT_FINAL_EVIDENCE_FAIL')
   with np.load(trajectory,allow_pickle=False) as z: data={k:z[k] for k in ('t','u','q','velocity','acceleration','jerk','chart','direction')}
   # The segment display already contains all exporter time-law and chart
   # events. If it is absent, preserve every source row rather than inventing
   # event metadata for an older reusable segment.
   csv=folder/'result/PRP/P1/recommended/trajectory.csv'
   data['storage_events']=np.loadtxt(csv,delimiter=',',skiprows=1,usecols=[0],ndmin=1) if csv.is_file() else data['t']
   if not segments:
    initial=data['q'][0]-np.asarray(task['start_q']);initial[2]=math.remainder(float(initial[2]),2*math.pi)
    if np.max(np.abs(initial))>1e-10:raise RuntimeError('REUSE_TASK_START_MISMATCH')
   if segments:
    delta=data['q'][0]-segments[-1]['q'][-1];delta[2]=math.remainder(float(delta[2]),2*math.pi)
    sv=data['velocity'][0]-segments[-1]['velocity'][-1];sa=data['acceleration'][0]-segments[-1]['acceleration'][-1]
    seam={'q_max':float(np.max(np.abs(delta))),'v_max':float(np.max(np.abs(sv))),'a_max':float(np.max(np.abs(sa))),'q_diff':delta.tolist(),'v_diff':sv.tolist(),'a_diff':sa.tolist()};summary.setdefault('seams',[]).append(seam)
    if seam['q_max']>1e-10 or seam['v_max']>1e-8 or seam['a_max']>1e-8:raise RuntimeError('SEAM_CONTINUITY_NOT_PASS')
   for endpoint in (0,-1):
    if max(np.max(abs(data['velocity'][endpoint])),np.max(abs(data['acceleration'][endpoint])))>1e-8:raise RuntimeError('GOAL_STOP_NOT_PASS')
   entry.update(status='PASS',final_fk=fkr,collision=cr,numeric=nr);segments.append(data);start=data['q'][-1].tolist();save()
  if len(segments)==5:
   rows=[];offset=0.;events=[];storage_events=[];lasttheta=None
   for i,z in enumerate(segments):
    q=z['q'].copy();shift=0. if lasttheta is None else round((lasttheta-q[0,2])/(2*math.pi))*2*math.pi;q[:,2]+=shift;storage_events.extend((z['storage_events']+offset).tolist())
    # The previous 1 s hold already contains this equal zero-velocity start.
    sl=slice(None) if i==0 else slice(1,None);n=len(z['t'][sl]);end=offset+float(z['t'][-1]);part={'t':z['t'][sl]+offset,'q':q[sl],'velocity':z['velocity'][sl],'acceleration':z['acceleration'][sl],'jerk':z['jerk'][sl],'segment':np.full(n,i+1,dtype=np.int16),'chart':z['chart'][sl],'direction':z['direction'][sl]};rows.append(part)
    rows.append({'t':np.array([end+1.]),'q':q[-1:],'velocity':np.zeros((1,9)),'acceleration':np.zeros((1,9)),'jerk':np.zeros((1,9)),'segment':np.array([i+1],dtype=np.int16),'chart':z['chart'][-1:],'direction':np.array([0],dtype=np.int8)});events.append({'segment':i+1,'motion_start_s':offset,'motion_end_s':end,'dwell_end_s':end+1.});offset=end+1.;lasttheta=float(q[-1,2])
   allz={k:np.concatenate([r[k] for r in rows]) for k in rows[0]};t=allz['t'];q=allz['q'];v=allz['velocity'];a=allz['acceleration'];j=allz['jerk'];lat=-np.sin(q[:,2])*v[:,0]+np.cos(q[:,2])*v[:,1];lon=np.cos(q[:,2])*v[:,0]+np.sin(q[:,2])*v[:,1];direction=allz['direction'];tol=1e-7
   if np.any(np.diff(t)<=0) or np.max(abs(v))>.5+tol or np.max(abs(a))>1.+tol or np.max(abs(lat))>1e-6 or np.any(lon[direction>0]<-tol) or np.any(lon[direction<0]>tol) or np.any(abs(lon[direction==0])>tol):raise RuntimeError('TASK_NUMERIC_NOT_PASS')
   np.savez_compressed(run/'task_trajectory.npz',**allz)
   taskdir=run/'task';taskdir.mkdir()
   csv_rows=write_task_csv(dict(allz,storage_events=storage_events),taskdir/'trajectory.csv')
   summary['task']={'numerical':'PASS','collision':'PASS_BY_SAME_SCENE_SEGMENTS_AND_REPEATED_HOLDS','collision_states':sum(x['collision']['checked_states'] for x in summary['segments']),'duration_s':float(t[-1]),'motion_s':float(t[-1]-5),'dwell_s':5.,'samples':len(t),'events':events,'velocity_peak':np.max(abs(v),axis=0).tolist(),'acceleration_peak':np.max(abs(a),axis=0).tolist(),'jerk_peak':np.max(abs(j),axis=0).tolist(),'jerk_timeweighted_mean':(sum(np.trapz(np.abs(z['jerk']),z['t'],axis=0) for z in segments)/t[-1]).tolist(),'lateral_max':float(np.max(abs(lat))),'scope':'discrete segment collision; holds reuse identical checked endpoint states'}
   # Each accepted segment used the same explicitly matched PlanningScene.
   # Keep one real snapshot, not five copies of the same OctoMap.
   snapshots=[Path(x['directory'])/'result/PRP/scene_snapshot.bin' for x in summary['segments']]
   if not all(p.is_file() for p in snapshots):raise RuntimeError('TASK_SCENE_SNAPSHOT_MISSING')
   scene_bytes=snapshots[0].read_bytes()
   if any(p.read_bytes()!=scene_bytes for p in snapshots[1:]):raise RuntimeError('TASK_SCENE_CHANGED')
   shared=run/'scene_snapshot.bin';shared.write_bytes(scene_bytes)
   mapping={str(p):str(shared) for p in snapshots}
   def replace_paths(value):
    if isinstance(value,dict):return {k:replace_paths(v) for k,v in value.items()}
    if isinstance(value,list):return [replace_paths(v) for v in value]
    return mapping.get(value,value) if isinstance(value,str) else value
   summary=replace_paths(summary)
   for entry in summary['segments']:
    if entry.get('reused'):continue
    for jf in (Path(entry['directory'])/'result/result.json',Path(entry['directory'])/'result/PRP/result.json'):
     if jf.is_file():jf.write_text(json.dumps(replace_paths(json.loads(jf.read_text())),ensure_ascii=False,indent=2))
   for p,entry in zip(snapshots,summary['segments']):
    if not entry.get('reused'):p.unlink()
   # All unique states are now retained in task NPZ. Remove only per-segment
   # display and dense duplicates from this run, after the task is saved.
   for entry in summary['segments']:
    if entry.get('reused'):continue
    folder=Path(entry['directory'])
    for f in (folder/'result/PRP/P1/recommended/trajectory.npz',):
     if f.is_file():f.unlink()
   source=Path(summary['segments'][0]['directory'])/'result/P/parent_path.json'
   if not source.is_file():raise RuntimeError('TASK_PARENT_METADATA_MISSING')
   summary['task']['csv']={'path':str(taskdir/'trajectory.csv'),'bytes':(taskdir/'trajectory.csv').stat().st_size,'rows':csv_rows,'source_rows':len(t),'sampling':'original rows, 50 Hz target, event/extrema and position-error refinement; dense acceptance retained'}
   summary['status']='SIGNED_FIVE_GOAL_TASK_PASS';save()
   (taskdir/'result.json').write_text(json.dumps({'status':'TASK_PASS','source_parent':os.path.relpath(source,taskdir),'collision':{'status':'PASS','checked_states':summary['task']['collision_states']},'scope':summary['task']['scope']},ensure_ascii=False,indent=2))
 except Exception as ex:
  summary['failure_stage_or_reason']=repr(ex);(run/'error.log').write_text(traceback.format_exc());print(traceback.format_exc(),flush=True)
 save();logical,allocated=prp_compact._measure(run);summary['storage']={'logical_bytes':logical,'allocated_bytes':allocated,'budget_bytes':prp_compact.BUDGET,'status':'PASS' if max(logical,allocated)<=prp_compact.BUDGET else 'OUTPUT_BUDGET_EXCEEDED'};save();logical,allocated=prp_compact._measure(run);summary['storage'].update(logical_bytes=logical,allocated_bytes=allocated,status='PASS' if max(logical,allocated)<=prp_compact.BUDGET else 'OUTPUT_BUDGET_EXCEEDED');save();print('PRP_MULTI_RESULT',run,summary['status'],json.dumps(summary['storage']),flush=True)
 return 0 if summary['status']=='SIGNED_FIVE_GOAL_TASK_PASS' and summary['storage']['status']=='PASS' else 1
