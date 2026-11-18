"""Bounded attempts (default three); publish only after full acceptance."""
import copy,json,shutil,time,traceback
from pathlib import Path
from . import stage_v2,phg1_run

def run(config,task,command,max_attempts=3):
 if not isinstance(max_attempts,int) or not 1<=max_attempts<=10:raise ValueError('INVALID_ATTEMPT_BUDGET')
 root=stage_v2.new_dir(config,task,suffix='-PHG1'); counts={k:0 for k in ('IK','P','H','G0','G1','collision')};summary=dict(status='FAIL',max_attempts=max_attempts,total_attempts=0,success_attempt=None,failures=counts,final_trajectory=None);work=root/'.attempt_work';ik=None;ik_result=None;goal_task=copy.deepcopy(task)
 def save(): (root/'summary.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False))
 def fail(stage,num,reason,extra=None):
  counts[stage]+=1;folder=root/('attempt_%s_%02d'%(stage,counts[stage]));folder.mkdir(exist_ok=False);(folder/'failure.json').write_text(json.dumps(dict(global_attempt=num,stage=stage,reason=str(reason),**(extra or {})),indent=2,ensure_ascii=False));print('ATTEMPT_FAIL',num,stage,reason,flush=True)
 try:
  pose=goal_task.pop('pose_goal',None)
  if pose is not None:
   if 'start_q' not in task:raise ValueError('POSE_REQUIRES_START_Q')
   (root/'IK').mkdir();(root/'IK'/'target.json').write_text(json.dumps(pose,indent=2))
   from .joint_ik import JointIK
   try:
    expected=stage_v2.context(config);ik=JointIK(config,pose)
   except Exception as exc:
    summary['total_attempts']=1;fail('IK',1,repr(exc));(root/'IK'/'result.json').write_text(json.dumps(dict(ik_feasible=False,error=repr(exc)),indent=2));save();return root
   (root/'IK'/'target.json').write_text(json.dumps(ik.target(),indent=2))
  else:expected=None
  for num in range(1,max_attempts+1):
   summary['total_attempts']=num
   if pose is not None and ik_result is None:
    try:
     ik_result=ik.solve(task['start_q'],num)
     (root/'IK'/'result.json').write_text(json.dumps(ik_result,indent=2,ensure_ascii=False))
     if not ik_result['ik_feasible']:
      fail('IK',num,'IK_POSTERIOR_NOT_FEASIBLE',dict(ik_result=ik_result));ik_result=None;save();continue
     goal_task['goal_q']=ik_result['q']
    except Exception as exc:
     fail('IK',num,repr(exc));(root/'IK'/'result.json').write_text(json.dumps(dict(ik_feasible=False,error=repr(exc)),indent=2));ik_result=None;save();continue
   work.mkdir(); t=time.perf_counter()
   try:
    parent=stage_v2.planning(config,goal_task,command,out=work/'P')
    p=json.loads((parent/'parent_path.json').read_text())
    if expected is not None and p['scene']['scene_sha256']!=expected['scene_sha256']:raise RuntimeError('IK_PLANNING_SCENE_MISMATCH')
   except Exception as exc:
    detail=(work/'P'/'error.log').read_text() if (work/'P'/'error.log').exists() else ''
    fail('P',num,repr(exc),dict(elapsed_s=time.perf_counter()-t,diagnostic=detail[-12000:]));shutil.rmtree(work);save();continue
   t=time.perf_counter()
   try:
    phg1_run.main(['--input',str(parent),'--out',str(work),'--until','g1','--existing-root']);record=json.loads((work/'result.json').read_text())
   except Exception as exc:
    fail('G1',num,repr(exc),dict(elapsed_s=time.perf_counter()-t));shutil.rmtree(work);save();continue
   status=record.get('status');failed=None
   if status!='PASS':
    err=record.get('error','');failed='collision' if status=='COLLISION_NOT_PASS' else ('H' if 'H_FAILED' in err else 'G0' if 'G0_' in err else 'G1')
    details=''
    for path in (work/'H/error.log',work/'G0/G/error.log',work/'g1/error.log',work/'g1/recommended/collision.json',work/'error.log'):
     if path.is_file():details+='\n'+str(path.relative_to(work))+': '+path.read_text()[-6000:]
    fail(failed,num,err or status,dict(elapsed_s=time.perf_counter()-t,parent_path_hash=p['path'].get('parent_path_hash'),planning=p.get('planning'),stage_record=record,diagnostic=details[-12000:]));shutil.rmtree(work);save();continue
   if ik is not None:
    import csv
    with (work/'g1/recommended/trajectory.csv').open() as f:
     for row in csv.DictReader(f):last=row
    q=[float(last['q_'+n]) for n in stage_v2.NAMES];end=ik.check(q)
    if not end['ik_feasible']:
     fail('G1',num,'FINAL_END_POSE_OR_STATE_NOT_PASS',dict(final_check=end));shutil.rmtree(work);save();continue
    record['final_pose_check']=end
   for name in ('P','H','G0','g1','collision_command.log'):
    src=work/name
    if src.exists():src.rename(root/name)
   (root/'result.json').write_text(json.dumps(record,indent=2,ensure_ascii=False));shutil.rmtree(work)
   summary.update(status='PASS',success_attempt=num,final_trajectory='g1/recommended/trajectory.csv');save();print('TASK_RESULT',root,'PASS',flush=True);return root
 finally:
  if work.exists():shutil.rmtree(work)
  if summary['status']!='PASS' and not (root/'result.json').exists():
   (root/'result.json').write_text(json.dumps(dict(status='FAIL',reason='ATTEMPTS_EXHAUSTED_OR_BLOCKED',summary='summary.json'),indent=2))
  save()
 print('TASK_RESULT',root,'FAIL',flush=True);return root
