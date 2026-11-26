"""Minimal v2-prp-v1 configuration adapter for saved P or signed R."""
import datetime,json,os,shutil,sys,traceback,subprocess,tempfile,math
from pathlib import Path
import yaml
from . import runtime_v2


def _path(value,base):
 if not isinstance(value,str) or not value:raise ValueError('CONFIG_PATH_REQUIRED')
 prefix='package://cr5_car_moveit/'
 if value.startswith(prefix):return (runtime_v2.PACKAGE/value[len(prefix):]).resolve()
 if value.startswith('package://'):raise ValueError('UNSUPPORTED_PACKAGE_URI: '+value)
 p=Path(os.path.expanduser(value));return (p if p.is_absolute() else base/p).resolve()

def experiment_name(mode, uses_ik, planner, now=None):
 if mode not in ('single','multi') or not isinstance(planner,str) or not planner or not all(c.isalnum() or c=='_' for c in planner):raise ValueError('INVALID_EXPERIMENT_IDENTITY')
 now=now or datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8)))
 return 'V2-%s-%s-%s-%s'%(now.strftime('%Y%m%d-%H%M%S'),mode.upper(),'IKPRP' if uses_ik else 'PRP',planner)


def run(config_path, control=None):
 control=control if control is not None else {}
 config_path=config_path.resolve(strict=True);base=config_path.parent;c=yaml.safe_load(config_path.read_text())
 if not isinstance(c,dict) or c.get('schema')!='v2-prp-v1':raise ValueError('EXPECTED_SCHEMA_V2_PRP_V1')
 scene=c['scene'];inp=c['input'];task=c['task'];planner=c['planner'];alg=c['algorithms'];limits=c['limits'];output=c['output'];runtime=c['runtime'];replay=c['replay']
 if task.get('mode')=='multi':
  from . import prp_multi
  return prp_multi.run(config_path,c)
 allowed={'scene':{'name','frame','octomap','planning_scene','snapshot'},'input':{'stage','path','until'},'task':{'mode','source','start_q','goal','targets','dwell_s'},'planner':{'id','group','allowed_time_s'},'algorithms':{'r','p0','p1'},'limits':{'profile','velocity','acceleration','jerk_soft_reference'},'output':{'root','npz','csv','plot'},'runtime':{'mode','master_uri','overlay_setup','python'},'replay':{'backend'}}
 if set(c)-({'schema'}|set(allowed)):raise ValueError('UNSUPPORTED_CONFIG_FIELDS: '+repr(sorted(set(c)-({'schema'}|set(allowed)))))
 for name,keys in allowed.items():
  if not isinstance(c[name],dict) or set(c[name])-keys:raise ValueError('UNSUPPORTED_CONFIG_FIELDS: '+name)
 stage=inp.get('stage')
 if stage not in ('new','P','R'):raise ValueError('UNSUPPORTED_INPUT_STAGE')
 if task.get('mode')!='single':raise ValueError('UNSUPPORTED_TASK_MODE')
 if planner.get('id')!='RRTstar' or planner.get('group')!='whole':raise ValueError('UNSUPPORTED_PLANNER')
 if stage=='new':
  if inp.get('path') is not None or task.get('source')!='configured_goal':raise ValueError('NEW_REQUIRES_CONFIGURED_GOAL_AND_NULL_PATH')
  goal=task.get('goal');start=task.get('start_q');duration=planner.get('allowed_time_s')
  def vec(x,n):return isinstance(x,list) and len(x)==n and all(isinstance(i,(int,float)) and not isinstance(i,bool) and math.isfinite(i) for i in x)
  pose_ok=isinstance(goal,dict) and set(goal)=={'type','link','frame','position_m','quaternion_xyzw'} and goal['type']=='link_pose' and goal['link']=='Link6' and goal['frame']=='world' and vec(goal['position_m'],3) and vec(goal['quaternion_xyzw'],4) and abs(sum(v*v for v in goal['quaternion_xyzw'])-1)<=2e-6
  state_ok=isinstance(goal,dict) and set(goal)=={'type','q'} and goal.get('type')=='joint_state' and vec(goal.get('q'),9)
  if not vec(start,9) or not (pose_ok or state_ok):raise ValueError('INVALID_CONFIGURED_GOAL')
  if not isinstance(duration,(int,float)) or isinstance(duration,bool) or not 0<duration<=120:raise ValueError('INVALID_PLANNING_TIME')
 else:
  if task.get('source')!='saved_parent' or task.get('start_q') is not None or task.get('goal') is not None:raise ValueError('SAVED_PARENT_CANNOT_OVERRIDE_GOAL')
  if planner.get('allowed_time_s') is not None:raise ValueError('SAVED_PARENT_CANNOT_REPLAN')
 if alg!={'r':'signed_v1','p0':'a_c_v1','p1':'formal_v1'}:raise ValueError('UNSUPPORTED_ALGORITHM_COMBINATION')
 if limits.get('profile')!='V2_NINE_DOF_VA_ONLY' or limits.get('velocity')!=[.5]*9 or limits.get('acceleration')!=[1.]*9:raise ValueError('UNSUPPORTED_LIMITS_PROFILE')
 if limits.get('jerk_soft_reference',5.0)!=5.0:raise ValueError('UNSUPPORTED_JERK_REFERENCE: formal P1 uses 5 as a soft search/selection reference')
 if scene.get('name')!='warehouse' or scene.get('frame')!='world' or scene.get('planning_scene') not in ('current','snapshot'):raise ValueError('UNSUPPORTED_SCENE')
 if scene.get('planning_scene')=='snapshot' and not scene.get('snapshot'):raise ValueError('SCENE_SNAPSHOT_REQUIRED')
 if scene.get('planning_scene')=='current' and scene.get('snapshot') is not None:raise ValueError('SNAPSHOT_REQUIRES_SNAPSHOT_MODE')
 if output.get('npz') not in (True,False) or output.get('csv')!='compact' or not isinstance(output.get('plot'),bool):raise ValueError('UNSUPPORTED_OUTPUT: boolean npz and compact CSV required')
 if runtime.get('mode')!='auto':raise ValueError('UNSUPPORTED_RUNTIME_MODE')
 if replay.get('backend')!='none':raise ValueError('UNSUPPORTED_REPLAY_BACKEND')
 until=inp.get('until')
 if until not in ('g0','g1'):raise ValueError('UNSUPPORTED_UNTIL')
 if output['plot'] and until!='g1':raise ValueError('PLOT_REQUIRES_G1')
 source=None;parent=None;relocated_map=False
 map_path=_path(scene['octomap'],base).resolve(strict=True)
 if stage!='new':
  source=_path(inp['path'],base)
  if source.is_file() and source.name in ('parent_path.json','geometry.json'):source=source.parent
  if not (source/'parent_path.json').is_file():raise ValueError('PARENT_PATH_MISSING')
  if inp['stage']=='R' and not (source/'geometry.json').is_file():raise ValueError('SIGNED_R_GEOMETRY_MISSING')
  parent=json.loads((source/'parent_path.json').read_text());profile=parent.get('experiment_limit_profile',parent['config'].get('constraint_profile',{}))
  disabled={'xy_speed','xy_acceleration','tangential_acceleration','wheel_velocity','wheel_acceleration','spin_jerk'};enabled=profile.get('enabled',{})
  if parent.get('planner_id')!='RRTstar' or profile.get('identity')!=limits['profile'] or profile.get('velocity')!=limits['velocity'] or profile.get('acceleration')!=limits['acceleration'] or set(enabled)!=disabled or any(enabled.values()):raise ValueError('SAVED_PARENT_PLANNER_OR_LIMITS_MISMATCH')
  parent_map=(runtime_v2.PACKAGE/parent['config']['static_octomap']).resolve()
  relocated_map=map_path!=parent_map
  if relocated_map:
   # A relocated saved P is usable only if its recorded map identity agrees;
   # comparing filenames or simply trusting the new configuration is unsafe.
   from hashlib import sha256
   recorded=parent.get('sources',{}).get('map',{}).get('sha256')
   if not recorded or sha256(map_path.read_bytes()).hexdigest()!=recorded:raise ValueError('SAVED_PARENT_SCENE_MAP_MISMATCH')
   parent['config']['static_octomap']=str(map_path)
 snapshot=_path(scene['snapshot'],base).resolve(strict=True) if scene.get('snapshot') else None
 # A caller may explicitly attest an external live scene without altering YAML.
 if stage=='new' and snapshot is None and os.environ.get('V2_PRP_SCENE_SNAPSHOT'):
  snapshot=Path(os.environ['V2_PRP_SCENE_SNAPSHOT']).expanduser().resolve(strict=True)
 chosen=os.environ.get('V2_CALLER_MASTER_URI') or runtime.get('master_uri') or runtime_v2.URI
 if not chosen.startswith(('http://','https://')):raise ValueError('INVALID_MASTER_URI')
 os.environ['ROS_MASTER_URI']=chosen;os.environ['V2_MASTER_URI']=chosen
 root=_path(output['root'],base);root.mkdir(parents=True,exist_ok=True);name=experiment_name('single',stage=='new' and task['goal']['type']=='link_pose',planner['id'] if stage=='new' else parent['planner_id']);run_dir=Path(control['run_dir_override']) if control.get('run_dir_override') else root/name;run_dir.mkdir(parents=True,exist_ok=False);control['run_dir']=str(run_dir)
 effective={'schema':c['schema'],'input_stage':inp['stage'],'input_path':str(source) if source else None,'scene_map':str(map_path),'parent_map_relocated':relocated_map,'scene_mode':scene['planning_scene'],'scene_snapshot':str(snapshot) if snapshot else None,'master_uri':chosen,'master_source':'environment' if os.environ.get('V2_CALLER_MASTER_URI') else 'config_or_default','python':sys.executable,'overlay_setup':os.environ.get('V2_EXTRA_SETUP') or None,'limits':limits,'algorithms':alg,'task':task,'output_root':str(root),'result_dir':str(run_dir),'compact_output':True,'source_config':str(config_path),'requested_config':c}
 (run_dir/'effective_config.json').write_text(json.dumps(effective,ensure_ascii=False,indent=2));print('PRP_CONFIG',json.dumps(effective,ensure_ascii=False),flush=True)
 work=run_dir/'.work';work.mkdir();result_dir=run_dir;code=1
 try:
  if stage=='new':
   from . import prp_from_r,prp_from_p,stage_v2
   cfg=yaml.safe_load((runtime_v2.PACKAGE/'v2/configs/v2_workspace_single_goal_9d_va_only.yaml').read_text())
   cfg['static_octomap']=str(map_path);cfg['planner_id']='RRTstar';cfg['group_name']='whole';cfg['allowed_planning_time_s']=float(planner['allowed_time_s']);cfg['master_uri']=chosen
   prp_from_r.ensure_master(chosen,cfg)
   # A newly planned P must record the same complete live scene used by IK,
   # MoveIt and final collision checking; do not assume a nonempty map matches.
   with tempfile.TemporaryDirectory(prefix='prp_scene_') as tmp:
    done=subprocess.run([sys.executable,'-B','-m','v2.scripts.ros_probe','--output',tmp,'--timeout',str(cfg.get('service_timeout_s',45))],cwd=str(runtime_v2.PACKAGE),capture_output=True,text=True)
    if done.returncode:raise RuntimeError('SCENE_PROBE_FAILED: '+done.stderr[-2000:])
    d=Path(tmp);scene_info=json.loads((d/'scene_probe.json').read_text())
    if (d/'robot_description.urdf').read_text()!=(runtime_v2.PACKAGE/'config/gazebo_combined_robot.urdf').read_text() or (d/'robot_description.srdf').read_text()!=(runtime_v2.PACKAGE/'config/combined_robot.srdf').read_text():raise RuntimeError('ROBOT_MODEL_MISMATCH')
    (work/'scene_snapshot.bin').write_bytes((d/'scene_snapshot.bin').read_bytes())
    runtime_state=runtime_v2.status() if chosen.rstrip('/')==runtime_v2.URI.rstrip('/') else {'status':'EXTERNAL'}
    if runtime_state.get('status')=='READY':
     if Path(runtime_state['map']).resolve()!=map_path:raise RuntimeError('WAREHOUSE_RUNTIME_MAP_MISMATCH')
    elif not snapshot or snapshot.read_bytes()!=(d/'scene_snapshot.bin').read_bytes():raise RuntimeError('EXTERNAL_SCENE_REQUIRES_MATCHING_EXPLICIT_SNAPSHOT')
   from .joint_ik import JointIK
   import numpy as np
   state_goal=goal['type']=='joint_state'
   if state_goal:
    from ..core.kdl_fk import LocalFK
    target_q=np.asarray(goal['q'],float).copy();target_q[2]=math.atan2(math.sin(target_q[2]),math.cos(target_q[2]));pos,rotation=LocalFK((runtime_v2.PACKAGE/'config/gazebo_combined_robot.urdf').read_text()).pose(target_q);pose={'frame_id':'world','link_name':'Link6','position':pos.tolist(),'orientation_xyzw':rotation.as_quat().tolist()}
   else:pose={'frame_id':goal['frame'],'link_name':goal['link'],'position':goal['position_m'],'orientation_xyzw':goal['quaternion_xyzw']}
   (work/'IK').mkdir();ik=JointIK(cfg,pose);(work/'IK/target.json').write_text(json.dumps(ik.target(),indent=2))
   if state_goal:
    checked=ik.check(target_q);start_checked=ik.check(task['start_q']);ik_result={'q':target_q.tolist(),'ik_feasible':checked['bounds']['status']=='PASS' and checked['scene_valid'],'source':'explicit_joint_state','target_validation':checked,'start_validation':start_checked}
    if start_checked['bounds']['status']!='PASS' or not start_checked['scene_valid']:raise RuntimeError('INVALID_START_STATE')
   else:ik_result=ik.solve(task['start_q'],1)
   (work/'IK/result.json').write_text(json.dumps(ik_result,indent=2))
   if not ik_result['ik_feasible']:raise RuntimeError('IK_POSTERIOR_NOT_FEASIBLE')
   plan_task={'id':'yaml_single','start_q':task['start_q'],'goal_q':ik_result['q'],'goal_tolerance':[.0005]*9}
   stage_v2.planning(cfg,plan_task,['global.yaml','new'],out=work/'P',context_fn=lambda _:scene_info)
   # A MoveIt joint-region success is not a Link6 task-goal success.
   parent_file=work/'P/parent_path.json';actual=json.loads(parent_file.read_text());p_fk=ik.check(actual['path']['q'][-1]);p_fk['joint_target_max_error']=float(max(np.max(np.abs(np.asarray(actual['path']['q'][-1])[[0,1,3,4,5,6,7,8]]-target_q[[0,1,3,4,5,6,7,8]])),abs(math.atan2(math.sin(actual['path']['q'][-1][2]-target_q[2]),math.cos(actual['path']['q'][-1][2]-target_q[2]))))) if state_goal else None;p_fk['ik_feasible']=p_fk['ik_feasible'] and (not state_goal or p_fk['joint_target_max_error']<=.002);(work/'IK/p_endpoint_fk.json').write_text(json.dumps(p_fk,indent=2));actual['planning']['task_goal_fk']=p_fk;actual['planning']['task_goal_status']='PASS' if p_fk['ik_feasible'] else 'FAIL';parent_file.write_text(json.dumps(actual,indent=2))
   if not p_fk['ik_feasible']:raise RuntimeError('P_ENDPOINT_TASK_FK_NOT_PASS')
   command=['--parent',str(work/'P'),'--out',str(work/'result'),'--until',until,'--scene-map',str(map_path),'--compact-output']
   command+=['--scene-snapshot',str(work/'scene_snapshot.bin')]
   # The matched parent-path comparison selected 36 as the formal default.
   rho_controls=control.get('rho_controls',36);command+=['--rho-controls',str(rho_controls)]
   if rho_controls==22 and control.get('rho_fallback')==36:command+=['--rho-fallback','36']
   if control.get('signed_recovery',True):command+=['--signed-recovery']
   code=prp_from_p.main(command)
   trajectory=work/'result/PRP/P1/recommended/trajectory.arrays'
   if until=='g1' and trajectory.is_dir():
    import numpy as np
    from ..core import array_store as store
    with store.load(trajectory) as z:
     fk=ik.check(z['q'][-1]);dq=np.asarray(z['q'][-1])-target_q if state_goal else None
     if state_goal:
      dq[2]=math.atan2(math.sin(dq[2]),math.cos(dq[2]));fk['joint_target_max_error']=float(np.max(np.abs(dq)));fk['ik_feasible']=fk['ik_feasible'] and fk['joint_target_max_error']<=.002
    (work/'IK/final_fk.json').write_text(json.dumps(fk,indent=2))
    if not fk['ik_feasible']:
     for f in (work/'result/result.json',work/'result/PRP/result.json'):
      if f.is_file():
       d=json.loads(f.read_text());d['status']='FINAL_TARGET_FK_FAIL';d['final_fk']=fk;f.write_text(json.dumps(d,indent=2))
     code=1;raise RuntimeError('FINAL_TARGET_FK_NOT_PASS')
  elif stage=='P':
   from . import prp_from_p
   if relocated_map:
    derived=work/'P_input';derived.mkdir();(derived/'parent_path.json').write_text(json.dumps(parent,indent=2));source=derived
   command=['--parent',str(source),'--out',str(work/'result'),'--until',until,'--scene-map',str(map_path),'--compact-output']
   if snapshot:command+=['--scene-snapshot',str(snapshot)]
   code=prp_from_p.main(command)
  else:
   staged=work/'R_input';staged.mkdir()
   if relocated_map:(staged/'parent_path.json').write_text(json.dumps(parent,indent=2))
   else:shutil.copyfile(source/'parent_path.json',staged/'parent_path.json')
   geo=json.loads((source/'geometry.json').read_text());key='signed_arrays' if 'signed_arrays' in geo else 'signed_npz';raw=Path(geo[key])
   local=source/raw.name
   resolved=(local if raw.is_absolute() and not raw.exists() and raw.name in ('candidate.npz','candidate.arrays') and local.exists() else raw if raw.is_absolute() else source/raw).resolve(strict=True)
   geo[key]=str(resolved);(staged/'geometry.json').write_text(json.dumps(geo,indent=2))
   from . import prp_from_r
   command=['--signed-r',str(staged),'--out',str(work/'result'),'--until',until,'--scene-map',str(map_path),'--compact-output']
   if snapshot:command+=['--scene-snapshot',str(snapshot)]
   code=prp_from_r.main(command)
 except Exception:
  (work/'error.log').write_text(traceback.format_exc());print(traceback.format_exc(),file=sys.stderr,flush=True);code=1
 from . import prp_compact
 storage=prp_compact.finalize(run_dir,work,stage,retain_dense=control.get('retain_dense',False))
 if storage['status']!='PASS':code=1
 csv=result_dir/'Parameterization/P1/recommended/trajectory.csv'
 if output['plot'] and csv.is_file():
  from .plot_trajectory import draw_one
  draw_one(csv,run_dir/'plots')
 print('PRP_GLOBAL_RESULT',str(result_dir),'exit',code,'storage',json.dumps(storage),flush=True);return code
