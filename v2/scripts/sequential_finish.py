#!/usr/bin/env python3
"""Finish a sequential baseline from a collision-verified fixed-arm Signed R base trajectory."""
import argparse,csv,io,json,math,time,traceback,types
from pathlib import Path
import numpy as np,rospy,yaml
from moveit_msgs.srv import GetMotionPlan,GetMotionPlanRequest
from moveit_msgs.msg import Constraints,JointConstraint
from v2.scripts.joint_ik import JointIK
from v2.core.kdl_fk import LocalFK
from v2.adapters.planning_scene_checker import PlanningSceneChecker
N=['world_car/x','world_car/y','world_car/theta']+['joint%d'%i for i in range(1,7)];P=Path(__file__).resolve().parents[2]
def main():
 ap=argparse.ArgumentParser();ap.add_argument('--parent',required=True);ap.add_argument('--base',required=True);ap.add_argument('--out',required=True);ap.add_argument('--smooth-arm',action='store_true');a=ap.parse_args();out=Path(a.out).resolve();out.mkdir(parents=True,exist_ok=False);res={'status':'FAIL','parent':str(Path(a.parent).resolve()),'base_csv':str(Path(a.base).resolve())}
 try:
  d=json.loads(Path(a.parent).read_text());start=np.array(d['path']['q'][0]);goal=np.array(d['path']['q'][-1]);res.update(start=start.tolist(),goal=goal.tolist());
  base_prp=Path(a.base).resolve().parents[2];base_report=json.loads((base_prp/'result.json').read_text());base_collision=json.loads((base_prp/'collision.json').read_text());base_snapshot=(base_prp/'scene_snapshot.bin').read_bytes()
  if base_report.get('status')!='FIXED_SIGNED_R_SINGLE_PASS' or base_collision.get('status')!='PASS':raise ValueError('BASE_NOT_VERIFIED')
  res['base_collision']={'status':base_collision['status'],'checked_states':base_collision.get('checked_states'),'evidence':str(base_prp/'collision.json')}
  with open(a.base,newline='') as f:br=list(csv.DictReader(f))
  tq=lambda r,prefix:np.array([float(r[prefix+n]) for n in N]);bq=np.array([tq(r,'q_') for r in br]);bt=np.array([float(r['time_s']) for r in br]);bv=np.array([tq(r,'v_') for r in br]);ba=np.array([tq(r,'a_') for r in br]);bj=np.array([tq(r,'jerk_') for r in br]);res['base_arm_max_drift_rad']=float(np.max(np.abs(bq[:,3:]-start[3:])));res['base_duration_s']=float(bt[-1]);
  res['base_start_error']=float(np.max(abs(bq[0]-start)))
  if res['base_start_error']>1e-6:raise ValueError('BASE_START_MISMATCH')
  if res['base_arm_max_drift_rad']>1e-8:raise ValueError('BASE_ARM_NOT_FIXED')
  if np.max(abs(bq[-1,:3]-goal[:3]))>.002:raise ValueError('BASE_ENDPOINT_NOT_GOAL')
  mid=bq[-1].copy();pos,rot=LocalFK((P/'config/gazebo_combined_robot.urdf').read_text()).pose(mid);cfg=yaml.safe_load((P/'v2/configs/v2_workspace_single_goal_9d_va_only.yaml').read_text());ik=JointIK(cfg,{'frame_id':'world','link_name':'Link6','position':pos.tolist(),'orientation_xyzw':rot.as_quat().tolist()});res['mid']=mid.tolist();call=rospy.ServiceProxy('/plan_kinematic_path',GetMotionPlan);r=GetMotionPlanRequest();m=r.motion_plan_request;m.group_name='arm';m.planner_id='RRTstar';m.num_planning_attempts=1;m.allowed_planning_time=5.;m.max_velocity_scaling_factor=1.;m.max_acceleration_scaling_factor=1.;m.start_state=ik.state(mid);m.workspace_parameters.header.frame_id='world';m.workspace_parameters.min_corner.x=m.workspace_parameters.min_corner.y=m.workspace_parameters.min_corner.z=-20;m.workspace_parameters.max_corner.x=m.workspace_parameters.max_corner.y=m.workspace_parameters.max_corner.z=20;c=Constraints()
  for n,v in zip(N[3:],goal[3:]):
   j=JointConstraint();j.joint_name=n;j.position=float(v);j.tolerance_above=j.tolerance_below=.0005;j.weight=1.;c.joint_constraints.append(j)
  m.goal_constraints=[c];t=time.monotonic();z=call(r).motion_plan_response;res['arm_planning_s']=z.planning_time;res['arm_wall_s']=time.monotonic()-t;res['arm_code']=z.error_code.val
  if z.error_code.val!=1:raise RuntimeError('ARM_MOVEIT_ERROR_'+str(z.error_code.val))
  raw=io.BytesIO();z.trajectory.serialize(raw);(out/'arm_trajectory.bin').write_bytes(raw.getvalue());jt=z.trajectory.joint_trajectory;inds=[list(jt.joint_names).index(n) for n in N[3:]];ts=np.array([p.time_from_start.to_sec() for p in jt.points]);q=np.array([np.r_[mid[:3],np.array(p.positions)[inds]] for p in jt.points]);v=np.array([np.r_[np.zeros(3),np.array(p.velocities)[inds]] for p in jt.points]);acc=np.array([np.r_[np.zeros(3),np.array(p.accelerations)[inds]] for p in jt.points]);res['arm_points']=len(q);res['arm_duration_s']=float(ts[-1]);res['seam_q_error']=float(np.max(abs(q[0]-bq[-1])));res['seam_v_max']=float(max(np.max(abs(v[0])),np.max(abs(bv[-1]))));res['arm_velocity_peak']=np.max(abs(v),axis=0).tolist();res['arm_acceleration_peak']=np.max(abs(acc),axis=0).tolist();res['final_joint_error']=float(np.max(abs(q[-1,3:]-goal[3:])))
  if a.smooth_arm:
   # Smooth monotone time warp on the native TOTG Hermite path; no dwell.
   from scipy.interpolate import CubicHermiteSpline
   if np.any(np.diff(ts)<=0):raise ValueError('NATIVE_ARM_TIMESTAMPS_INVALID')
   native_T=float(ts[-1]-ts[0]);interp=CubicHermiteSpline(ts,q,v,axis=0);duration=native_T
   for iteration in range(4):
    count=max(2,int(math.ceil(duration/.02))+1);new_t=np.linspace(0,duration,count);u=new_t/duration;h=10*u**3-15*u**4+6*u**5;hd=30*u**2*(1-u)**2;hdd=60*u*(1-u)*(1-2*u);source_t=ts[0]+native_T*h;q=interp(source_t);v=interp(source_t,1)*(native_T/duration*hd)[:,None];acc=interp(source_t,2)*(native_T/duration*hd)[:,None]**2+interp(source_t,1)*(native_T/duration**2*hdd)[:,None];factor=max(1.,float(np.max(abs(v))/.5),float(np.sqrt(np.max(abs(acc))/1.)))
    if factor<=1.000001:break
    duration*=factor*1.005
   ts=new_t;res.update(arm_time_law='TOTG Hermite path with monotone quintic time warp; no dwell',arm_duration_s=float(ts[-1]),arm_velocity_peak=np.max(abs(v),axis=0).tolist(),arm_acceleration_peak=np.max(abs(acc),axis=0).tolist(),seam_q_error=float(np.max(abs(q[0]-bq[-1]))),seam_v_max=float(max(np.max(abs(v[0])),np.max(abs(bv[-1])))),final_joint_error=float(np.max(abs(q[-1,3:]-goal[3:]))))
  if np.any(np.diff(ts)<=0) or res['seam_q_error']>1e-6 or res['seam_v_max']>1e-5 or np.max(abs(v))>.5001 or np.max(abs(acc))>1.0001 or res['final_joint_error']>.002:raise ValueError('ARM_TIME_LIMIT_OR_SEAM_FAILURE')
  checker=PlanningSceneChecker();ph=types.SimpleNamespace(schema=types.SimpleNamespace(frame_id='world'));col=checker.check([ph],{'q':q},include_geometry=False);scene_match=bool(col.get('scene_snapshot_hex')) and bytes.fromhex(col['scene_snapshot_hex'])==base_snapshot;model_match=col.get('robot_description')==(P/'config/gazebo_combined_robot.urdf').read_text() and col.get('robot_description_semantic')==(P/'config/combined_robot.srdf').read_text();col.pop('scene_snapshot_hex',None);col.pop('robot_description',None);col.pop('robot_description_semantic',None);col['scene_match_base']=scene_match;col['model_match']=model_match;res['arm_collision']=col
  if not scene_match or not model_match:raise ValueError('SCENE_OR_MODEL_MISMATCH')
  if col['status']!='PASS':raise ValueError('ARM_COLLISION_NOT_PASS')
  j=np.gradient(acc,ts,axis=0);rows=[]
  with (out/'arm.csv').open('x',newline='') as f:
   w=csv.writer(f);w.writerow(['time_s']+[p+n for p in ('q_','v_','a_','jerk_') for n in N]+['direction']);
   for i in range(len(ts)):w.writerow([ts[i],*q[i],*v[i],*acc[i],*j[i],0])
  with (out/'trajectory.csv').open('x',newline='') as f:
   fields=['time_s']+[p+n for p in ('q_','v_','a_','jerk_') for n in N]+['direction'];w=csv.DictWriter(f,fieldnames=fields);w.writeheader();
   for row in br:w.writerow({x:row[x] for x in fields})
   for i in range(1,len(ts)):
    vals=[bt[-1]+ts[i],*q[i],*v[i],*acc[i],*j[i],0];w.writerow(dict(zip(fields,vals)))
  res.update(status='PASS',total_s=float(bt[-1]+ts[-1]),planning_definition='MoveIt base RRTstar, Signed R + P1 base reconstruction, MoveIt arm RRTstar + TOTG with optional monotone smoothing; full robot discrete checks',jerk='base original analytic; arm finite difference of accelerations on emitted time grid')
 except Exception as e:res['error']=repr(e);res['traceback']=traceback.format_exc();print('FAIL',repr(e),flush=True)
 (out/'result.json').write_text(json.dumps(res,indent=2));print('RESULT',json.dumps(res,ensure_ascii=False),flush=True);return 0 if res['status']=='PASS' else 1
if __name__=='__main__':raise SystemExit(main())
