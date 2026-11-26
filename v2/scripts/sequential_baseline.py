#!/usr/bin/env python3
"""Two-stage MoveIt/OMPL baseline; never command physical robot."""
import argparse,csv,json,math,time,traceback,types
from pathlib import Path
import numpy as np,rospy,yaml
from moveit_msgs.srv import GetMotionPlan,GetMotionPlanRequest
from moveit_msgs.msg import Constraints,JointConstraint
from v2.scripts.joint_ik import JointIK
from v2.core.kdl_fk import LocalFK
from v2.adapters.planning_scene_checker import PlanningSceneChecker
N=['world_car/x','world_car/y','world_car/theta']+['joint%d'%i for i in range(1,7)]
P=Path(__file__).resolve().parents[2]
def main():
 a=argparse.ArgumentParser();a.add_argument('--parent',required=True);a.add_argument('--out',required=True);a.add_argument('--seconds',type=float,default=5.);args=a.parse_args();out=Path(args.out).resolve();out.mkdir(parents=True,exist_ok=False);res={'status':'FAIL','reference_parent':str(Path(args.parent).resolve()),'stages':{}}
 try:
  d=json.loads(Path(args.parent).read_text());q0=np.array(d['path']['q'][0]);goal=np.array(d['path']['q'][-1]);mid=np.r_[goal[:3],q0[3:]];res.update(start=q0.tolist(),goal=goal.tolist(),mid=mid.tolist());fk=LocalFK((P/'config/gazebo_combined_robot.urdf').read_text());pos,rot=fk.pose(mid);cfg=yaml.safe_load((P/'v2/configs/v2_workspace_single_goal_9d_va_only.yaml').read_text());ik=JointIK(cfg,{'frame_id':'world','link_name':'Link6','position':pos.tolist(),'orientation_xyzw':rot.as_quat().tolist()})
  for name,q in [('start',q0),('mid',mid),('goal',goal)]:
   z=ik.check(q);res[name+'_valid']={'bounds':z['bounds']['status'],'scene_valid':z['scene_valid'],'contacts':z['contacts']}
   if z['bounds']['status']!='PASS' or not z['scene_valid']:raise ValueError('INVALID_'+name.upper())
  call=rospy.ServiceProxy('/plan_kinematic_path',GetMotionPlan);trajectories=[];base_actual=None
  for group,start,target,names,vals in [('base',q0,mid,N[:3],mid[:3]),('arm',mid,goal,N[3:],goal[3:])]:
   if group=='arm':start=np.r_[base_actual,q0[3:]]
   r=GetMotionPlanRequest();m=r.motion_plan_request;m.group_name=group;m.planner_id='RRTstar';m.num_planning_attempts=1;m.allowed_planning_time=args.seconds;m.max_velocity_scaling_factor=1.;m.max_acceleration_scaling_factor=1.;m.start_state=ik.state(start);m.workspace_parameters.header.frame_id='world';m.workspace_parameters.min_corner.x=m.workspace_parameters.min_corner.y=m.workspace_parameters.min_corner.z=-20;m.workspace_parameters.max_corner.x=m.workspace_parameters.max_corner.y=m.workspace_parameters.max_corner.z=20;c=Constraints()
   for n,v in zip(names,vals):
    j=JointConstraint();j.joint_name=n;j.position=float(v);j.tolerance_above=j.tolerance_below=.0005;j.weight=1.;c.joint_constraints.append(j)
   m.goal_constraints=[c];t=time.monotonic();z=call(r).motion_plan_response;res['stages'][group]={'error_code':z.error_code.val,'planning_s':z.planning_time,'wall_s':time.monotonic()-t};print('STAGE',group,res['stages'][group],flush=True)
   if z.error_code.val!=1:raise RuntimeError('MOVEIT_'+group+'_FAILED_'+str(z.error_code.val))
   import io
   raw=io.BytesIO();z.trajectory.serialize(raw);(out/(group+'_trajectory.bin')).write_bytes(raw.getvalue())
   if group=='base':
    points=z.trajectory.multi_dof_joint_trajectory.points
    if not points:raise ValueError('NO_BASE_POINTS')
    arr=[]
    for pt in points:
     x=pt.transforms[0];q=np.r_[x.translation.x,x.translation.y,2*math.atan2(x.rotation.z,x.rotation.w),q0[3:]]
     if not pt.velocities or not pt.accelerations:raise ValueError('BASE_VELOCITY_OR_ACCELERATION_MISSING')
     v=pt.velocities[0];ac=pt.accelerations[0];arr.append((pt.time_from_start.to_sec(),q,np.r_[v.linear.x,v.linear.y,v.angular.z,np.zeros(6)],np.r_[ac.linear.x,ac.linear.y,ac.angular.z,np.zeros(6)]))
    base_actual=arr[-1][1][:3]
   else:
    jt=z.trajectory.joint_trajectory;indices=[list(jt.joint_names).index(n) for n in N[3:]];arr=[]
    for pt in jt.points:
     if len(pt.velocities)!=6 or len(pt.accelerations)!=6:raise ValueError('ARM_VELOCITY_OR_ACCELERATION_MISSING')
     arr.append((pt.time_from_start.to_sec(),np.r_[base_actual,np.array(pt.positions)[indices]],np.r_[np.zeros(3),np.array(pt.velocities)[indices]],np.r_[np.zeros(3),np.array(pt.accelerations)[indices]]))
   trajectories.append(arr)
  b,arm=trajectories;seamq=float(np.max(np.abs(b[-1][1]-arm[0][1])));seamv=float(max(np.max(abs(b[-1][2])),np.max(abs(arm[0][2]))));res['seam']={'q_error':seamq,'endpoint_speed_max':seamv}
  if seamq>1e-6 or seamv>1e-5:raise ValueError('SEAM_DISCONTINUITY')
  for name,arr in [('base',b),('arm',arm)]:
   ts=np.array([x[0] for x in arr]);qs=np.array([x[1] for x in arr]);vv=np.array([x[2] for x in arr]);aa=np.array([x[3] for x in arr]);
   if np.any(np.diff(ts)<=0):raise ValueError('TIMESTAMPS_NOT_STRICT')
   j=np.gradient(aa,ts,axis=0);dx=np.diff(qs[:,0]);dy=np.diff(qs[:,1]);th=np.unwrap(qs[:,2]);perp=-np.sin((th[:-1]+th[1:])/2)*dx+np.cos((th[:-1]+th[1:])/2)*dy;lat=-np.sin(th)*vv[:,0]+np.cos(th)*vv[:,1];res['stages'][name].update(points=len(arr),duration_s=float(ts[-1]),velocity_peak=np.max(abs(vv),axis=0).tolist(),acceleration_peak=np.max(abs(aa),axis=0).tolist(),lateral_velocity_max=float(np.max(abs(lat))),per_segment_lateral_m_max=float(max(abs(perp))) if len(perp) else 0.)
   res['stages'][name]['trajectory_csv']=str(out/(name+'.csv'))
   with (out/(name+'.csv')).open('x',newline='') as f:
    w=csv.writer(f);w.writerow(['time_s']+['q_'+x for x in N]+['v_'+x for x in N]+['a_'+x for x in N]+['jerk_'+x for x in N]+['direction'])
    for i in range(len(ts)):w.writerow([ts[i],*qs[i],*vv[i],*aa[i],*j[i],0 if name=='arm' else np.sign(math.cos(th[i])*vv[i,0]+math.sin(th[i])*vv[i,1])])
   if np.max(abs(vv))>.50001 or np.max(abs(aa))>1.0001:raise ValueError('MOTION_LIMIT_EXCEEDED_'+name)
   if name=='base' and (np.max(abs(lat))>1e-4 or max(abs(perp))>1e-3):raise ValueError('NONHOLONOMIC_NOT_VERIFIED')
   checker=PlanningSceneChecker();phase=types.SimpleNamespace(schema=types.SimpleNamespace(frame_id='world'));col=checker.check([phase],{'q':qs},include_geometry=False);col.pop('scene_snapshot_hex',None);col.pop('robot_description',None);col.pop('robot_description_semantic',None);res['stages'][name]['collision']=col
   if col['status']!='PASS':raise ValueError('COLLISION_NOT_PASS_'+name)
  allrows=b+[(x[0]+b[-1][0],x[1],x[2],x[3]) for x in arm[1:]];ts=np.array([x[0] for x in allrows]);qq=np.array([x[1] for x in allrows]);vv=np.array([x[2] for x in allrows]);aa=np.array([x[3] for x in allrows]);jj=np.gradient(aa,ts,axis=0)
  with (out/'trajectory.csv').open('x',newline='') as f:
   w=csv.writer(f);w.writerow(['time_s']+['q_'+x for x in N]+['v_'+x for x in N]+['a_'+x for x in N]+['jerk_'+x for x in N]+['direction'])
   for i in range(len(ts)):w.writerow([ts[i],*qq[i],*vv[i],*aa[i],*jj[i],np.sign(math.cos(qq[i,2])*vv[i,0]+math.sin(qq[i,2])*vv[i,1])])
  res.update(status='PASS',duration_s=float(ts[-1]),notes='MoveIt response discrete points and midpoints; jerk is acceleration finite difference on native time grid')
 except Exception as e:res['error']=repr(e);res['traceback']=traceback.format_exc();print('FAIL',repr(e),flush=True)
 (out/'result.json').write_text(json.dumps(res,indent=2));print('RESULT',json.dumps(res,ensure_ascii=False),flush=True);return 0 if res['status']=='PASS' else 1
if __name__=='__main__':raise SystemExit(main())
