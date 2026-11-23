"""Isaac 4.2 session-only kinematic playback, launched by the V2 display menu."""
import argparse,csv,json,math,sys,time,traceback
from pathlib import Path

parser=argparse.ArgumentParser()
parser.add_argument('--trajectory',required=True)
parser.add_argument('--stage',required=True)
parser.add_argument('--output',required=True)
parser.add_argument('--gui',action='store_true')
parser.add_argument('--max-duration',type=float,default=None)
parser.add_argument('--hold-open',action='store_true')
parser.add_argument('--wait-for-start',action='store_true')
args=parser.parse_args()
if args.max_duration is not None and (not math.isfinite(args.max_duration) or args.max_duration<=0):parser.error('--max-duration must be positive')
sys.argv=[sys.argv[0],'--disable','omni.isaac.ros2_bridge','--disable','omni.isaac.ros_bridge']
from isaacsim import SimulationApp
app=SimulationApp({'headless':not args.gui,'renderer':'RayTracedLighting','width':960,'height':540,'multi_gpu':False})
out=Path(args.output)
metrics={'status':'STARTED','gui_requested':args.gui,'gui_visual_status':'PENDING_USER' if args.gui else 'NOT_STARTED','scope':'kinematic state teleport; not physical tracking or collision validation'}
try:
 import numpy as np
 import omni.usd
 import omni.physx
 from pxr import UsdGeom,UsdPhysics,PhysxSchema
 from omni.isaac.core import World
 from omni.isaac.core.articulations import Articulation
 from omni.isaac.core.prims import RigidPrim
 from v2.scripts.isaac_trajectory import load_trajectory
 data=load_trajectory(args.trajectory);times=np.asarray(data.time_s);q=np.asarray(data.q);q[:,2]=np.unwrap(q[:,2]);full_duration=float(times[-1]-times[0]);duration=min(full_duration,args.max_duration) if args.max_duration is not None else full_duration;names=['joint%d'%i for i in range(1,7)]
 if not omni.usd.get_context().open_stage(args.stage):raise RuntimeError('STAGE_OPEN_FAILED')
 stage=omni.usd.get_context().get_stage();roots=[p for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
 if len(roots)!=1:raise RuntimeError('EXPECTED_ONE_ARTICULATION_ROOT: '+str(len(roots)))
 root=roots[0];robot_container=root.GetParent();stage.SetEditTarget(stage.GetSessionLayer());disabled=[]
 for p in stage.Traverse():
  if p.GetPath().HasPrefix(robot_container.GetPath()) and p.HasAPI(UsdPhysics.RigidBodyAPI):
   if not PhysxSchema.PhysxRigidBodyAPI.Apply(p).CreateDisableGravityAttr(True).Set(True):raise RuntimeError('DISABLE_GRAVITY_FAILED: '+str(p.GetPath()))
   disabled.append(str(p.GetPath()))
 if str(root.GetPath()) not in disabled:raise RuntimeError('ROOT_GRAVITY_NOT_DISABLED')
 world=World(stage_units_in_meters=1.0,physics_dt=1/60,rendering_dt=1/60);world.reset();art=Articulation(prim_path=str(root.GetPath()));art.initialize()
 if not art.handles_initialized:raise RuntimeError('ARTICULATION_INITIALIZATION_FAILED')
 dof=list(art.dof_names)
 if any(dof.count(n)!=1 for n in names):raise RuntimeError('JOINT_NAMES_NOT_UNIQUE')
 indices=np.array([dof.index(n) for n in names]);art.set_linear_velocity(np.zeros(3));art.set_angular_velocity(np.zeros(3));art.set_joint_velocities(np.zeros(len(dof)))
 wheels={}
 for name in ('wheel_left_link','wheel_right_link'):
  prim=robot_container.GetPath().AppendChild(name);wheel=RigidPrim(prim_path=str(prim));wheel.initialize();wheels[name]=wheel
 print('READY',json.dumps({'stage':args.stage,'source':data.source,'root':str(root.GetPath()),'indices':indices.tolist(),'disabled_robot_bodies':len(disabled),'full_duration_s':full_duration,'requested_duration_s':duration,'gui':args.gui}),flush=True)
 fields=['sim_time_s','trajectory_time_s','trajectory_elapsed_s','time_sync_error_s','z_after_step','z_after_set','z_after_render','usd_z_after_render','vz_after_step','vz_after_render','left_wheel_z_after_render','right_wheel_z_after_render','left_wheel_usd_z_after_render','right_wheel_usd_z_after_render','render_time_delta_s','x_ref','y_ref','yaw_ref','x_actual','y_actual','yaw_actual','usd_x','usd_y','usd_yaw']+['joint%d_ref'%i for i in range(1,7)]+['joint%d_actual'%i for i in range(1,7)]
 # Set and synchronize the initial state before the first recorded/rendered frame.
 initial=q[0];art.set_world_pose(position=np.array([initial[0],initial[1],0.034224]),orientation=np.array([math.cos(initial[2]/2),0,0,math.sin(initial[2]/2)]));art.set_joint_positions(initial[3:],joint_indices=indices)
 world.physics_sim_view.update_articulations_kinematic();omni.physx.get_physx_interface().update_transformations(False,True,False,False)
 if args.wait_for_start:
  import threading
  world.render();world.pause();answer=[]
  def wait_terminal():
   try:
    print('Isaac 场景已就绪，按 Enter 开始回放，输入 q 取消。',flush=True)
    answer.append(input().strip().lower())
   except EOFError:answer.append('q')
  threading.Thread(target=wait_terminal,daemon=True).start()
  while not answer and app.is_running():app.update();time.sleep(.03)
  if not answer or answer[0] in ('q','quit'):raise RuntimeError('PLAYBACK_CANCELLED')
  world.play()
 filename='short_playback.csv' if args.max_duration is not None else 'playback.csv';start_sim=float(world.current_time);wall_start=time.monotonic();last_sim=None;rows=0;max_xy=0.;max_x=0.;max_y=0.;max_yaw=0.;max_joint=np.zeros(6);max_time=0.;max_vz=0.;max_render_dt=0.;max_usd_xy=0.;max_usd_yaw=0.;zs=[];left=[];right=[];first_error=None;last_error=None
 def yaw_of(quat):
  w,x,y,z=quat;return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
 def angle_diff(a,b):return math.atan2(math.sin(a-b),math.cos(a-b))
 with (out/filename).open('x',newline='') as f:
  writer=csv.writer(f);writer.writerow(fields)
  while True:
   if rows:world.step(render=False)
   now=float(world.current_time);elapsed=now-start_sim
   if last_sim is not None and now<=last_sim:raise RuntimeError('SIM_TIME_NOT_STRICTLY_INCREASING')
   step_pos,_=art.get_world_pose();step_v=float(art.get_linear_velocity()[2]);trajectory_elapsed=min(max(elapsed,0.),duration);query=times[0]+trajectory_elapsed;ref=np.array([np.interp(query,times,q[:,j]) for j in range(9)])
   art.set_world_pose(position=np.array([ref[0],ref[1],0.034224]),orientation=np.array([math.cos(ref[2]/2),0,0,math.sin(ref[2]/2)]));art.set_joint_positions(ref[3:],joint_indices=indices);set_z=float(art.get_world_pose()[0][2]);world.physics_sim_view.update_articulations_kinematic();omni.physx.get_physx_interface().update_transformations(False,True,False,False);before_render=float(world.current_time);world.render();after_render=float(world.current_time);render_dt=after_render-before_render
   pos,quat=art.get_world_pose();actual_j=np.asarray(art.get_joint_positions(joint_indices=indices));yaw=yaw_of(quat);delta=np.array([float(pos[0]-ref[0]),float(pos[1]-ref[1]),angle_diff(yaw,ref[2])]+(actual_j-ref[3:]).tolist());sync=elapsed-trajectory_elapsed;cache=UsdGeom.XformCache();matrix=cache.GetLocalToWorldTransform(root);usd_xyz=matrix.ExtractTranslation();usd_q=matrix.ExtractRotationQuat();im=usd_q.GetImaginary();usd_yaw=yaw_of([usd_q.GetReal(),im[0],im[1],im[2]]);wheel_z=[float(wheels[n].get_world_pose()[0][2]) for n in ('wheel_left_link','wheel_right_link')];wheel_usd=[float(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(robot_container.GetPath().AppendChild(n))).ExtractTranslation()[2]) for n in ('wheel_left_link','wheel_right_link')];render_v=float(art.get_linear_velocity()[2])
   writer.writerow([now,query,trajectory_elapsed,sync,float(step_pos[2]),set_z,float(pos[2]),float(usd_xyz[2]),step_v,render_v,*wheel_z,*wheel_usd,render_dt,ref[0],ref[1],ref[2],pos[0],pos[1],yaw,usd_xyz[0],usd_xyz[1],usd_yaw]+ref[3:].tolist()+actual_j.tolist());f.flush();rows+=1
   max_x=max(max_x,abs(delta[0]));max_y=max(max_y,abs(delta[1]));max_xy=max(max_xy,float(np.linalg.norm(delta[:2])));max_yaw=max(max_yaw,abs(delta[2]));max_joint=np.maximum(max_joint,np.abs(delta[3:]));max_time=max(max_time,abs(sync));max_vz=max(max_vz,abs(step_v),abs(render_v));max_render_dt=max(max_render_dt,abs(render_dt));max_usd_xy=max(max_usd_xy,math.hypot(usd_xyz[0]-ref[0],usd_xyz[1]-ref[1]));max_usd_yaw=max(max_usd_yaw,abs(angle_diff(usd_yaw,ref[2])));zs.extend([float(step_pos[2]),set_z,float(pos[2]),float(usd_xyz[2])]);left.append(wheel_z[0]);right.append(wheel_z[1]);first_error=delta.tolist() if first_error is None else first_error;last_error=delta.tolist();last_sim=now
   if args.gui:
    delay=wall_start+elapsed-time.monotonic()
    if delay>0:time.sleep(delay)
   if trajectory_elapsed>=duration:break
   if rows%60==0:print('PLAYBACK_PROGRESS',rows,'simulation_s',elapsed,flush=True)
   if not app.is_running():raise RuntimeError('ISAAC_APPLICATION_CLOSED')
 wall=time.monotonic()-wall_start;drift=max(abs(x-.034224) for x in zs);checks=rows>2 and abs(float(world.current_time)-start_sim-duration)<=1/60+1e-5 and drift<1e-4 and max_vz<1e-5 and max_xy<1e-4 and max_yaw<1e-4 and max(max_joint)<1e-4 and max_render_dt<1e-9 and max_usd_xy<1e-4 and max_usd_yaw<1e-4
 metrics.update(status='PASS' if checks else 'FAIL',source=data.source,source_samples=len(times),stage=args.stage,root=str(root.GetPath()),disabled_robot_bodies=disabled,session_layer_only=True,reference_start_s=float(times[0]),full_trajectory_duration_s=full_duration,requested_duration_s=duration,sim_coverage_s=float(world.current_time)-start_sim,trajectory_coverage_s=trajectory_elapsed,updates=rows,physics_steps=rows-1,playback_wall_s=wall,root_z_range_m=[min(zs),max(zs)],max_z_drift_m=drift,max_abs_vertical_velocity_m_s=max_vz,left_wheel_center_z_range_m=[min(left),max(left)],right_wheel_center_z_range_m=[min(right),max(right)],max_abs_x_m=max_x,max_abs_y_m=max_y,max_xy_m=max_xy,max_abs_yaw_rad=max_yaw,max_abs_joint_rad=max_joint.tolist(),max_usd_xy_error_m=max_usd_xy,max_usd_yaw_error_rad=max_usd_yaw,max_render_time_advance_s=max_render_dt,max_time_sync_error_s=max_time,first_error=first_error,last_error=last_error,root_z_m=.034224,world_transform='identity XY/yaw; provisional visual height',joint_indices=indices.tolist())
 print('PLAYBACK_RESULT',json.dumps(metrics),flush=True)
except Exception as e:
 metrics.update(status='FAIL',error=repr(e));traceback.print_exc()
finally:
 (out/'metrics.json').write_text(json.dumps(metrics,indent=2)+'\n');(out/'playback_metrics.json').write_text(json.dumps(metrics,indent=2)+'\n');(out/'REPORT.md').write_text('# Isaac 运动学回放\n\n状态：%s；详见 metrics.json 和运行日志。GUI 视觉验收由用户完成。\n' % metrics['status'])
 if args.gui and args.hold_open:
  try:
   world.pause();print('GUI_HOLD_OPEN: 窗口已暂停保持末态；完成观察后关闭 Isaac 窗口。',flush=True)
   while app.is_running():app.update();time.sleep(.03)
  except Exception:traceback.print_exc()
 app.close()
sys.exit(0 if metrics['status']=='PASS' else 1)
