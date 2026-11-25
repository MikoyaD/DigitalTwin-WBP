"""Short P1 physical tracking experiment; no state teleport after initialization."""
import argparse,csv,json,math,sys,time,traceback
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--mode',choices=['feedforward','feedback'],required=True);p.add_argument('--start',type=float,default=0);p.add_argument('--duration',type=float,default=4);p.add_argument('--gui',action='store_true');p.add_argument('--hold-open',action='store_true');p.add_argument('--physics-hz',type=int,choices=[120,240,480],default=120);p.add_argument('--stage',default='/home/robot/digitaltwin/zzs_ws/src/cr5_car_moveit/v2/assets/isaac_import_20261124/warehouse_color_overlay.usda');p.add_argument('--trajectory',default='/home/robot/digitaltwin/zzs_ws/src/cr5_car_moveit/v2/outputs/V2-20261124-014348-SINGLE-IKPRP-RRTstar/Parameterization/P1/recommended/trajectory.csv');args=p.parse_args();sys.argv=[sys.argv[0],'--disable','omni.isaac.ros2_bridge','--disable','omni.isaac.ros_bridge']
from isaacsim import SimulationApp
app=SimulationApp({'headless':not args.gui,'renderer':'RayTracedLighting','width':960 if args.gui else 320,'height':540 if args.gui else 240,'multi_gpu':False})
out=Path(args.output);out.mkdir(parents=True,exist_ok=True);result={'status':'STARTED','mode':args.mode,'gui':args.gui}
try:
 import numpy as np,omni.usd
 from pxr import UsdGeom,UsdPhysics,UsdShade,PhysxSchema,Gf
 from omni.isaac.core import World
 from omni.isaac.core.articulations import Articulation
 from omni.isaac.core.prims import RigidPrim
 from omni.isaac.core.utils.types import ArticulationAction
 from omni.isaac.wheeled_robots.controllers import DifferentialController
 from v2.scripts.isaac_trajectory import load_trajectory
 d=load_trajectory(args.trajectory);times=np.asarray(d.time_s);qs=np.asarray(d.q).copy();qs[:,2]=np.unwrap(qs[:,2]);vs=np.asarray(d.velocity);dt=1/args.physics_hz
 if args.start<times[0] or args.start+args.duration>times[-1] or args.duration<=0:raise ValueError('INVALID_WINDOW')
 def interp(arr,t):return np.array([np.interp(t,times,arr[:,j]) for j in range(9)])
 q0=interp(qs,args.start);reference_v0=interp(vs,args.start)
 if np.max(np.abs(reference_v0))>1e-5:raise ValueError('WINDOW_START_REQUIRES_REST_REFERENCE')
 if not omni.usd.get_context().open_stage(args.stage):raise RuntimeError('STAGE_OPEN_FAILED')
 stage=omni.usd.get_context().get_stage();stage.SetEditTarget(stage.GetSessionLayer())
 ground=UsdGeom.Cube.Define(stage,'/Warehouse/ExperimentTrackingGround');ground.CreateSizeAttr(1);ground.AddTranslateOp().Set(Gf.Vec3d(0.,0.,.034224-.01));ground.AddScaleOp().Set(Gf.Vec3f(40.,40.,.02));UsdPhysics.CollisionAPI.Apply(ground.GetPrim());ground.MakeInvisible()
 mat=UsdShade.Material.Define(stage,'/Warehouse/ExperimentTrackingSupportFriction');m=UsdPhysics.MaterialAPI.Apply(mat.GetPrim());m.CreateStaticFrictionAttr(0.);m.CreateDynamicFrictionAttr(0.);m.CreateRestitutionAttr(0.);PhysxSchema.PhysxMaterialAPI.Apply(mat.GetPrim()).CreateFrictionCombineModeAttr('min')
 for name in ('ball_left_link','ball_right_link'):
  prim=stage.GetPrimAtPath('/Warehouse/robot_with_default_prim/'+name+'/collisions');UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat,materialPurpose='physics')
 for name in ('wheel_left_link','wheel_right_link'):
  link='/Warehouse/robot_with_default_prim/'+name;UsdPhysics.CollisionAPI(stage.GetPrimAtPath(link+'/collisions')).GetCollisionEnabledAttr().Set(False)
  c=UsdGeom.Cylinder.Define(stage,link+'/TrackingCylinder');c.CreateAxisAttr('Y');c.CreateRadiusAttr(.06);c.CreateHeightAttr(.04);UsdPhysics.CollisionAPI.Apply(c.GetPrim())
 world=World(stage_units_in_meters=1.,physics_dt=dt,rendering_dt=1/60);world.get_physics_context().set_solver_type('PGS');world.reset();art=Articulation(prim_path='/Warehouse/robot_with_default_prim/car_link');art.initialize()
 if not art.handles_initialized:raise RuntimeError('ARTICULATION_INIT_FAILED')
 wheels_rigid=[RigidPrim(prim_path='/Warehouse/robot_with_default_prim/'+name) for name in ('wheel_left_link','wheel_right_link')]
 for rigid in wheels_rigid:rigid.initialize()
 tip=RigidPrim(prim_path='/Warehouse/robot_with_default_prim/Link6');tip.initialize()
 names=list(art.dof_names);wi=np.array([names.index('wheel_left_joint'),names.index('wheel_right_joint')]);ai=np.array([names.index('joint%d'%i) for i in range(1,7)]);ctrl=art.get_articulation_controller();kp,kd=map(np.array,ctrl.get_gains());kp[wi]=0;kd[wi]=5;kp[ai]=500;kp[ai[1]]=2000;kd[ai]=50;ctrl.set_gains(kps=kp,kds=kd,save_to_usd=False);ctrl.set_max_efforts(np.array([10.,10.]),joint_indices=wi);diff=DifferentialController(name='p1_short',wheel_radius=.06,wheel_base=.395)
 # Initialization only. All tracking motion below is physically driven.
 art.set_world_pose(position=np.array([q0[0],q0[1],.034224]),orientation=np.array([math.cos(q0[2]/2),0,0,math.sin(q0[2]/2)]));art.set_joint_positions(q0[3:],joint_indices=ai);art.set_linear_velocity(np.zeros(3));art.set_angular_velocity(np.zeros(3));art.set_joint_velocities(np.zeros(len(names)))
 prev=float(world.current_time)
 def step(n):
  global prev
  world.step(render=False);now=float(world.current_time)
  if abs(now-prev-dt)>1e-5:raise RuntimeError('BAD_PHYSICS_TIMESTEP')
  prev=now
  if args.gui and n%max(1,args.physics_hz//60)==0:
   world.render()
   if abs(float(world.current_time)-now)>1e-9:raise RuntimeError('RENDER_ADVANCED_PHYSICS')
  return now
 def yaw(q):
  w,x,y,z=q;return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
 def wrap(a):return math.atan2(math.sin(a),math.cos(a))
 for n in range(args.physics_hz):
  ctrl.apply_action(ArticulationAction(joint_velocities=np.zeros(2),joint_indices=wi));ctrl.apply_action(ArticulationAction(joint_positions=q0[3:],joint_indices=ai));step(n)
 start_sim=float(world.current_time);pos0,rot0=art.get_world_pose();initial_read={'xyz':pos0.tolist(),'yaw':yaw(rot0),'joint':art.get_joint_positions(joint_indices=ai).tolist()};lastcmd=np.zeros(2);last_world=np.zeros(2);applied_wheels=np.zeros(2);rows=[];wall=time.monotonic();steps=math.ceil(args.duration/dt)
 fields=['sim_time_s','trajectory_time_s','x_actual','y_actual','yaw_actual','vx_world_actual','vy_world_actual','yaw_velocity_actual','v_cmd','omega_cmd','wheel_left_cmd','wheel_right_cmd','wheel_left_actual','wheel_right_actual']+['joint%d_actual'%j for j in range(1,7)]+['joint%d_vactual'%j for j in range(1,7)]+['link6_x','link6_y','link6_z','link6_qw','link6_qx','link6_qy','link6_qz']
 with (out/'states.csv').open('x',newline='') as f:
  writer=csv.writer(f);writer.writerow(fields)
  for n in range(steps+1):
   now=float(world.current_time);elapsed=now-start_sim;t=min(args.start+elapsed,args.start+args.duration);q=interp(qs,t);v=interp(vs,t);pos,quat=art.get_world_pose();y=yaw(quat);lin=np.asarray(art.get_linear_velocity());ang=np.asarray(art.get_angular_velocity());jw=np.asarray(art.get_joint_velocities(joint_indices=wi));ja=np.asarray(art.get_joint_positions(joint_indices=ai));jv=np.asarray(art.get_joint_velocities(joint_indices=ai));dx=q[0]-pos[0];dy=q[1]-pos[1];ex=math.cos(y)*dx+math.sin(y)*dy;ey=-math.sin(y)*dx+math.cos(y)*dy;eth=wrap(q[2]-y);vref=math.cos(q[2])*v[0]+math.sin(q[2])*v[1];wref=v[2];cmd=np.array([vref,wref]);
   if args.mode=='feedback':cmd+=np.array([1.0*ex,1.5*eth+1.5*ey*(1 if vref>=0 else -1)])
   # The P1 profile bounds world x/y components, not planar speed norm.
   # Intersect velocity and finite-difference acceleration bounds for both components.
   axis=np.array([math.cos(y),math.sin(y)]);cap=.5/max(abs(axis));low,high=-cap,cap
   for j in range(2):
    if abs(axis[j])<1e-10:
     if abs(last_world[j])>dt+1e-8:raise RuntimeError('NO_FEASIBLE_COMPONENT_ACCELERATION')
     continue
    ends=sorted(((last_world[j]-dt)/axis[j],(last_world[j]+dt)/axis[j]));low=max(low,ends[0]);high=min(high,ends[1])
   if low>high+1e-8:raise RuntimeError('NO_FEASIBLE_COMPONENT_ACCELERATION')
   cmd[0]=np.clip(cmd[0],low,high);cmd[1]=np.clip(cmd[1],max(-.5,lastcmd[1]-dt),min(.5,lastcmd[1]+dt));wheels=np.asarray(diff.forward(cmd).joint_velocities);vact=math.cos(y)*lin[0]+math.sin(y)*lin[1];lateral=-math.sin(y)*lin[0]+math.cos(y)*lin[1];xy=float(math.hypot(dx,dy));wheel_read=[rigid.get_world_pose() for rigid in wheels_rigid];wheel_z=[float(x[0][2]) for x in wheel_read];wheel_gap=[]
   for wp,wq in wheel_read:
    ww,xx,yy,zz=wq;axis_z=2*(yy*zz+ww*xx);extent=.06*math.sqrt(max(0.,1-axis_z**2))+.02*abs(axis_z);wheel_gap.append(float(wp[2]-extent-.034224))
   roll=math.atan2(2*(quat[0]*quat[1]+quat[2]*quat[3]),1-2*(quat[1]**2+quat[2]**2));pitch=math.asin(max(-1.,min(1.,2*(quat[0]*quat[2]-quat[3]*quat[1]))))
   tip_pos,tip_quat=tip.get_world_pose()
   record=[now,t,pos[0],pos[1],y,lin[0],lin[1],ang[2],cmd[0],cmd[1],*applied_wheels,*jw,*ja,*jv,*tip_pos,*tip_quat];writer.writerow(record)
   row=[now,t,q[0],q[1],q[2],pos[0],pos[1],y,pos[2],vref,wref,cmd[0],cmd[1],vact,lateral,ang[2],*applied_wheels,*jw,xy,eth,*q[3:],*ja,*v[3:],*jv,*lin,roll,pitch,*wheel_z,*wheel_gap];rows.append([float(x) for x in row])
   if n%2400==0:print('PROGRESS',n,'reference_s',t,'xy_error_m',xy,flush=True)
   if not np.isfinite(row).all() or pos[2]<-.15 or pos[2]>1. or abs(roll)>.6 or abs(pitch)>.6 or xy>1.:raise RuntimeError('UNSTABLE_STATE t=%.6f xyz=%s roll=%.4f pitch=%.4f xy_error=%.4f'%(t,pos.tolist(),roll,pitch,xy))
   if n==steps:break
   ctrl.apply_action(ArticulationAction(joint_velocities=wheels,joint_indices=wi));ctrl.apply_action(ArticulationAction(joint_positions=q[3:],joint_indices=ai));lastcmd=cmd;last_world=cmd[0]*axis;applied_wheels=wheels.copy();step(n)
   if args.gui:
    delay=wall+(float(world.current_time)-start_sim)-time.monotonic()
    if delay>0:time.sleep(delay)
 a=np.asarray(rows);xy=a[:,20];angle=a[:,21];je=a[:,28:34]-a[:,22:28];we=a[:,18:20]-a[:,16:18];lat=a[:,14];qrange=np.ptp(np.vstack([interp(qs,args.start+k*args.duration/steps)[3:] for k in range(steps+1)]),axis=0)
 result.update(status='COMPLETED',physics_hz=args.physics_hz,solver='PGS',joint2_stiffness=2000,trajectory=args.trajectory,stage=args.stage,window_s=[args.start,args.start+args.duration],sim_duration_s=float(a[-1,0]-a[0,0]),updates=len(a),initial_actual=initial_read,reference_delta_xy=(a[-1,2:4]-a[0,2:4]).tolist(),reference_yaw_delta_rad=float(a[-1,4]-a[0,4]),reference_joint_range_rad=qrange.tolist(),xy_rmse_m=float(np.sqrt(np.mean(xy**2))),xy_max_m=float(np.max(xy)),yaw_rmse_rad=float(np.sqrt(np.mean(angle**2))),yaw_max_rad=float(np.max(np.abs(angle))),joint_max_abs_rad=np.max(np.abs(je),axis=0).tolist(),wheel_max_abs_error_rad_s=np.max(np.abs(we),axis=0).tolist(),lateral_velocity_max_m_s=float(np.max(np.abs(lat))),lateral_velocity_rms_m_s=float(np.sqrt(np.mean(lat**2))),end_xy_error_m=float(xy[-1]),end_yaw_error_rad=float(angle[-1]),joint_end_error_rad=je[-1].tolist(),root_z_range_m=[float(np.min(a[:,8])),float(np.max(a[:,8]))],max_command_linear=float(np.max(np.abs(a[:,11]))),max_command_angular=float(np.max(np.abs(a[:,12]))),wall_s=time.monotonic()-wall,limitations='experimental low-friction support and cylinder wheel collision; no P1 continuous collision or physical parameter validation')
 print('TRACKING_RESULT',json.dumps(result),flush=True)
except Exception as e:
 result.update(status='FAIL',error=repr(e),last_trajectory_time_s=float(t) if 't' in locals() else None,recorded_samples=len(rows) if 'rows' in locals() else 0);traceback.print_exc()
finally:
 (out/'metrics.json').write_text(json.dumps(result,indent=2)+'\n')
 if args.gui and args.hold_open:
  try:
   world.pause();print('GUI_HOLD_OPEN',flush=True)
   while app.is_running():app.update();time.sleep(.03)
  except Exception:traceback.print_exc()
 app.close()
sys.exit(0 if result['status']=='COMPLETED' else 1)
