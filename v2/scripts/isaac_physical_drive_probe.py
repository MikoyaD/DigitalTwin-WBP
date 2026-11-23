"""Isolated short PhysX wheel/arm experiment. No pose resets after initialization."""
import argparse,csv,json,math,sys,time,traceback
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--gui',action='store_true');p.add_argument('--hold-open',action='store_true');p.add_argument('--stage',default='/home/robot/digitaltwin/zzs_ws/src/cr5_car_moveit/v2/assets/isaac_import_20261124/warehouse_color_overlay.usda');p.add_argument('--wheel-damping',type=float,default=30.0);p.add_argument('--low-support-friction',action='store_true');p.add_argument('--cylinder-wheel',action='store_true');p.add_argument('--wheel-effort',type=float,default=10.0);p.add_argument('--physics-hz',type=int,default=240);p.add_argument('--arm-stiffness',type=float,default=500.0);p.add_argument('--arm-damping',type=float,default=50.0);args=p.parse_args();sys.argv=[sys.argv[0],'--disable','omni.isaac.ros2_bridge','--disable','omni.isaac.ros_bridge']
from isaacsim import SimulationApp
app=SimulationApp({'headless':not args.gui,'renderer':'RayTracedLighting','width':960 if args.gui else 320,'height':540 if args.gui else 240,'multi_gpu':False})
out=Path(args.output);out.mkdir(parents=True,exist_ok=True);result={'status':'STARTED','stage':args.stage,'gui':args.gui,'scope':'physical contact wheel velocity and arm position, not P1 tracking'}
try:
 import numpy as np,omni.usd
 from pxr import UsdGeom,UsdPhysics,UsdShade,PhysxSchema,Gf
 from omni.isaac.core import World
 from omni.isaac.core.articulations import Articulation
 from omni.isaac.core.utils.types import ArticulationAction
 from omni.isaac.wheeled_robots.controllers import DifferentialController
 if not omni.usd.get_context().open_stage(args.stage):raise RuntimeError('STAGE_OPEN_FAILED')
 stage=omni.usd.get_context().get_stage();stage.SetEditTarget(stage.GetSessionLayer())
 ground=UsdGeom.Cube.Define(stage,'/Warehouse/PhysicsExperimentGround');ground.CreateSizeAttr(1);ground.AddTranslateOp().Set(Gf.Vec3d(0.,0.,.034224-.01));ground.AddScaleOp().Set(Gf.Vec3f(40.,40.,.02));UsdPhysics.CollisionAPI.Apply(ground.GetPrim());ground.MakeInvisible()
 if args.low_support_friction:
  mat=UsdShade.Material.Define(stage,'/Warehouse/ExperimentLowFriction')
  m=UsdPhysics.MaterialAPI.Apply(mat.GetPrim());m.CreateStaticFrictionAttr(0.0);m.CreateDynamicFrictionAttr(0.0);m.CreateRestitutionAttr(0.0)
  pm=PhysxSchema.PhysxMaterialAPI.Apply(mat.GetPrim());pm.CreateFrictionCombineModeAttr('min')
  for wheel in ('ball_left_link','ball_right_link'):
   collider=stage.GetPrimAtPath('/Warehouse/robot_with_default_prim/'+wheel+'/collisions')
   UsdShade.MaterialBindingAPI.Apply(collider).Bind(mat,materialPurpose='physics')
 if args.cylinder_wheel:
  for wheel in ('wheel_left_link','wheel_right_link'):
   link='/Warehouse/robot_with_default_prim/'+wheel
   mesh=stage.GetPrimAtPath(link+'/collisions');UsdPhysics.CollisionAPI(mesh).GetCollisionEnabledAttr().Set(False)
   cylinder=UsdGeom.Cylinder.Define(stage,link+'/ExperimentCylinder');cylinder.CreateAxisAttr('Y');cylinder.CreateRadiusAttr(.06);cylinder.CreateHeightAttr(.04);UsdPhysics.CollisionAPI.Apply(cylinder.GetPrim())
 root='/Warehouse/robot_with_default_prim/car_link';world=World(stage_units_in_meters=1.,physics_dt=1/args.physics_hz,rendering_dt=1/60);world.reset();art=Articulation(prim_path=root);art.initialize()
 if not art.handles_initialized:raise RuntimeError('ARTICULATION_INIT_FAILED')
 names=list(art.dof_names);wi=np.array([names.index('wheel_left_joint'),names.index('wheel_right_joint')]);ai=np.array([names.index('joint%d'%j) for j in range(1,7)]);ctrl=art.get_articulation_controller();kp,kd=ctrl.get_gains();kp=np.array(kp);kd=np.array(kd);kp[wi]=0;kd[wi]=args.wheel_damping;kp[ai]=args.arm_stiffness;kd[ai]=args.arm_damping;ctrl.set_gains(kps=kp,kds=kd,save_to_usd=False);ctrl.set_max_efforts(np.array([args.wheel_effort,args.wheel_effort]),joint_indices=wi)
 # One-time initial placement. All subsequently observed motion comes from PhysX drives and contact.
 yaw0=1.7959;art.set_world_pose(position=np.array([.4315,-7.7584,.034224]),orientation=np.array([math.cos(yaw0/2),0,0,math.sin(yaw0/2)]));art.set_linear_velocity(np.zeros(3));art.set_angular_velocity(np.zeros(3));art.set_joint_velocities(np.zeros(len(names)));initial=np.array(art.get_joint_positions(joint_indices=ai));target=initial+np.array([.04,-.04,.035,-.035,.03,-.03]);diff=DifferentialController(name='diff',wheel_radius=.06,wheel_base=.395)
 def euler(q):
  w,x,y,z=q
  return (math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)),math.asin(max(-1.,min(1.,2*(w*y-z*x)))),math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)))
 def adiff(a,b):return math.atan2(math.sin(a-b),math.cos(a-b))
 fields=['sim_time_s','phase','v_ref_m_s','omega_ref_rad_s','wheel_left_ref_rad_s','wheel_right_ref_rad_s','wheel_left_actual_rad_s','wheel_right_actual_rad_s','x_ref','y_ref','yaw_ref','x_actual','y_actual','z_actual','roll_actual','pitch_actual','yaw_actual','vx_actual','vy_actual','vz_actual','wx_actual','wy_actual','wz_actual']+['joint%d_ref'%j for j in range(1,7)]+['joint%d_actual'%j for j in range(1,7)]+['joint%d_vel'%j for j in range(1,7)]
 records=[];phase_start={};ref=None;wall=time.monotonic();sim_start=float(world.current_time);previous_time=sim_start;phase_list=[('settle',2.,0.,0.,False),('forward',2.,.12,0.,False),('stop',.5,0.,0.,False),('rotate',2.,0.,.3,False),('arm',3.,0.,0.,True)]
 with (out/'states.csv').open('x',newline='') as f:
  w=csv.writer(f);w.writerow(fields)
  for phase,duration,v,omega,move_arm in phase_list:
   print('PHASE_START',phase,flush=True)
   for n in range(round(duration*args.physics_hz)):
    wheels=np.asarray(diff.forward(np.array([v,omega])).joint_velocities);arm=target if move_arm else initial
    ctrl.apply_action(ArticulationAction(joint_velocities=wheels,joint_indices=wi));ctrl.apply_action(ArticulationAction(joint_positions=arm,joint_indices=ai));world.step(render=False)
    now=float(world.current_time)
    if abs((now-previous_time)-1/args.physics_hz)>1e-5:raise RuntimeError('UNEXPECTED_SIM_TIME_STEP: '+str(now-previous_time))
    previous_time=now
    if args.gui and n%4==0:
     world.render()
     if abs(float(world.current_time)-now)>1e-9:raise RuntimeError('RENDER_ADVANCED_PHYSICS')
    pos,quat=art.get_world_pose();roll,pitch,yaw=euler(quat);vel=np.array(art.get_linear_velocity());ang=np.array(art.get_angular_velocity());wv=np.array(art.get_joint_velocities(joint_indices=wi));q=np.array(art.get_joint_positions(joint_indices=ai));qv=np.array(art.get_joint_velocities(joint_indices=ai))
    if phase not in phase_start:phase_start[phase]={'p':pos.tolist(),'yaw':yaw,'q':q.tolist(),'time':now};ref=[float(pos[0]),float(pos[1]),float(yaw)]
    if n>0:
     dt=1/args.physics_hz
     if abs(omega)<1e-12:ref[0]+=v*math.cos(ref[2])*dt;ref[1]+=v*math.sin(ref[2])*dt
     else:ref[0]+=v/omega*(math.sin(ref[2]+omega*dt)-math.sin(ref[2]));ref[1]-=v/omega*(math.cos(ref[2]+omega*dt)-math.cos(ref[2]));ref[2]+=omega*dt
    row=[now,phase,v,omega,*wheels,*wv,*ref,*pos,roll,pitch,yaw,*vel,*ang,*arm,*q,*qv];w.writerow(row);records.append(dict(phase=phase,t=now,p=pos.tolist(),rpy=[roll,pitch,yaw],v=vel.tolist(),ang=ang.tolist(),wv=wv.tolist(),wref=wheels.tolist(),q=q.tolist(),qref=arm.tolist(),qv=qv.tolist(),ref=ref.copy()))
    if args.gui:
     delay=wall+(now-sim_start)-time.monotonic()
     if delay>0:time.sleep(delay)
    if not np.isfinite(np.array(row[:1]+row[2:],dtype=float)).all() or abs(roll)>1.0 or abs(pitch)>1.0 or pos[2]<-.15 or pos[2]>1.0:raise RuntimeError('UNSTABLE_SAFETY_STOP '+phase+' '+str(row[:25]))
   print('PHASE_END',phase,'xyz',pos.tolist(),'rpy',[roll,pitch,yaw],'wheel',wv.tolist(),'arm',q.tolist(),flush=True)
 def stats(phase):
  r=[x for x in records if x['phase']==phase];a=r[0];b=r[-1];delta=np.array(b['p'])-a['p'];forward=float(delta[0]*math.cos(a['rpy'][2])+delta[1]*math.sin(a['rpy'][2]));side=float(-delta[0]*math.sin(a['rpy'][2])+delta[1]*math.cos(a['rpy'][2]));j=np.array([x['q'] for x in r]);jref=np.array([x['qref'] for x in r]);wv=np.array([x['wv'] for x in r]);wr=np.array([x['wref'] for x in r]);return {'samples':len(r),'duration_s':b['t']-a['t'],'start_xyz':a['p'],'end_xyz':b['p'],'delta_xyz_m':delta.tolist(),'forward_displacement_m':forward,'lateral_displacement_m':side,'yaw_change_rad':adiff(b['rpy'][2],a['rpy'][2]),'ideal_end_xy_yaw':b['ref'],'position_error_m':float(np.linalg.norm(np.array(b['p'][:2])-b['ref'][:2])),'yaw_error_rad':adiff(b['rpy'][2],b['ref'][2]),'z_min_max_m':[float(min(x['p'][2] for x in r)),float(max(x['p'][2] for x in r))],'max_abs_roll_pitch_rad':float(max(max(abs(x['rpy'][0]),abs(x['rpy'][1])) for x in r)),'wheel_max_abs_error_rad_s':np.max(np.abs(wv-wr),axis=0).tolist(),'wheel_mean_last_half_rad_s':np.mean(wv[len(wv)//2:],axis=0).tolist(),'root_final_linear_velocity_m_s':b['v'],'root_final_angular_velocity_rad_s':b['ang'],'arm_max_error_rad':np.max(np.abs(j-jref),axis=0).tolist(),'arm_final_error_rad':(j[-1]-jref[-1]).tolist(),'arm_displacement_rad':(j[-1]-j[0]).tolist(),'max_joint_velocity_rad_s':np.max(np.abs(np.array([x['qv'] for x in r])),axis=0).tolist()}
 result.update(status='COMPLETED',dof_names=names,wheel_indices=wi.tolist(),arm_indices=ai.tolist(),ground_z_m=.034224,cylinder_wheel=args.cylinder_wheel,low_support_friction=args.low_support_friction,session_only=True,gravity='USD/physics default, enabled',wheel_damping=args.wheel_damping,wheel_effort=args.wheel_effort,physics_hz=args.physics_hz,arm_stiffness=args.arm_stiffness,arm_damping=args.arm_damping,phases={x[0]:stats(x[0]) for x in phase_list},wall_s=time.monotonic()-wall)
 print('PHYSICAL_RESULT',json.dumps(result),flush=True)
except Exception as e:
 result.update(status='FAIL',error=repr(e));traceback.print_exc()
finally:
 (out/'metrics.json').write_text(json.dumps(result,indent=2)+'\n')
 if args.gui and args.hold_open:
  try:
   world.pause();print('GUI_HOLD_OPEN close window after observation',flush=True)
   while app.is_running():app.update();time.sleep(.03)
  except Exception:traceback.print_exc()
 app.close()
sys.exit(0 if result['status']=='COMPLETED' else 1)
