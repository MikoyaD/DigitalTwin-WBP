"""Bounded nine-variable IK using the live MoveIt model; no planning or execution."""
import copy,math,time,xml.etree.ElementTree as ET
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from ..core.contracts import NAMES
from ..adapters.workspace_check import parse_workspace,bounds

class JointIK:
 def __init__(self,config,pose):
  import rospy
  from moveit_msgs.srv import GetPositionFK,GetStateValidity,GetPlanningScene,GetPlanningSceneRequest
  from moveit_msgs.msg import PlanningSceneComponents
  self.rospy=rospy
  if not rospy.core.is_initialized():rospy.init_node('v2_joint_ik',anonymous=True,disable_signals=True)
  for n in ('/compute_fk','/check_state_validity','/get_planning_scene'):rospy.wait_for_service(n,timeout=5)
  self.fk=rospy.ServiceProxy('/compute_fk',GetPositionFK);self.valid=rospy.ServiceProxy('/check_state_validity',GetStateValidity)
  r=GetPlanningSceneRequest();r.components.components=PlanningSceneComponents.ROBOT_STATE|PlanningSceneComponents.OCTOMAP
  self.scene=rospy.ServiceProxy('/get_planning_scene',GetPlanningScene)(r).scene
  if not self.scene.world.octomap.octomap.data:raise ValueError('PLANNING_SCENE_OCTOMAP_MISSING')
  if pose.get('frame_id','world')!='world' or pose.get('link_name','Link6')!='Link6':raise ValueError('POSE_REQUIRES_WORLD_AND_Link6')
  self.p=np.asarray(pose['position'],float)
  if self.p.shape!=(3,) or not np.all(np.isfinite(self.p)):raise ValueError('INVALID_TARGET_POSITION')
  if ('orientation_xyzw' in pose)==('rpy' in pose):raise ValueError('SPECIFY_ONE_ORIENTATION')
  if 'rpy' in pose:
   a=np.asarray(pose['rpy'],float)
   if a.shape!=(3,) or not np.all(np.isfinite(a)):raise ValueError('INVALID_RPY')
   self.rot=Rotation.from_euler('xyz',a)
  else:
   a=np.asarray(pose['orientation_xyzw'],float)
   if a.shape!=(4,) or not np.all(np.isfinite(a)) or abs(np.linalg.norm(a)-1)>1e-6:raise ValueError('INVALID_QUATERNION')
   self.rot=Rotation.from_quat(a)
  self.urdf=rospy.get_param('/robot_description');tree=ET.fromstring(self.urdf);js=[j for j in tree.findall('joint') if j.find('parent') is not None];bychild={j.find('child').get('link'):j for j in js};chain=[];link='Link6'
  try:
   from ..core.kdl_fk import LocalFK
   self.local_fk=LocalFK(self.urdf)
  except ImportError:
   self.local_fk=None  # Preserve ROS FK operation if PyKDL is not installed.
  while link!='car_link':
   j=bychild[link];chain.append(j);link=j.find('parent').get('link')
  chain.reverse()
  if [j.get('name') for j in chain]!=['base_to_car']+list(NAMES[3:]):raise ValueError('UNEXPECTED_KINEMATIC_CHAIN')
  def origin(j):
   o=j.find('origin');return np.array([float(x) for x in (o.get('xyz','0 0 0') if o is not None else '0 0 0').split()])
  o=chain[0].find('origin');rpy=[float(x) for x in o.get('rpy','0 0 0').split()];mount=origin(chain[0])+Rotation.from_euler('xyz',rpy).apply(origin(chain[1]));self.radius=float(np.linalg.norm(mount[:2])+sum(np.linalg.norm(origin(j)) for j in chain[2:]));self.mount=mount.tolist()
  self.workspace=parse_workspace(config)
  if self.workspace is None:raise ValueError('EXPLICIT_WORKSPACE_REQUIRED')
  lo=self.workspace['min_corner'];hi=self.workspace['max_corner'];limits={j.get('name'):(float(j.find('limit').get('lower')),float(j.find('limit').get('upper'))) for j in chain[1:]}
  self.lo=np.array([max(lo['x'],self.p[0]-self.radius),max(lo['y'],self.p[1]-self.radius),-math.pi]+[limits[n][0] for n in NAMES[3:]])
  self.hi=np.array([min(hi['x'],self.p[0]+self.radius),min(hi['y'],self.p[1]+self.radius),math.pi]+[limits[n][1] for n in NAMES[3:]])
  if np.any(self.lo>=self.hi):raise ValueError('EMPTY_IK_WORKSPACE')
 def state(self,q):
  from geometry_msgs.msg import Transform
  s=copy.deepcopy(self.scene.robot_state);s.is_diff=False;js=s.joint_state;js.name=list(js.name);js.position=list(js.position)
  for n,v in zip(NAMES[3:],q[3:]):
   if n in js.name:js.position[js.name.index(n)]=float(v)
   else:js.name.append(n);js.position.append(float(v))
  md=s.multi_dof_joint_state;md.joint_names=list(md.joint_names);md.transforms=list(md.transforms);md.header.frame_id='world';tr=Transform();tr.translation.x=float(q[0]);tr.translation.y=float(q[1]);tr.rotation.z=math.sin(q[2]/2);tr.rotation.w=math.cos(q[2]/2)
  if 'world_car' in md.joint_names:md.transforms[md.joint_names.index('world_car')]=tr
  else:md.joint_names.append('world_car');md.transforms.append(tr)
  return s
 def pose(self,q):
  from moveit_msgs.srv import GetPositionFKRequest
  r=GetPositionFKRequest();r.header.frame_id='world';r.fk_link_names=['Link6'];r.robot_state=self.state(q);z=self.fk(r)
  if z.error_code.val!=1:raise ValueError('FK_ERROR %s'%z.error_code.val)
  p=z.pose_stamped[0].pose;return np.array([p.position.x,p.position.y,p.position.z]),Rotation.from_quat([p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w])
 def check(self,q):
  from moveit_msgs.srv import GetStateValidityRequest
  q=np.asarray(q,float);p,r=self.pose(q);ep=float(np.linalg.norm(p-self.p));er=float(np.linalg.norm((self.rot.inv()*r).as_rotvec()));b=bounds(q,self.urdf,self.workspace);circle=bool(np.linalg.norm(q[:2]-self.p[:2])<=self.radius+1e-9);v=GetStateValidityRequest();v.group_name='whole';v.robot_state=self.state(q);z=self.valid(v)
  return dict(position_error_m=ep,orientation_error_rad=er,bounds=b,circle_pass=circle,scene_valid=bool(z.valid),contacts=[dict(a=c.contact_body_1,b=c.contact_body_2) for c in z.contacts],ik_feasible=bool(ep<=.01 and er<=.01 and b['status']=='PASS' and circle and z.valid))
 def solve(self,start,attempt):
  seed=np.asarray(start,float).copy()
  if seed.shape!=(9,) or not np.all(np.isfinite(seed)):raise ValueError('START_Q_MUST_HAVE_NINE_FINITE_VALUES')
  seed[2]=math.atan2(math.sin(seed[2]),math.cos(seed[2]));d=seed[:2]-self.p[:2];dist=np.linalg.norm(d)
  if dist>.9*self.radius:seed[:2]=self.p[:2]+d/dist*(.9*self.radius)
  if attempt>1:seed[3:]+=np.array([.08,-.06,.04,-.04,.05,-.03])*(attempt-1)
  seed=np.minimum(np.maximum(seed,self.lo+1e-8),self.hi-1e-8);n=[0];ros_calls=[0]
  def residual(q):
   n[0]+=1
   if self.local_fk is None:p,r=self.pose(q);ros_calls[0]+=1
   else:p,r=self.local_fk.pose(q)
   return np.r_[p-self.p,(self.rot.inv()*r).as_rotvec(),1e-5*(q-seed)]
  t=time.monotonic();posterior_wall=0.;q=seed.copy();evaluations=0;check=None;termination='EVALUATION_BUDGET';z=None
  # SciPy 1.3 has no least_squares callback. Examine only the returned iterate,
  # never a finite-difference probe; a stricter trigger leaves planning margin.
  for _ in range(30):
   z=least_squares(residual,q,bounds=(self.lo,self.hi),diff_step=1e-5,xtol=1e-10,ftol=1e-10,gtol=1e-10,max_nfev=2);evaluations+=int(z.nfev);q=z.x
   if self.local_fk is None:p,r=self.pose(q);ros_calls[0]+=1
   else:p,r=self.local_fk.pose(q);n[0]+=1
   if np.linalg.norm(p-self.p)<=.005 and np.linalg.norm((self.rot.inv()*r).as_rotvec())<=.005 and np.linalg.norm(q[:2]-self.p[:2])<=self.radius+1e-9:
    check_start=time.monotonic();check=self.check(q);posterior_wall+=time.monotonic()-check_start;ros_calls[0]+=1
    if check['ik_feasible']:termination='TASK_FEASIBLE_POSTERIOR_PASS';break
   if z.success:termination='SCIPY_CONVERGED';break
  if check is None or not check['ik_feasible']:
   # Segmentation changes the trust-region history. Preserve the old single-run
   # search as a fallback when no checked feasible iterate was found.
   z=least_squares(residual,seed,bounds=(self.lo,self.hi),diff_step=1e-5,xtol=1e-10,ftol=1e-10,gtol=1e-10,max_nfev=60);evaluations+=int(z.nfev);q=z.x;check_start=time.monotonic();check=self.check(q);posterior_wall+=time.monotonic()-check_start;ros_calls[0]+=1;termination='ORIGINAL_SEARCH_FALLBACK'
  total_wall=time.monotonic()-t
  return dict(q=q.tolist(),initial_q=seed.tolist(),solver_success=bool(z.success),solver_status=int(z.status),solver_message=z.message,termination_reason=termination,nfev=evaluations,fk_calls=n[0],ros_fk_calls=ros_calls[0],solver_wall_s=total_wall-posterior_wall,posterior_wall_s=posterior_wall,total_wall_s=total_wall,radius_m=self.radius,mount_xyz_m=self.mount,bounds_min=self.lo.tolist(),bounds_max=self.hi.tolist(),**check)
 def target(self):return dict(frame_id='world',link_name='Link6',position=self.p.tolist(),orientation_xyzw=self.rot.as_quat().tolist(),position_tolerance_m=.01,orientation_tolerance_rad=.01)
