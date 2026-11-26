#!/usr/bin/env python3
"""RViz task editor: planar markers, six angular dials, Link6 pose marker and console commands."""
import argparse,copy,math,subprocess,sys,xml.etree.ElementTree as ET
from scipy.spatial.transform import Rotation
from pathlib import Path
import numpy as np,yaml
import rospy
from geometry_msgs.msg import Pose
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker,InteractiveMarkerControl,Marker
from moveit_msgs.msg import DisplayRobotState,PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene,GetPlanningSceneRequest
from tf.transformations import euler_from_quaternion
from v2.core.kdl_fk import LocalFK
from v2.scripts.joint_ik import JointIK
P=Path(__file__).resolve().parents[2]
class Editor:
 def __init__(self,config=None):
  self.cfg=yaml.safe_load(Path(config or P/'v2/configs/global.yaml').read_text());self.start=np.array(self.cfg['task']['start_q'],float);self.end=self.start.copy();self.selected='start';self.goals=[];self.mode='pose';self.fk=LocalFK((P/'config/gazebo_combined_robot.urdf').read_text());self.pose=Pose();pos,rot=self.fk.pose(self.end);self.pose.position.x,self.pose.position.y,self.pose.position.z=pos;self.pose.orientation.x,self.pose.orientation.y,self.pose.orientation.z,self.pose.orientation.w=rot.as_quat();self._preview_ik=self.state_ik(self.start); self._joint_chain=self.build_chain();self.server=InteractiveMarkerServer('v2_task_editor');self.pub=rospy.Publisher('/v2/interactive_robot_state',DisplayRobotState,queue_size=1,latch=True);self.load_config(config) if config else None;self.rebuild()
 def load_config(self,path):
  self.cfg=yaml.safe_load(Path(path).expanduser().read_text());task=self.cfg['task'];self.start=np.array(task['start_q'],float);self.end=self.start.copy();self.goals=[];goal=task.get('goal') or {}
  if goal.get('type')=='joint_state':self.end=np.array(goal['q'],float);self.mode='state'
  elif goal.get('type')=='link_pose':
   p=goal['position_m'];o=goal['quaternion_xyzw'];self.pose.position.x,self.pose.position.y,self.pose.position.z=p;self.pose.orientation.x,self.pose.orientation.y,self.pose.orientation.z,self.pose.orientation.w=o;self.mode='pose'
  if task.get('mode')=='multi':self.goals=[{k:v for k,v in item.items() if k!='id'} for item in task.get('targets',[])]
 def active(self):return self.start if self.selected=='start' else self.end
 def state_ik(self,q):
  p,r=self.fk.pose(q);cfg=yaml.safe_load((P/'v2/configs/v2_workspace_single_goal_9d_va_only.yaml').read_text());return JointIK(cfg,{'frame_id':'world','link_name':'Link6','position':p.tolist(),'orientation_xyzw':r.as_quat().tolist()})
 def current(self):
  rospy.wait_for_service('/get_planning_scene',5);r=GetPlanningSceneRequest();r.components.components=PlanningSceneComponents.ROBOT_STATE;st=rospy.ServiceProxy('/get_planning_scene',GetPlanningScene)(r).scene.robot_state;js=st.joint_state;md=st.multi_dof_joint_state
  if 'world_car' not in md.joint_names:raise ValueError('CURRENT_WORLD_CAR_MISSING')
  tf=md.transforms[md.joint_names.index('world_car')];q=tf.rotation;yaw=euler_from_quaternion([q.x,q.y,q.z,q.w])[2];return np.array([tf.translation.x,tf.translation.y,yaw]+[js.position[list(js.name).index('joint%d'%i)] for i in range(1,7)])
 def validate(self,q):
  ik=self.state_ik(q);c=ik.check(q)
  if c['bounds']['status']!='PASS' or not c['scene_valid']:raise ValueError('STATE_INVALID '+str(c))
  return c
 def publish_state(self):
  msg=DisplayRobotState();msg.state=self._preview_ik.state(self.active());self.pub.publish(msg)
 def build_chain(self):
  tree=ET.fromstring((P/'config/gazebo_combined_robot.urdf').read_text()); joints={j.get('name'):j for j in tree.findall('joint')}; out=[]
  for name in ['base_to_car']+['joint%d'%i for i in range(1,7)]:
   j=joints[name];o=j.find('origin');a=j.find('axis');xyz=np.array([float(x) for x in (o.get('xyz','0 0 0') if o is not None else '0 0 0').split()]);rpy=[float(x) for x in (o.get('rpy','0 0 0') if o is not None else '0 0 0').split()];axis=np.array([float(x) for x in (a.get('xyz','1 0 0') if a is not None else '1 0 0').split()]);axis/=np.linalg.norm(axis);out.append((xyz,Rotation.from_euler('xyz',rpy),axis))
  return out
 def frames(self,q):
  pos=np.array([q[0],q[1],0.]);rot=Rotation.from_euler('z',q[2]);frames=[]
  for i,(xyz,origin,axis) in enumerate(self._joint_chain):
   pos=pos+rot.apply(xyz);rot=rot*origin
   if i:
    frames.append((pos.copy(),rot,axis));rot=rot*Rotation.from_rotvec(axis*q[i+2])
  return frames
 def update_markers(self):
  q=self.active()
  for i,(pos,rot,axis) in enumerate(self.frames(q)):
   pose=Pose();pose.position.x,pose.position.y,pose.position.z=pos;quat=(rot*Rotation.from_rotvec(axis*q[i+3])).as_quat();pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w=quat;self.server.setPose('joint%d'%(i+1),pose)
  self.server.applyChanges()
 def feedback(self,f):
  q=self.active();name=f.marker_name
  if name=='base':
   q[0]=f.pose.position.x;q[1]=f.pose.position.y;o=f.pose.orientation;q[2]=euler_from_quaternion([o.x,o.y,o.z,o.w])[2]
  elif name.startswith('joint'):
   i=int(name[5:])-1;_,rot,axis=self.frames(q)[i];o=f.pose.orientation;relative=rot.inv()*Rotation.from_quat([o.x,o.y,o.z,o.w]);q[3+i]=float(np.dot(relative.as_rotvec(),axis))
  elif name=='Link6_target':self.pose=f.pose
  try:
   self.publish_state()
   if name=='base' or name.startswith('joint'):self.update_markers()
  except Exception as exc:rospy.logwarn('state preview: %s',exc)
 def make(self,name,pos,orientation=None):
  m=InteractiveMarker();m.header.frame_id='world';m.name=name;m.description=self.selected+' '+name;m.scale=.6;m.pose.position.x,m.pose.position.y,m.pose.position.z=pos
  if orientation is None:m.pose.orientation.w=1.
  else:m.pose.orientation.z=math.sin(orientation/2);m.pose.orientation.w=math.cos(orientation/2)
  return m
 def rebuild(self):
  self.server.clear();q=self.active();m=self.make('base',(q[0],q[1],.3),q[2]);c=InteractiveMarkerControl();c.name='xy';c.orientation.w=.70710678;c.orientation.y=.70710678;c.interaction_mode=InteractiveMarkerControl.MOVE_PLANE;m.controls.append(c);c=InteractiveMarkerControl();c.name='yaw';c.orientation.w=.70710678;c.orientation.y=.70710678;c.interaction_mode=InteractiveMarkerControl.ROTATE_AXIS;m.controls.append(c);self.server.insert(m,self.feedback)
  for i,(pos,rot,axis) in enumerate(self.frames(q)):
   m=self.make('joint%d'%(i+1),pos);quat=(rot*Rotation.from_rotvec(axis*q[i+3])).as_quat();m.pose.orientation.x,m.pose.orientation.y,m.pose.orientation.z,m.pose.orientation.w=quat;c=InteractiveMarkerControl();c.name='joint_angle';xaxis=np.array([1.,0.,0.]);dot=float(np.dot(xaxis,axis));cross=np.cross(xaxis,axis);v=np.r_[cross,1.+dot] if dot > -.999999 else np.array([0.,0.,1.,0.]);v=v/np.linalg.norm(v);c.orientation.x,c.orientation.y,c.orientation.z,c.orientation.w=v;c.interaction_mode=InteractiveMarkerControl.ROTATE_AXIS;m.controls.append(c);self.server.insert(m,self.feedback)
  m=self.make('Link6_target',(self.pose.position.x,self.pose.position.y,self.pose.position.z));m.pose.orientation=self.pose.orientation
  for name,axis in [('x',(1.,0.,0.)),('y',(0.,1.,0.)),('z',(0.,0.,1.))]:
   for mode in (InteractiveMarkerControl.MOVE_AXIS,InteractiveMarkerControl.ROTATE_AXIS):
    c=InteractiveMarkerControl();c.name=name+str(mode);c.orientation.w=.70710678;c.orientation.x,c.orientation.y,c.orientation.z=[.70710678*v for v in axis];c.interaction_mode=mode;m.controls.append(c)
  self.server.insert(m,self.feedback);self.server.applyChanges();self.publish_state()
 def target_pose(self):
  p=self.pose.position;o=self.pose.orientation;v=np.array([o.x,o.y,o.z,o.w]);norm=np.linalg.norm(v)
  if not np.isfinite(norm) or norm<1e-9:raise ValueError('INVALID_TARGET_ORIENTATION')
  return {'type':'link_pose','link':'Link6','frame':'world','position_m':[p.x,p.y,p.z],'quaternion_xyzw':(v/norm).tolist()}
 def save(self,path):
  self.validate(self.start);c=copy.deepcopy(self.cfg);c['task']['start_q']=self.start.tolist();c['output']['root']=str((P/'v2/outputs').resolve())
  if self.goals:
   if len(self.goals)!=5:raise ValueError('MULTI_REQUIRES_FIVE_GOALS')
   c['task']['mode']='multi';c['task']['targets']=[dict(x,id=str(i+1)) for i,x in enumerate(self.goals)]
  else:
   c['task']['mode']='single';self.validate(self.end) if self.mode=='state' else None;c['task']['goal']={'type':'joint_state','q':self.end.tolist()} if self.mode=='state' else self.target_pose()
  dest=Path(path).expanduser().resolve();dest.parent.mkdir(parents=True,exist_ok=True)
  with dest.open('x') as f:yaml.safe_dump(c,f,sort_keys=False)
  print('SAVED',dest,flush=True);return dest
 def loop(self):
  print('commands: edit start|goal; current; load PATH; mode pose|state; add; list; delete N; clear; save PATH; plan PATH; quit',flush=True)
  while not rospy.is_shutdown():
   try:
    x=input('task> ').split()
    if not x:continue
    if x[0]=='quit':break
    if x[0]=='edit':self.selected=x[1];assert self.selected in ('start','goal');self.rebuild()
    elif x[0]=='current':self.start=self.current();self.validate(self.start);self.rebuild();print('START',self.start.tolist())
    elif x[0]=='load':
     self.load_config(x[1]);self.validate(self.start);self.rebuild();print('LOADED_START',self.start.tolist(),'GOALS',len(self.goals))
    elif x[0]=='mode':self.mode=x[1];assert self.mode in ('pose','state')
    elif x[0]=='add':self.goals.append(self.target_pose());print('GOALS',len(self.goals))
    elif x[0]=='list':print('START',self.start.tolist(),'GOAL',self.end.tolist(),'POSE',self.target_pose(),'GOALS',self.goals)
    elif x[0]=='delete':del self.goals[int(x[1])-1]
    elif x[0]=='clear':self.goals=[]
    elif x[0] in ('save','plan'):
     path=self.save(x[1]);
     if x[0]=='plan':subprocess.run([str(P/'v2/scripts/plan_v2.sh'),'--config',str(path)],cwd=str(P),env=dict(__import__('os').environ,V2_MASTER_URI=rospy.get_master().getUri()),check=False)
    else:print('UNKNOWN_COMMAND')
   except (ValueError,IndexError,KeyError,AssertionError,OSError) as e:print('TASK_ERROR',str(e),flush=True)
if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--config',default=str(P/'v2/configs/global.yaml'));a=p.parse_args();rospy.init_node('v2_interactive_task',anonymous=True);Editor(a.config).loop()
