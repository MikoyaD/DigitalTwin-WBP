#!/usr/bin/env python3
"""Publish multiple planning and execution paths; no robot commands."""
import argparse,csv,json,math
from pathlib import Path
import numpy as np
from v2.core.kdl_fk import LocalFK
from v2.geometry.tm_rho_feedback_frozen import thaw

ROOT=Path(__file__).resolve().parents[2]
N=['world_car/x','world_car/y','world_car/theta']+['joint%d'%i for i in range(1,7)]

def read_source(kind,path,spacing=.002):
 p=Path(path).expanduser().resolve(strict=True)
 if kind=='P':
  x=json.loads((p/'parent_path.json' if p.is_dir() else p).read_text());return np.asarray(x['path']['q'],float),None,None
 if kind=='R':
  x=json.loads((p/'geometry.json' if p.is_dir() else p).read_text());block=thaw(x);qs=[]
  for c in block.charts:
   n=max(2,int(math.ceil((c.s1-c.s0)/spacing))+1);q=np.asarray(c.evaluate_global(np.linspace(c.s0,c.s1,n)).q);qs.extend(q if not qs else q[1:])
  return np.asarray(qs,float),None,None
 with p.open(newline='') as f:rows=list(csv.DictReader(f))
 if not rows:raise ValueError('EMPTY_SOURCE')
 if kind=='execution':
  q=np.array([[float(r[x]) for x in ('x_actual','y_actual','yaw_actual')]+[float(r['joint%d_actual'%i]) for i in range(1,7)] for r in rows]);t=np.array([float(r['trajectory_time_s']) for r in rows]);end=np.array([[float(r[x]) for x in ('link6_x','link6_y','link6_z')] for r in rows]);return q,t,end
 q=np.array([[float(r['q_'+x]) for x in N] for r in rows]);t=np.array([float(r['time_s']) for r in rows]);return q,t,None

def main():
 import rospy
 from geometry_msgs.msg import Point,Pose
 from visualization_msgs.msg import Marker,MarkerArray
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--color-index',type=int,default=0);p.add_argument('--source',action='append',required=True,help='LABEL:TYPE:PATH; TYPE P,R,trajectory,execution');p.add_argument('--goal',action='append',default=[],help='ID:X:Y:Z task target in world metres');p.add_argument('--interval',type=float,default=2.);p.add_argument('--arrow-stride',type=int,default=80);p.add_argument('--topic',default='/v2/trajectory_overlay');args=p.parse_args()
 if args.interval<=0 or args.arrow_stride<1:raise ValueError('INVALID_SAMPLING')
 rospy.init_node('v2_trajectory_overlay',anonymous=True);pub=rospy.Publisher(args.topic,MarkerArray,queue_size=1,latch=True);fk=LocalFK((ROOT/'config/gazebo_combined_robot.urdf').read_text());palette=[(1.,.2,.15),(.1,.8,.3),(.1,.5,1.),(1.,.65,.1),(.8,.2,1.),(.1,.9,.9)];msg=MarkerArray();marker_id=0
 def marker(ns,typ,col):
  nonlocal marker_id
  m=Marker();m.header.frame_id='world';m.header.stamp=rospy.Time.now();m.ns=ns;m.id=marker_id;marker_id+=1;m.type=typ;m.action=Marker.ADD;m.pose.orientation.w=1;m.color.r,m.color.g,m.color.b=col;m.color.a=1.;return m
 for index,source in enumerate(args.source):
  label,kind,path=source.split(':',2);q,t,actual_ee=read_source(kind,path);col=palette[(index+args.color_index)%len(palette)];print('LEGEND',label,'RGB',col,'base=thick Link6=thin yaw=arrows time=points',flush=True);ee=actual_ee if actual_ee is not None else np.array([fk.pose(z)[0] for z in q]);
  for name,xyz in [('base',np.column_stack((q[:,0:2],np.full(len(q),.08)))),('Link6',ee)]:
   m=marker(label+'/'+name,Marker.LINE_STRIP,col);m.scale.x=.025 if name=='base' else .015;m.points=[Point(*map(float,x)) for x in xyz];msg.markers.append(m)
  for j in range(0,len(q),args.arrow_stride):
   m=marker(label+'/yaw',Marker.ARROW,col);m.pose.position=Point(float(q[j,0]),float(q[j,1]),.1);m.pose.orientation.z=math.sin(q[j,2]/2);m.pose.orientation.w=math.cos(q[j,2]/2);m.scale.x=.18;m.scale.y=.035;m.scale.z=.035;msg.markers.append(m)
  if t is not None:
   ix=np.unique(np.searchsorted(t,np.arange(t[0],t[-1]+1e-9,args.interval)).clip(0,len(t)-1));m=marker(label+'/time_marks',Marker.SPHERE_LIST,col);m.scale.x=m.scale.y=m.scale.z=.08;m.points=[Point(float(q[j,0]),float(q[j,1]),.12) for j in ix];msg.markers.append(m)
  for j,role in ((0,'start'),(-1,'end')):
   m=marker(label+'/'+role,Marker.TEXT_VIEW_FACING,col);m.pose.position=Point(float(q[j,0]),float(q[j,1]),.35);m.scale.z=.14;m.text=label+' '+role;msg.markers.append(m)
  print('SOURCE',label,kind,'states',len(q),'time_marks',0 if t is None else len(ix),'start',q[0].tolist(),'end',q[-1].tolist(),'link6_start',ee[0].tolist(),flush=True)
 for spec in args.goal:
  name,x,y,z=spec.split(':');m=marker('task_goal',Marker.TEXT_VIEW_FACING,(1.,1.,1.));m.pose.position=Point(float(x),float(y),float(z));m.scale.z=.18;m.text='Goal '+name;msg.markers.append(m)
 pub.publish(msg);print('PUBLISHED',args.topic,'visualization_msgs/MarkerArray','markers',len(msg.markers),flush=True);rospy.spin()
if __name__=='__main__':main()
