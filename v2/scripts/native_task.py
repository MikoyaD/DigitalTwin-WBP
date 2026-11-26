#!/usr/bin/env python3
"""Native MoveIt query editor; all captured data retain full precision."""
import argparse,copy,os,subprocess,datetime
from pathlib import Path
import numpy as np,yaml,rospy
from moveit_msgs.msg import RobotState
from tf.transformations import euler_from_quaternion
from v2.scripts.interactive_task import Editor,P
from v2.core.kdl_fk import LocalFK

def read_query(which):
 msg=rospy.wait_for_message('/v2/native_motionplanning/'+which,RobotState,timeout=5);j=msg.joint_state;m=msg.multi_dof_joint_state
 if 'world_car' not in m.joint_names:raise ValueError('原生状态缺少 world_car')
 tf=m.transforms[list(m.joint_names).index('world_car')];r=tf.rotation
 return np.array([tf.translation.x,tf.translation.y,euler_from_quaternion([r.x,r.y,r.z,r.w])[2]]+[j.position[list(j.name).index('joint%d'%i)] for i in range(1,7)],float)
def show_q(q):
 print('  底盘：x={:.6f} m  y={:.6f} m  yaw={:.6f} rad'.format(*q[:3]))
 print('  关节：'+'  '.join('joint{}={:.6f} rad'.format(i+1,v) for i,v in enumerate(q[3:])))
def show_goal(g):
 if g['type']=='joint_state':print('构型目标');show_q(g['q'])
 else:
  print('位姿目标：world / Link6');print('  位置：'+', '.join('{:.6f}'.format(v) for v in g['position_m'])+' m');print('  四元数 xyzw：'+', '.join('{:.6f}'.format(v) for v in g['quaternion_xyzw']))
class Native(Editor):
 def __init__(self,config):
  self.cfg=yaml.safe_load(Path(config).read_text());self.start=np.array(self.cfg['task']['start_q'],float);self.end=self.start.copy();self.goals=[];self.mode='state';self.fk=LocalFK((P/'config/gazebo_combined_robot.urdf').read_text());self.saved_pose=None;self.edit_goal=None
 def capture(self,mode):
  q=read_query('goal');self.validate(q);self.end=q
  if mode=='state':g={'type':'joint_state','q':q.tolist()}
  else:
   p,r=self.fk.pose(q);g={'type':'link_pose','link':'Link6','frame':'world','position_m':p.tolist(),'quaternion_xyzw':r.as_quat().tolist()}
  self.edit_goal=copy.deepcopy(g);return g
 def show(self):
  print('\n任务模式：{}；参考坐标系：world；目标连杆：Link6'.format('单目标' if len(self.goals)<=1 else '多目标'))
  print('已保存起点：');show_q(self.start);print('当前编辑目标缓存：')
  if self.edit_goal is None:print('  尚未捕获')
  else:show_goal(self.edit_goal)
  print('已保存任务目标：{} 个'.format(len(self.goals)))
  for i,g in enumerate(self.goals,1):print('  {:02d}. '.format(i),end='');show_goal(g)
  print('正式规划支持任意正整数个同类构型或位姿目标；草稿可保存。',flush=True)
 def save_task(self,path=None,planning=False):
  c=copy.deepcopy(self.cfg);c['task']['start_q']=self.start.tolist();c['output']['root']=str((P/'v2/outputs').resolve());goals=copy.deepcopy(self.goals or ([self.edit_goal] if self.edit_goal else []))
  if planning:
   if not goals:raise ValueError('任务没有目标，请先添加目标')
   if len({g.get('type') for g in goals})!=1:raise ValueError('正式规划暂不支持混合目标类型')
   self.validate(self.start)
   for g in goals:
    if g.get('type')=='joint_state':self.validate(g['q'])
  if len(goals)==1:
   c['task']['mode']='single';c['task']['goal']=goals[0];c['task'].pop('targets',None)
  else:
   c['task']['mode']='multi';c['task']['targets']=[dict(g,id=str(i)) for i,g in enumerate(goals,1)];c['task'].pop('goal',None);c['task'].setdefault('dwell_s',1.0)
  if path is None:
   folder=P/'v2/configs/tasks';folder.mkdir(parents=True,exist_ok=True);kind='构型' if goals and all(g.get('type')=='joint_state' for g in goals) else ('位姿' if goals and all(g.get('type')=='link_pose' for g in goals) else '混合');category='单目标' if len(goals)==1 else '多目标-{}'.format(len(goals));path=folder/(datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'-'+kind+'-'+category+'.yaml')
  dest=Path(path).expanduser().resolve();dest.parent.mkdir(parents=True,exist_ok=True)
  with dest.open('x') as f:yaml.safe_dump(c,f,sort_keys=False,allow_unicode=True)
  print('任务已保存：',dest,flush=True);return dest
 def loop(self):
  helptext='命令：start | current | mode state|pose | add | list | delete N | replace N | move N M | clear | save [PATH] | load PATH | plan [PATH] | help | quit'
  print(helptext,flush=True)
  while not rospy.is_shutdown():
   try:
    x=input('任务> ').split()
    if not x:continue
    c=x[0]
    if c=='quit':break
    elif c=='help':print(helptext,'\n正式规划：任意正整数个同类型目标；支持构型或位姿。')
    elif c=='start':q=read_query('start');self.validate(q);self.start=q;print('已保存绿色编辑起点：');show_q(self.start)
    elif c=='current':q=self.current();self.validate(q);self.start=q;print('已保存 PlanningScene 当前状态（非 Isaac 实际状态）：');show_q(self.start)
    elif c=='mode':
     if x[1] not in ('state','pose'):raise ValueError('模式必须为 state 或 pose')
     self.mode=x[1];print('目标采集模式：',self.mode)
    elif c=='goal':
     if x[1] not in ('state','pose'):raise ValueError('模式必须为 state 或 pose')
     self.mode=x[1];g=self.capture(self.mode);print('已捕获当前橙色编辑目标（尚未添加）：');show_goal(g)
    elif c=='add':self.goals.append(self.capture(self.mode));print('已添加目标 {:02d}，共 {} 个。'.format(len(self.goals),len(self.goals)))
    elif c=='list':self.show()
    elif c in ('delete','replace','move'):
     i=int(x[1])-1
     if not 0<=i<len(self.goals):raise ValueError('目标编号无效')
     if c=='delete':self.goals.pop(i)
     elif c=='replace':self.goals[i]=self.capture(self.mode)
     else:
      j=int(x[2])-1
      if not 0<=j<len(self.goals):raise ValueError('目标编号无效')
      self.goals.insert(j,self.goals.pop(i))
     print('操作成功，剩余 {} 个目标。'.format(len(self.goals)))
    elif c=='clear':self.goals=[];print('已清空，剩余 0 个目标。')
    elif c=='load':
     self.cfg=yaml.safe_load(Path(x[1]).expanduser().read_text());task=self.cfg['task'];self.start=np.array(task['start_q'],float);g=task.get('goal') or {};self.goals=[{k:v for k,v in y.items() if k!='id'} for y in task.get('targets',[])] if task.get('mode')=='multi' else ([g] if g else []);self.edit_goal=None
     if self.goals and len({item.get('type') for item in self.goals})==1:self.mode='state' if self.goals[0].get('type')=='joint_state' else 'pose'
     print('已加载：{}，共 {} 个目标。'.format(x[1],len(self.goals)))
    elif c in ('save','plan'):
     dest=self.save_task(x[1] if len(x)>1 else None,planning=c=='plan')
     if c=='plan':
      print('正在规划；完整日志：',os.environ.get('V2_BACKGROUND_LOG','终端'),flush=True)
      with open(os.environ.get('V2_BACKGROUND_LOG','/tmp/v2_plan.log'),'a') as log:r=subprocess.run([str(P/'v2/scripts/plan_v2.sh'),'--config',str(dest)],cwd=str(P),env=dict(os.environ,V2_MASTER_URI=rospy.get_master().getUri()),stdout=log,stderr=subprocess.STDOUT)
      print('规划{}，退出码 {}；详情见后台日志。'.format('完成' if r.returncode==0 else '失败',r.returncode),flush=True)
    else:print('错误：未知命令；输入 help。')
   except (ValueError,IndexError,KeyError,AssertionError,OSError,rospy.ROSException) as exc:print('错误：{}'.format(exc),flush=True)
   except (EOFError,KeyboardInterrupt):break
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--config',default=str(P/'v2/configs/global.yaml'));a=p.parse_args();rospy.init_node('v2_native_task',anonymous=True);Native(a.config).loop()
