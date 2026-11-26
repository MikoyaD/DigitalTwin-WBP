#!/usr/bin/env python3
"""Group existing discoveries by task; independently publish latched tracks."""
import os,re,shlex,subprocess,sys,time,json
from pathlib import Path
import rospy
from visualization_msgs.msg import Marker,MarkerArray
from v2.scripts.overlay_menu import options
from v2.scripts.trajectory_overlay import read_source

class Manager:
 def __init__(self):self.groups={};self.active={};self.selected=None;self.next_id=1;self.rescan()
 def rescan(self):
  self.groups={}
  for label,kind,path in options():
   m=re.search(r'(V2-\d+-\d+-(?:SINGLE|MULTI)-[^/ ]+|SEQUENTIAL-[^/ ]+)',label)
   name=m.group(1) if m else label.split('/')[0];real=Path(path).resolve();default=(Path(__file__).resolve().parents[2]/'v2/outputs').resolve();origin='formal:'+str(default) if default in real.parents else 'external:'+str(next((a for a in real.parents if a.name=='outputs'),real.parent));key=name+' ['+origin+']';self.groups.setdefault(key,[]).append((label,kind,path))
  print('扫描到 {} 个任务。'.format(len(self.groups)),flush=True)
 def tasks(self):
  for i,name in enumerate(sorted(self.groups),1):
   path=self.groups[name][0][2];kind='多目标' if '-MULTI-' in name else ('分段' if name.startswith('SEQUENTIAL-') else '单目标');origin='正式输出' if '[formal:' in name else '历史/其他根';short=name.split(' [')[0];print('{:3d}. {} | {} | {} | {}'.format(i,short,kind,'RRTstar' if 'RRTstar' in short else '未知',origin))
 def use(self,n):
  names=sorted(self.groups);i=int(n);assert 1<=i<=len(names),'任务编号无效';self.selected=names[i-1];self.sources()
 def sources(self):
  if not self.selected:print('请先选择任务');return
  print('当前任务：',self.selected.split(' [')[0])
  for i,(label,kind,path) in enumerate(self.groups[self.selected],1):
   part=label.split('/')[-1];stage=('Isaac 执行' if kind=='execution' else 'P' if kind=='P' else 'R' if kind=='R' else 'P0' if label.endswith(' P0') else 'P1 fast' if label.endswith(' fast') else '分段轨迹' if 'sequential' in label else '完整多目标' if '/task ' in label else 'P1 recommended');segment=re.search(r'(segment_\d+/attempt_\d+)',label);print('  {:2d}. {} | {}'.format(i,stage,segment.group(1) if segment else part))
 def add(self,label,kind,path):
  read_source(kind,path);ident=self.next_id;self.next_id+=1;topic='/v2/overlay/track_%d'%ident
  cmd=[sys.executable,'-B','-m','v2.scripts.trajectory_overlay','--source','track%d:%s:%s'%(ident,kind,path),'--color-index',str(ident-1),'--topic',topic]
  root=next((p for p in Path(path).resolve().parents if p.name.startswith('V2-') and '-MULTI-' in p.name),None)
  if root and (root/'result.json').is_file():
   try:
    for goal in json.loads((root/'result.json').read_text()).get('goals',[]):
     if goal.get('frame')=='world' and len(goal.get('position_m',[]))==3:cmd+=['--goal',':'.join([str(goal['id'])]+[str(x) for x in goal['position_m']])]
   except (ValueError,KeyError,OSError):pass
  log=open(os.environ.get('V2_BACKGROUND_LOG','/tmp/v2_overlay.log'),'a');proc=subprocess.Popen(cmd,stdout=log,stderr=log);log.close()
  try:
   rospy.wait_for_message(topic,MarkerArray,timeout=45)
   if proc.poll() is not None:raise RuntimeError('轨迹发布进程提前退出')
  except (rospy.ROSException,RuntimeError) as exc:
   proc.terminate();proc.wait();raise ValueError('轨迹发布失败：{}；检查后台日志'.format(exc))
  self.active[ident]=(proc,topic,label,kind,path)
  colors=['红','绿','蓝','橙','紫','青'];print('已发布 T{:03d}：{}\n  来源：{}\n  颜色：{}；话题：{}\n  当前 {} 条；RViz 中 Add → By topic 选择该话题。'.format(ident,self.selected.split(' [')[0] if self.selected else '手动文件',label,colors[(ident-1)%6],topic,len(self.active)),flush=True)
 def listing(self):
  for ident,(proc,topic,label,kind,path) in self.active.items():print('T{:03d} | {} | {} | {}'.format(ident,label,topic,'运行中' if proc.poll() is None else '已退出'),flush=True)
  if not self.active:print('当前没有已发布轨迹。',flush=True)
 def remove(self,ident):
  proc,topic,*_=self.active.pop(ident);pub=rospy.Publisher(topic,MarkerArray,queue_size=1,latch=True);time.sleep(.25);msg=MarkerArray();m=Marker();m.action=Marker.DELETEALL;msg.markers.append(m);pub.publish(msg);time.sleep(.2);proc.terminate()
  try:proc.wait(timeout=3)
  except subprocess.TimeoutExpired:proc.kill();proc.wait()
  pub.unregister();print('已移除 T{:03d}：{}'.format(ident,topic),flush=True)
 def loop(self):
  while not rospy.is_shutdown():
   print('\n1 添加轨迹  2 已发布轨迹  3 删除  4 清空  5 重新扫描  6 指定文件  7 退出（detail T001 查看路径）',flush=True)
   try:
    c=input('主菜单> ').strip()
    if c in ('7','quit','q'):break
    elif c=='1':
     while True:
      self.tasks();answer=input('任务编号（b 返回主菜单）> ').strip()
      if answer.lower()=='b':break
      try:self.use(answer)
      except (ValueError,AssertionError):print('错误：任务编号无效');continue
      while True:
       answer=input('来源编号（b 返回任务，q 返回主菜单）> ').strip().lower()
       if answer in ('b','q'):break
       try:
        n=int(answer);items=self.groups[self.selected]
        if not 1<=n<=len(items):raise ValueError('来源编号无效')
        self.add(*items[n-1])
       except (ValueError,IndexError,KeyError,OSError) as exc:print('错误：',exc,flush=True)
      if answer=='q':break
    elif c in ('2','list'):self.listing()
    elif c.startswith('detail '):
     ident=int(c.split()[1].lower().lstrip('t'));print('详细路径：',self.active[ident][4],flush=True)
    elif c=='3':
     self.listing();ident=input('删除已发布编号（如 T001；b 返回）> ').strip().lower()
     if ident!='b':self.remove(int(ident.lstrip('t')))
    elif c=='4':
     for n in list(self.active):self.remove(n)
     print('已清空。')
    elif c=='5':self.rescan()
    elif c=='6':
     kind=input('类型 P / R / trajectory / execution > ').strip();path=input('文件绝对路径 > ').strip()
     if kind not in ('P','R','trajectory','execution') or not Path(path).is_absolute():raise ValueError('类型或绝对路径无效')
     self.selected=None;self.add('手动文件',kind,path)
    else:print('错误：请输入有效菜单编号。')
   except (ValueError,IndexError,KeyError,AssertionError,OSError) as exc:print('错误：',exc,flush=True)
   except (EOFError,KeyboardInterrupt):break
  for n in list(self.active):self.remove(n)

def main():rospy.init_node('v2_dynamic_overlay',anonymous=True);Manager().loop()
if __name__=='__main__':main()
