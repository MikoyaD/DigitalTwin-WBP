#!/usr/bin/env python3
"""Small launcher for existing V2 entry points; no planning/control logic."""
import argparse,datetime,json,os,shutil,subprocess,sys,time
from pathlib import Path
import tkinter as tk
from tkinter import ttk,filedialog,messagebox,simpledialog
ROOT=Path(__file__).resolve().parents[2]
SCRIPTS=ROOT/'v2/scripts'
LOGS=Path.home()/'.ros/v2_launcher'
URI='http://127.0.0.1:11411'
PLATFORM_TITLE='V2-PRP 平台'
MANUAL=Path('/home/robot/digitaltwin/使用手册.md')
REPORT=Path('/home/robot/digitaltwin/项目报告.md')
ACTIONS={
 'environment':('启动 V2 环境',[str(SCRIPTS/'start_v2.sh')],False,'已有环境会由正式入口检查；请阅读终端结果。'),
 'editor':('原生任务编辑',[str(SCRIPTS/'interactive_task.sh')],True,'终端输入任务命令；RViz 由原入口启动。'),
 'view':('轨迹展示菜单',[str(SCRIPTS/'view_v2.sh')],False,'在终端选择 1 RViz、2 Gazebo 或 3 Isaac。'),
 'compare':('多轨迹对比',[str(SCRIPTS/'compare_v2.sh')],True,'终端选择轨迹；RViz 中按话题添加显示。'),
 'plan':('任务 YAML 规划',[str(SCRIPTS/'plan_v2.sh'),'--config'],True,'规划日志写入本次日志文件；输出保存在正式 outputs。'),
 'sequential':('顺序规划基线',[str(SCRIPTS/'sequential_v2.sh')],True,'使用所选父路径和场景；失败结果如实保留。'),
 'plot':('轨迹绘图',[str(SCRIPTS/'v2_python.sh'),'-m','v2.scripts.plot_trajectory'],False,'绘图仅按现有工具规则输出，不修改参考轨迹。'),
 'evaluate':('执行评价',[str(SCRIPTS/'v2_python.sh'),'-m','v2.scripts.evaluate_execution'],False,'使用所选任务及 Execution 记录；默认不覆盖已有指标。'),
}
# UI labels and their placement are kept here so users can rename/reorder the
# launcher without changing action IDs, scripts, arguments, or ROS behavior.
DISPLAY_GROUPS=(
 ('运行环境',(
  ('启动环境','environment'),
  ('环境状态','status'),
 )),
 ('任务规划',(
  ('任务点编辑','editor'),
  ('PRP任务规划','plan'),
  ('分阶段规划','sequential'),
 )),
 ('轨迹可视化',(
  ('RViz轨迹回放','rviz'),
  ('轨迹对比','compare'),
 )),
 ('数字孪生',(
  ('Gazebo回放','gazebo'),
  ('Isaac Sim回放/执行','isaac'),
 )),
 ('数据分析',(
  ('轨迹曲线绘制','plot'),
  ('执行误差分析','evaluate'),
 )),
 ('项目资料',(
  ('使用手册','manual'),
  ('项目报告','report'),
  ('任务配置','tasks'),
  ('实验结果','outputs'),
 )),
)
def runtime_status():
 try:
  p=subprocess.run([str(SCRIPTS/'status_v2.sh')],cwd=str(ROOT),capture_output=True,text=True,timeout=8)
  if p.returncode: return {'status':'CHECK_FAILED','detail':p.stderr.strip() or p.stdout.strip()}
  lines=[x for x in p.stdout.splitlines() if x.strip()];return json.loads(lines[-1])
 except (OSError,ValueError,subprocess.TimeoutExpired) as exc:return {'status':'CHECK_FAILED','detail':str(exc)}
def build_command(action,extra=()):
 if action not in ACTIONS:raise ValueError('未知功能')
 label,base,needs,hint=ACTIONS[action]
 for p in base:
  if p.startswith('/') and not Path(p).is_file():raise FileNotFoundError(p)
 return label,list(base)+list(map(str,extra)),needs,hint
def terminal_env():
 # Avoid Anaconda/Qt and shell overlays corrupting the GNOME terminal client.
 names=('HOME','USER','LOGNAME','DISPLAY','XAUTHORITY','DBUS_SESSION_BUS_ADDRESS','XDG_RUNTIME_DIR','LANG','LC_ALL','V2_EXTRA_SETUP','V2_LOG_DIR','V2_NO_RVIZ','ROS_HOME')
 env={name:os.environ[name] for name in names if name in os.environ};env['PATH']='/usr/local/bin:/usr/bin:/bin';env['V2_MASTER_URI']=URI;return env
def terminal_argv(label,command,log):
 term=shutil.which('gnome-terminal')
 if not term:raise RuntimeError('未找到 gnome-terminal，请安装或使用现有命令行入口。')
 # Arguments are positional, not interpolated shell text; command retains terminal stdin.
 script='log=$1; shift; set -o pipefail; "$@" 2>&1 | tee -a "$log"; rc=${PIPESTATUS[0]}; printf "\\n退出码：%s；日志：%s\\n按 Enter 关闭终端。\\n" "$rc" "$log"; read -r; exit "$rc"'
 return [term,'--title=V2 '+label,'--','bash','-c',script,'v2-terminal',str(log)]+command
class Launcher:
 def __init__(self,root):
  self.root=root;root.title(PLATFORM_TITLE);root.geometry('830x570');self.status=tk.StringVar(value='正在检查环境…');self.message=tk.StringVar(value='选择功能；交互工具将在独立终端运行。')
  ttk.Label(root,text=PLATFORM_TITLE,font=('Sans',17,'bold')).pack(pady=8)
  bar=ttk.Frame(root);bar.pack(fill='x',padx=12);ttk.Label(bar,textvariable=self.status).pack(side='left');ttk.Button(bar,text='刷新环境状态',command=self.refresh).pack(side='right')
  book=ttk.Notebook(root);book.pack(fill='both',expand=True,padx=12,pady=8)
  callbacks={
   'environment':lambda:self.launch('environment'),
   'status':self.show_status,
   'editor':lambda:self.launch('editor'),
   'plan':self.plan,
   'sequential':self.sequential,
   # These controls intentionally preserve the existing unified menu. The
   # official view_v2.sh owns the actual RViz/Gazebo/Isaac choice and setup.
   'rviz':lambda:self.launch('view'),
   'gazebo':lambda:self.launch('view'),
   'isaac':lambda:self.launch('view'),
   'compare':lambda:self.launch('compare'),
   'plot':self.plot,
   'evaluate':self.evaluate,
   'manual':lambda:self.open_path(MANUAL),
   'report':lambda:self.open_path(REPORT),
   'tasks':lambda:self.open_path(ROOT/'v2/configs/tasks'),
   'outputs':lambda:self.open_path(ROOT/'v2/outputs'),
  }
  for title,items in DISPLAY_GROUPS:
   frame=ttk.Frame(book,padding=18);book.add(frame,text=title)
   for text,action_id in items:ttk.Button(frame,text=text,command=callbacks[action_id]).pack(anchor='w',fill='x',pady=5)
  ttk.Label(root,textvariable=self.message,wraplength=790,justify='left').pack(fill='x',padx=15,pady=8);self.refresh()
 def refresh(self):
  d=runtime_status();self.state=d['status'];labels={'READY':'ROS Master / MoveIt / 场景：就绪','STOPPED':'ROS Master 未运行：请在环境管理中启动','PARTIAL':'环境部分就绪：请检查正式状态','INCOMPATIBLE_UNOWNED_MASTER':'端口被非 V2 Master 占用，未自动接管','BROKEN_OR_FOREIGN_MASTER':'环境异常或 Master 不匹配','CHECK_FAILED':'状态检查失败'};self.status.set(labels.get(self.state,self.state)+'  ('+d.get('uri',URI)+')');return d
 def show_status(self):
  d=self.refresh();messagebox.showinfo('V2 环境状态',json.dumps(d,ensure_ascii=False,indent=2))
 def launch(self,action,extra=()):
  try:
   label,cmd,needs,hint=build_command(action,extra)
   if needs and self.refresh().get('status')!='READY':messagebox.showwarning('环境未就绪','该功能需要 V2 ROS Master、MoveIt 和场景。请先使用“环境管理 → 启动 ROS / MoveIt 环境”。');return
   LOGS.mkdir(parents=True,exist_ok=True);log=LOGS/(datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'-'+action+'.log')
   argv=terminal_argv(label,cmd,log);env=terminal_env()
   proc=subprocess.Popen(argv,cwd=str(ROOT),env=env);time.sleep(.3)
   if proc.poll() not in (None,0):raise RuntimeError('终端启动失败，退出码 {}'.format(proc.returncode))
   self.message.set('{}：已请求打开终端（PID {}）。{}\n日志：{}'.format(label,proc.pid,hint,log))
  except (OSError,ValueError,RuntimeError) as exc:self.message.set('启动失败：'+str(exc));messagebox.showerror('启动失败',str(exc))
 def plan(self):
  p=filedialog.askopenfilename(title='选择任务 YAML',initialdir=str(ROOT/'v2/configs'),filetypes=[('YAML','*.yaml *.yml'),('所有文件','*')]);
  if p:self.launch('plan',[str(Path(p).resolve())])
 def sequential(self):
  parent=filedialog.askopenfilename(title='选择 ParentPath/parent_path.json',initialdir=str(ROOT/'v2/outputs'),filetypes=[('JSON','*.json')]);
  if not parent:return
  scene=filedialog.askopenfilename(title='选择对应 scene_snapshot.bin',initialdir=str(ROOT/'v2/outputs'),filetypes=[('场景快照','*.bin')]);
  if not scene:return
  out=ROOT/'v2/outputs'/('SEQUENTIAL-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f'))
  self.launch('sequential',['--parent',str(Path(parent).resolve()),'--scene-snapshot',str(Path(scene).resolve()),'--out',str(out)])
 def plot(self):
  p=filedialog.askopenfilename(title='选择轨迹 CSV',initialdir=str(ROOT/'v2/outputs'),filetypes=[('CSV','*.csv')]);
  if p:self.launch('plot',[str(Path(p).resolve())])
 def evaluate(self):
  task=filedialog.askdirectory(title='选择规划任务目录',initialdir=str(ROOT/'v2/outputs'))
  if not task:return
  out=filedialog.askdirectory(title='选择该任务下的 Execution 记录',initialdir=str(Path(task)/'Execution'))
  if not out:return
  if not (Path(out)/'execution_metadata.json').is_file() or not (Path(out)/'states.csv').is_file():messagebox.showerror('缺少数据','执行目录必须包含 execution_metadata.json 和 states.csv。');return
  self.launch('evaluate',[str(Path(task).resolve()),str(Path(out).resolve())])
 def open_path(self,path):
  try:
   if path==ROOT/'v2/configs/tasks':path.mkdir(parents=True,exist_ok=True)
   if not path.exists():raise FileNotFoundError(str(path))
   proc=subprocess.Popen(['xdg-open',str(path)],cwd=str(ROOT));self.message.set('已请求打开 {}（PID {}）'.format(path,proc.pid))
  except OSError as exc:messagebox.showerror('打开失败',str(exc))
def self_test():
 print('ROOT',ROOT,'DISPLAY',os.environ.get('DISPLAY',''),'STATUS',json.dumps(runtime_status(),ensure_ascii=False))
 expected=('运行环境','任务规划','轨迹可视化','数字孪生','数据分析','项目资料')
 titles=tuple(title for title,_ in DISPLAY_GROUPS)
 if titles!=expected:raise AssertionError('功能目录顺序或名称不匹配: {}'.format(titles))
 action_ids={action_id for _,items in DISPLAY_GROUPS for _,action_id in items}
 expected_ids={'environment','status','editor','plan','sequential','rviz','compare','gazebo','isaac','plot','evaluate','manual','report','tasks','outputs'}
 if action_ids!=expected_ids:raise AssertionError('功能按钮映射缺失或多余: {}'.format(sorted(action_ids^expected_ids)))
 for title,items in DISPLAY_GROUPS:
  print('GROUP',title,'BUTTONS',json.dumps([{'label':label,'action_id':action_id} for label,action_id in items],ensure_ascii=False))
 for action in ACTIONS:
  label,cmd,needs,hint=build_command(action);print('ACTION',action,'EXISTS',Path(cmd[0]).is_file(),'NEEDS_RUNTIME',needs,'COMMAND',json.dumps(cmd,ensure_ascii=False));terminal_argv(label,cmd,'/tmp/v2-launcher-probe.log')
 for p in (MANUAL,REPORT,ROOT/'v2/configs/tasks',ROOT/'v2/outputs'):print('PATH',p,'EXISTS',p.exists())
 root=tk.Tk();app=Launcher(root);root.update();root.destroy();print('TK_START_PASS')
def main():
 p=argparse.ArgumentParser();p.add_argument('--self-test',action='store_true');args=p.parse_args()
 if args.self_test:self_test();return
 root=tk.Tk();Launcher(root);root.mainloop()
if __name__=='__main__':main()
