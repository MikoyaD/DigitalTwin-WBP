#!/usr/bin/env python3
"""Execute a formal P1 task and archive its measured states and evaluation."""
import argparse,csv,json,os,subprocess,sys,tempfile,traceback
from datetime import datetime
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2]
DEFAULT=ROOT/'v2/outputs/V2-20261124-014348-SINGLE-IKPRP-RRTstar'
p=argparse.ArgumentParser()
p.add_argument('--task',default=str(DEFAULT))
p.add_argument('--trajectory',help='CSV within the selected task; defaults to P1 recommended')
p.add_argument('--stage',default=str(ROOT/'v2/assets/isaac_import_20261124/warehouse_color_overlay.usda'))
p.add_argument('--physics-hz',type=int,default=120)
p.add_argument('--gui',action='store_true')
a=p.parse_args()
task=Path(a.task).expanduser().resolve(strict=True)
trajectory=Path(a.trajectory).expanduser().resolve(strict=True) if a.trajectory else task/'Parameterization/P1/recommended/trajectory.csv'
if task not in trajectory.parents:raise SystemExit('TRAJECTORY_OUTSIDE_TASK')
stage=Path(a.stage).expanduser().resolve(strict=True)
if not trajectory.is_file():raise SystemExit('P1_RECOMMENDED_NOT_FOUND: '+str(trajectory))
if a.physics_hz<=0:raise SystemExit('INVALID_PHYSICS_HZ')
with trajectory.open(newline='') as f:
 rows=list(csv.DictReader(f));t0=float(rows[0]['time_s']);t1=float(rows[-1]['time_s'])
if t1<=t0:raise SystemExit('INVALID_TRAJECTORY_TIME')
root=task/'Execution';root.mkdir(exist_ok=True)
label='P1-recommended' if trajectory.parent.name=='recommended' else 'MULTI'
name=datetime.now().strftime('%Y%m%d_%H%M%S')+'-'+label+'-%dHz'%a.physics_hz
index=0
while True:
 out=root/(name+('-%d'%index if index else ''))
 try:out.mkdir();break
 except FileExistsError:index+=1
meta={'planning_task_id':task.name,'reference_trajectory':str(trajectory.relative_to(task)),'physics_hz':a.physics_hz,'physics_dt_s':1/a.physics_hz,'control_update_hz':a.physics_hz,'derived_policy':'on demand: evaluate_execution --save-derived','solver':'PGS','trajectory_time_range_s':[t0,t1],'status':'STARTED','stage':str(stage),'joint_order':['joint%d'%i for i in range(1,7)],'root_initial_z_m':0.034224,'control':'DifferentialController wheel velocity; ArticulationController position drives','wheel_radius_m':0.06,'wheel_track_m':0.395,'joint2_stiffness':2000,'joint_damping':50,'contact':'temporary ground plane, wheel cylinders and low-friction support','limitations':'Experimental contact model; warehouse visual geometry lacks full collision validation; no hardware safety certification.'}
(out/'execution_metadata.json').write_text(json.dumps(meta,indent=2)+'\n')
lib=Path('/tmp/v2_isaac_lib');lib.mkdir(exist_ok=True);link=lib/'libcuda.so'
if not link.exists():link.symlink_to('/usr/lib/x86_64-linux-gnu/libcuda.so.1')
env={'HOME':os.environ['HOME'],'USER':os.environ.get('USER','robot'),'LOGNAME':os.environ.get('LOGNAME','robot'),'PATH':'/usr/local/bin:/usr/bin:/bin','DISPLAY':os.environ.get('DISPLAY',''),'XAUTHORITY':os.environ.get('XAUTHORITY',str(Path.home()/'.Xauthority')),'LD_LIBRARY_PATH':str(lib),'PYTHONPATH':str(ROOT),'PYTHONUNBUFFERED':'1','OMP_NUM_THREADS':'4','OPENBLAS_NUM_THREADS':'1'}
cmd=['/home/robot/isaacsim/python.sh',str(ROOT/'v2/scripts/isaac_p1_physical_tracking.py'),'--mode','feedback','--physics-hz',str(a.physics_hz),'--start',str(t0),'--duration',str(t1-t0),'--trajectory',str(trajectory),'--stage',str(stage),'--output',str(out)]
if a.gui:cmd.extend(['--gui','--hold-open'])
print('EXECUTION_OUTPUT',out,flush=True)
code=1
with (out/'execution.log').open('w') as log:
 log.write('COMMAND '+repr(cmd)+'\n');log.flush()
 try:
  proc=subprocess.Popen(cmd,cwd=str(ROOT),env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,bufsize=1)
  for line in proc.stdout:
   log.write(line);log.flush()
   if line.startswith(('PROGRESS','TRACKING_RESULT','Traceback','ERROR')):print(line,end='',flush=True)
  code=proc.wait()
  worker=json.loads((out/'metrics.json').read_text())
  meta.update(status=worker.get('status','FAILED'),actual_sim_duration_s=worker.get('sim_duration_s'),updates=worker.get('updates'),actual_trajectory_time_range_s=worker.get('window_s'))
  if code or meta['status']!='COMPLETED':raise RuntimeError('PHYSICAL_EXECUTION_FAILED')
  from v2.scripts.evaluate_execution import evaluate
  evaluate(task,out,replace_metrics=True)
  meta['status']='COMPLETED_AND_EVALUATED'
 except Exception as exc:
  meta['status']='FAILED';meta['error']=repr(exc);traceback.print_exc(file=log);traceback.print_exc();code=1
 finally:(out/'execution_metadata.json').write_text(json.dumps(meta,indent=2)+'\n')
print('EXECUTION_EXIT',code,'OUTPUT',out,flush=True)
sys.exit(code)
