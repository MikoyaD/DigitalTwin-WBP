"""Staged, append-only V2 experiment supervisor. No robot execution calls."""
import argparse
import datetime as dt
import json
import os
import signal
import socket
import subprocess
import sys
import time
import uuid
import xmlrpc.client
from pathlib import Path
from hashlib import sha256
import yaml
from .planning_request import build as build_request

V2=Path(__file__).resolve().parents[1]
PACKAGE=V2.parent
RUNS=V2/'outputs'/'runs'
DEFAULT=V2/'configs'/'experiment_ii_b.yaml'
PROTECTED=tuple(PACKAGE/p for p in ('script/execute_1goal_qp.py','script/qp_optimizer.py',
              'config/gazebo_combined_robot.urdf','config/combined_robot.srdf','config/joint_limits.yaml',
              'launch/warehouse_test.launch','map/waypoints.json','map/warehouse_cropped.bt'))
ARTIFACTS=('commands.log','environment.json','launch.log','planning.log','optimization.log',
           'ros_nodes.txt','ros_services.txt','manifest.json','validation.json','result_summary.md')


def digest(p):return sha256(Path(p).read_bytes()).hexdigest()
def utc():return dt.datetime.now(dt.timezone.utc).isoformat()
def put(path,data):
    with Path(path).open('x') as f:json.dump(data,f,indent=2,ensure_ascii=False,default=str)
def read(path):return json.loads(Path(path).read_text())
def process_start(pid):
    try:return Path('/proc/%d/stat'%pid).read_text().split()[21]
    except (OSError,IndexError):return None

def check_master(uri):
    socket.setdefaulttimeout(2)
    try:
        code,_,_=xmlrpc.client.ServerProxy(uri).getSystemState('/v2_runner_probe')
        return 'REACHABLE' if code==1 else 'MASTER_ERROR_'+str(code)
    except PermissionError:return 'NETWORK_PERMISSION_DENIED'
    except ConnectionRefusedError:return 'NOT_RUNNING'
    except OSError as exc:return 'MASTER_OS_ERROR_'+type(exc).__name__+':'+str(exc)


def owner_alive(info):
    if not info or process_start(info['pid'])!=info['proc_start']:return False
    try:
        environment=Path('/proc/%d/environ'%info['pid']).read_bytes().split(b'\0')
    except OSError:return False
    return ('V2_RUN_ID='+info['run_id']).encode() in environment


class Run:
    def __init__(self,args):
        self.args=args;self.config_path=Path(args.config).resolve();self.config=yaml.safe_load(self.config_path.read_text())
        if self.config.get('version')!=1:raise ValueError('config version must be 1')
        if self.config.get('group_name')!='whole':raise ValueError('GROUP_MUST_BE_WHOLE')
        self.map_path=(PACKAGE/self.config['static_octomap']).resolve()
        if not self.map_path.is_file() or self.map_path.suffix!='.bt':raise ValueError('STATIC_BT_MAP_MISSING')
        from ..adapters.workspace_check import parse_workspace
        parse_workspace(self.config)
        extent=float(self.config.get('default_workspace_bounds',10.))
        if not __import__('math').isfinite(extent) or extent<=0:raise ValueError('INVALID_DEFAULT_WORKSPACE_BOUNDS')
        self.parent=None
        if args.input and Path(args.input).is_dir():self.parent=Path(args.input).resolve()
        if args.run_id and (RUNS/args.run_id).exists():self.parent=RUNS/args.run_id
        if self.parent:
            if not (self.parent/'manifest.json').is_file():raise ValueError('input is not a V2 run')
            prior=read(self.parent/'manifest.json')
            if prior['config_sha256']!=digest(self.config_path):raise ValueError('CONFIG_CHANGED: create a new config with explicit provenance')
            for name,old in prior['protected_hashes'].items():
                if digest(name)!=old:raise ValueError('MODEL_OR_SOURCE_CHANGED: '+name)
        self.id=args.run_id if args.run_id and not self.parent else ('run_'+dt.datetime.now(dt.timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+uuid.uuid4().hex[:8])
        if not self.id.replace('_','').replace('-','').isalnum() or '/' in self.id:raise ValueError('bad run-id')
        self.root=RUNS/self.id;self.root.mkdir(parents=True,exist_ok=False)
        for file in ARTIFACTS:
            (self.root/file).touch(exist_ok=False)
        self.manifest=dict(run_id=self.id,parent_run_id=None if not self.parent else self.parent.name,
                           created_utc=utc(),stage=args.stage,status='RUNNING',dry_run=args.dry_run,
                           config_path=str(self.config_path),config_sha256=digest(self.config_path),
                           protected_hashes={str(p):digest(p) for p in PROTECTED},
                           stages={},commands=[],artifacts={},failures=[])
        self.environment=dict(created_utc=utc(),cwd=os.getcwd(),python=sys.version,
                              ros_distro=os.environ.get('ROS_DISTRO'),ros_master_uri=os.environ.get('ROS_MASTER_URI'),
                              ros_package_path=os.environ.get('ROS_PACKAGE_PATH'),path=os.environ.get('PATH'),
                              hostname=socket.gethostname(),setup_order=['/opt/ros/noetic/setup.bash','/home/robot/ws_moveit/devel/setup.bash',
                              '/home/robot/digitaltwin/zzs_ws/devel/setup.bash'],
                              setup_results=os.environ.get('V2_ENV_SETUP_RESULTS'),
                              config_sha256=digest(self.config_path),source_hashes=self.manifest['protected_hashes'])
        put(self.root/'config_snapshot.json',self.config)
        self.manifest['static_map_sha256']=digest(self.map_path)
        self._flush()
    def _flush(self):
        (self.root/'environment.json').write_text(json.dumps(self.environment,indent=2,ensure_ascii=False))
        (self.root/'manifest.json').write_text(json.dumps(self.manifest,indent=2,ensure_ascii=False))
    def command(self,stage,command,log,timeout):
        start=utc();cmd=[str(x) for x in command]
        with (self.root/'commands.log').open('a') as f:f.write(json.dumps(dict(stage=stage,at=start,command=cmd,timeout_s=timeout))+'\n')
        if self.args.dry_run:
            print('DRY_RUN',stage,' '.join(cmd),flush=True)
            self.manifest['commands'].append(dict(stage=stage,command=cmd,dry_run=True));self._flush();return 0
        try:
            done=subprocess.run(cmd,cwd=str(PACKAGE),env=os.environ.copy(),stdout=subprocess.PIPE,stderr=subprocess.STDOUT,
                                universal_newlines=True,timeout=timeout,check=False)
            output=done.stdout;rc=done.returncode
        except subprocess.TimeoutExpired as exc:output=str(exc.stdout or '')+'\nTIMEOUT '+str(timeout);rc=124
        with (self.root/log).open('a') as f:f.write('\n['+start+'] '+repr(cmd)+'\n'+output+'\n[exit='+str(rc)+']\n')
        self.manifest['commands'].append(dict(stage=stage,command=cmd,exit_code=rc,at=start,log=log))
        self._flush()
        if rc:raise RuntimeError('%s command exit %d, see %s'%(stage,rc,log))
        return rc
    def spawn(self,name,cmd,log):
        with (self.root/'commands.log').open('a') as f:f.write(json.dumps(dict(stage=name,at=utc(),command=cmd,spawn=True))+'\n')
        stream=(self.root/log).open('a');p=subprocess.Popen(cmd,cwd=str(PACKAGE),env=dict(os.environ,V2_RUN_ID=self.id),
                                stdout=stream,stderr=subprocess.STDOUT,start_new_session=True);stream.close()
        for _ in range(25):
            start=process_start(p.pid)
            if start:break
            time.sleep(.02)
        info=dict(pid=p.pid,proc_start=start,command=cmd,run_id=self.id,log=log)
        self.manifest.setdefault('owned_processes',{})[name]=info;self._flush();return p
    def require_environment(self):
        if os.environ.get('ROS_DISTRO')!='noetic':raise RuntimeError('ROS_DISTRO_NOT_NOETIC')
        if os.environ.get('ROS_MASTER_URI')!=self.config['master_uri']:raise RuntimeError('MASTER_URI_CONFIG_MISMATCH')
        for name in ('python3','roscore','roslaunch'):
            self.command('env_check',['/usr/bin/which',name],'launch.log',4)
        self.command('dependency_check',['python3','-B','-c','import rospy, moveit_msgs, tf2_ros, cvxpy, scipy, numpy, yaml'],'launch.log',12)
    def wait_master(self,limit):
        deadline=time.monotonic()+limit
        while time.monotonic()<deadline:
            state=check_master(self.config['master_uri'])
            if state=='REACHABLE':return
            if state=='NETWORK_PERMISSION_DENIED':raise RuntimeError(state)
            time.sleep(.2)
        raise RuntimeError('ROS_MASTER_TIMEOUT: '+state)
    def launch_if_needed(self):
        state=check_master(self.config['master_uri'])
        if state=='NETWORK_PERMISSION_DENIED':raise RuntimeError(state)
        if state!='REACHABLE':
            if not self.args.headless:raise RuntimeError('ROS_MASTER_NOT_RUNNING; rerun with --headless')
            self.spawn('roscore',['roscore','-p',self.config['master_uri'].rsplit(':',1)[1]],'launch.log')
            self.wait_master(15)
        else:
            # Never silently adopt another user's master. Parent ownership may be reused read-only.
            holder=self.parent if self.parent else self.root
            owners=read(holder/'manifest.json').get('owned_processes',{}) if holder!=self.root else self.manifest.get('owned_processes',{})
            if not owners or not owner_alive(owners.get('roscore')):raise RuntimeError('UNOWNED_MASTER: choose a free V2_MASTER_URI')
        if self.args.headless:
            holder=self.parent if self.parent else self.root
            owners=read(holder/'manifest.json').get('owned_processes',{}) if holder!=self.root else self.manifest.get('owned_processes',{})
            if not owner_alive(owners.get('launch')):
                self.spawn('launch',['roslaunch',str(V2/'launch/v2_headless_test.launch'),'static_octomap:='+str(self.map_path),'default_workspace_bounds:='+str(self.config.get('default_workspace_bounds',10.))],'launch.log')
    def probe(self,name):
        folder=self.root/name;folder.mkdir(exist_ok=False)
        self.command(name,['python3','-B','-m','v2.scripts.ros_probe','--output',str(folder),'--timeout',str(self.config['service_timeout_s'])],
                     'launch.log',self.config['service_timeout_s']+20)
        return read(folder/'scene_probe.json')
    def preflight(self):
        self.require_environment()
        self.launch_if_needed()
        # Scene injection is permitted only on the dedicated master owned by this run/parent.
        holder=self.parent if self.parent else self.root
        owners=read(holder/'manifest.json').get('owned_processes',{}) if holder!=self.root else self.manifest.get('owned_processes',{})
        if not owner_alive(owners.get('roscore')):raise RuntimeError('UNOWNED_MASTER_SCENE_INJECTION_REFUSED')
        try:return self.probe('preflight')
        except Exception as first:
            if 'STATIC_OBSTACLE_MISSING' not in (self.root/'launch.log').read_text():raise
            self.command('scene_loader',['python3','-B','-m','v2.scripts.scene_loader','--timeout',str(self.config['service_timeout_s'])],
                         'launch.log',self.config['service_timeout_s']+20)
            # Failed first probe created an output dir but no snapshot; unique retry.
            return self.probe('preflight_recovered')
    def compatible_scene(self,previous=None):
        current=self.probe('scene_before_'+uuid.uuid4().hex[:6])
        if previous and current['scene_sha256']!=previous['scene_sha256']:raise RuntimeError('PARENT_SCENE_MISMATCH')
        return current
    def ancestry(self):
        holder=self.root
        visited=set()
        while holder not in visited:
            visited.add(holder)
            yield holder
            parent_id=read(holder/'manifest.json').get('parent_run_id')
            if not parent_id:break
            holder=RUNS/parent_id
    def parent_scene(self):
        for holder in self.ancestry():
            for name in ('preflight_recovered','preflight'):
                p=holder/name/'scene_probe.json'
                if p.is_file():return read(p)
        raise RuntimeError('PREFLIGHT_REQUIRED')
    def planning(self):
        self.require_environment();self.wait_master(3)
        scene=self.compatible_scene(self.parent_scene())
        results=[]
        for task in self.config['tasks']:
            taskdir=self.root/('planning_'+task['id']);taskdir.mkdir(exist_ok=False)
            payload=build_request(self.config,task,PACKAGE)
            payload['metadata'].update(scene_sha256=scene['scene_sha256'],model_sha256=scene['urdf_sha256'])
            put(taskdir/'request.json',payload)
            try:
                self.command('planning_'+task['id'],['python3','-B','-m','v2.adapters.capture_plan',str(taskdir/'request.json'),
                                                    '--output-dir',str(taskdir/'capture')],
                             'planning.log',self.config['allowed_planning_time_s']+20)
                capture=read(taskdir/'capture'/'capture_result.json')
                source=taskdir/'capture'/'g_input.json'
                results.append(dict(task=task['id'],status='PASS',parent_path_hash=capture['parent_path_hash'],
                                    source=str(source),source_sha256=digest(source)))
            except Exception as exc:
                results.append(dict(task=task['id'],status='FAIL',reason=str(exc)))
                self.manifest['failures'].append(dict(stage='planning',task=task['id'],reason=str(exc)));self._flush()
        put(self.root/'planning_index.json',results)
        if not any(x['status']=='PASS' for x in results):raise RuntimeError('NO_REAL_PARENT_PATHS')
        return results
    def source_index(self):
        for holder in self.ancestry():
            p=holder/'planning_index.json'
            if p.is_file():return read(p)
        raise RuntimeError('PLANNING_INDEX_REQUIRED')
    def g(self):
        self.require_environment();results=[]
        for entry in self.source_index():
            if entry['status']!='PASS':continue
            task=entry['task'];source=Path(entry['source']);gdir=self.root/('g_'+task)
            if not source.is_file():raise RuntimeError('FROZEN_PARENT_MISSING '+str(source))
            if digest(source)!=entry['source_sha256']:raise RuntimeError('FROZEN_PARENT_CHANGED '+str(source))
            try:
                # run_g returns 2 on UNKNOWN collision even without --check-scene; do not treat numerical G as full PASS.
                cmd=['python3','-B','-m','v2.adapters.run_g',str(source),'--output-dir',str(gdir)]
                start=utc()
                with (self.root/'commands.log').open('a') as f:f.write(json.dumps(dict(stage='g_'+task,at=start,command=cmd))+'\n')
                done=subprocess.run(cmd,cwd=str(PACKAGE),stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=120)
                with (self.root/'optimization.log').open('a') as f:f.write('\n'+start+' '+repr(cmd)+'\n'+done.stdout+'\nexit='+str(done.returncode)+'\n')
                self.manifest['commands'].append(dict(stage='g_'+task,command=cmd,exit_code=done.returncode,log='optimization.log'));self._flush()
                result=read(gdir/'result.json')
                if result.get('numerical_status')!='PASS':raise RuntimeError('G_NUMERICAL_FAILURE '+result.get('error',''))
                self.command('diagnostics_'+task,['python3','-B','-m','v2.scripts.diagnose_g',str(gdir)],'optimization.log',15)
                results.append(dict(task=task,status='G_NUMERICAL_PASS_COLLISION_UNKNOWN',g_dir=str(gdir),source=str(source),
                                    result_sha256=digest(gdir/'result.json'),csv_sha256=digest(gdir/'trajectory.csv')))
            except Exception as exc:
                results.append(dict(task=task,status='FAIL',reason=str(exc)))
                self.manifest['failures'].append(dict(stage='g',task=task,reason=str(exc)));self._flush()
        put(self.root/'g_index.json',results)
        if not any(x['status'].startswith('G_NUMERICAL_PASS') for x in results):raise RuntimeError('NO_G_NUMERICAL_PATHS')
        return results
    def g_index(self):
        for holder in self.ancestry():
            p=holder/'g_index.json'
            if p.is_file():return read(p)
        raise RuntimeError('G_INDEX_REQUIRED')
    def validate(self):
        self.require_environment();self.wait_master(3)
        expected=self.parent_scene();self.compatible_scene(expected)
        results=[]
        for entry in self.g_index():
            if not entry['status'].startswith('G_NUMERICAL_PASS'):continue
            task=entry['task'];dest=self.root/('validation_'+task+'.json')
            if digest(Path(entry['g_dir'])/'result.json')!=entry['result_sha256'] or digest(Path(entry['g_dir'])/'trajectory.csv')!=entry['csv_sha256']:
                raise RuntimeError('FROZEN_G_OUTPUT_CHANGED '+task)
            try:
                self.command('validate_'+task,['python3','-B','-m','v2.scripts.validate_g','--g-dir',entry['g_dir'],
                             '--source',entry['source'],'--output',str(dest),'--check-scene'],
                             'optimization.log',100)
                audit=read(dest)
                results.append(dict(task=task,status=audit['overall_status'],validation=str(dest)))
            except Exception as exc:
                audit=read(dest) if dest.is_file() else {}
                results.append(dict(task=task,status=audit.get('overall_status','FAIL'),reason=str(exc),validation=str(dest)))
                self.manifest['failures'].append(dict(stage='validate',task=task,reason=str(exc)));self._flush()
        put(self.root/'validation_index.json',results)
        (self.root/'validation.json').write_text(json.dumps(results,indent=2))
        if len([x for x in results if x['status']=='PASS'])<len(self.config['tasks']):raise RuntimeError('TWO_REAL_G_TRAJECTORIES_NOT_VERIFIED')
        return results
    def inventory(self):
        try:
            for command,file in [(['rosnode','list'],'ros_nodes.txt'),(['rosservice','list'],'ros_services.txt')]:
                done=subprocess.run(command,capture_output=True,text=True,timeout=5)
                (self.root/file).write_text(done.stdout+done.stderr+'\nexit='+str(done.returncode)+'\n')
        except Exception as exc:
            with (self.root/'ros_nodes.txt').open('a') as f:f.write(repr(exc)+'\n')
    def finish(self):
        self.inventory()
        changed=[p for p,h in self.manifest['protected_hashes'].items() if digest(p)!=h]
        self.manifest['protected_files_changed']=changed
        self.manifest['artifacts']={str(p.relative_to(self.root)):dict(sha256=digest(p),bytes=p.stat().st_size,created_utc=dt.datetime.fromtimestamp(p.stat().st_ctime,dt.timezone.utc).isoformat())
                                     for p in self.root.rglob('*') if p.is_file() and p.name not in ('manifest.json','result_summary.md')}
        if changed:self.manifest['status']='FAIL_PROTECTED_CHANGED'
        elif self.args.dry_run:self.manifest['status']='DRY_RUN'
        elif self.manifest['failures']:self.manifest['status']='PARTIAL'
        elif self.args.stage in ('all','validate'):
            index=self.root/'validation_index.json'
            self.manifest['status']='PASS' if index.is_file() and len([x for x in read(index) if x['status']=='PASS'])>=len(self.config['tasks']) else 'PARTIAL'
        else:self.manifest['status']='STAGE_COMPLETE'
        self.manifest['finished_utc']=utc();self._flush()
        (self.root/'result_summary.md').write_text('# '+self.id+'\n\nStatus: '+self.manifest['status']+'\n\n'+
            '\n'.join('- '+k+': '+str(v) for k,v in self.manifest['stages'].items())+'\n\nFailures: '+json.dumps(self.manifest['failures'],ensure_ascii=False)+'\n')
        self.manifest['artifacts']['result_summary.md']=dict(sha256=digest(self.root/'result_summary.md'),bytes=(self.root/'result_summary.md').stat().st_size,created_utc=utc())
        self._flush()


def stop(args):
    run=RUNS/args.run_id;manifest=read(run/'manifest.json');results=[]
    for name,info in manifest.get('owned_processes',{}).items():
        if not owner_alive(info):results.append(dict(name=name,status='NOT_OWNED_OR_ALREADY_EXITED'));continue
        try:os.killpg(os.getpgid(info['pid']),signal.SIGINT);results.append(dict(name=name,status='SIGINT_SENT',pid=info['pid']))
        except (ProcessLookupError,PermissionError) as exc:results.append(dict(name=name,status='ERROR',error=repr(exc)))
    out=run/('stop_'+uuid.uuid4().hex[:8]+'.json');put(out,dict(at=utc(),results=results));print(out)


def main():
    parser=argparse.ArgumentParser(description='V2 Noetic staged planning and G validation, never executes motion')
    parser.add_argument('action',nargs='?',choices=['stop'])
    parser.add_argument('--stage',choices=['preflight','planning','g','validate','all'],default='all')
    parser.add_argument('--config',default=str(DEFAULT));parser.add_argument('--run-id');parser.add_argument('--input');parser.add_argument('--headless',action='store_true');parser.add_argument('--dry-run',action='store_true')
    args=parser.parse_args()
    if args.action=='stop':
        if not args.run_id:parser.error('stop requires --run-id')
        stop(args);return 0
    run=Run(args);print('RUN_DIR='+str(run.root),flush=True)
    try:
        if args.dry_run:
            for stage,cmd in [('preflight',['roscore -p '+run.config['master_uri'].rsplit(':',1)[1],'roslaunch '+str(V2/'launch/v2_headless_test.launch'),'python3 -B -m v2.scripts.ros_probe ...']),
                              ('planning',['python3 -B -m v2.adapters.capture_plan <frozen request> --output-dir <run>/planning_<task>/capture']),
                              ('g',['python3 -B -m v2.adapters.run_g <frozen g_input.json> --output-dir <run>/g_<task>']),
                              ('validate',['python3 -B -m v2.scripts.validate_g --g-dir <run>/g_<task> --check-scene'])]:
                if args.stage in (stage,'all'):
                    for c in cmd:run.command(stage,c.split(' '),'commands.log',0)
            run.manifest['stages']['dry_run']='PASS'
            print('EXPECTED_ARTIFACTS',', '.join(ARTIFACTS)+', planning_*/capture/{capture_result.json,g_input.json,parent_path.json}, g_*/{trajectory.csv,phase_arrays.npz,result.json}, validation_*.json',flush=True)
        else:
            stages=['preflight','planning','g','validate'] if args.stage=='all' else [args.stage]
            for stage in stages:
                started=utc()
                try:
                    getattr(run,stage)();run.manifest['stages'][stage]=dict(status='PASS',at=started)
                except Exception as exc:
                    run.manifest['stages'][stage]=dict(status='FAIL',at=started,error=repr(exc))
                    run.manifest['failures'].append(dict(stage=stage,at=utc(),error=repr(exc)))
                    with (run.root/'commands.log').open('a') as f:f.write(json.dumps(dict(stage=stage,error=repr(exc),at=utc()))+'\n')
                    break
                run._flush()
    finally:run.finish()
    print('STATUS='+run.manifest['status'])
    return 0 if run.manifest['status'] in ('PASS','STAGE_COMPLETE','DRY_RUN') else 2

if __name__=='__main__':raise SystemExit(main())
