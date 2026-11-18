"""One V2-owned, run-independent Noetic environment; never adopts unknown ROS processes."""
import argparse
import json
import os
import signal
import socket
import subprocess
import sys
import tempfile
import time
import uuid
from hashlib import sha256
from pathlib import Path
from .experiment import check_master, process_start

V2 = Path(__file__).resolve().parents[1]
PACKAGE = V2.parent
RUNTIME = Path(tempfile.gettempdir()) / ('cr5_v2_runtime_' + str(os.getuid()))
URI = 'http://127.0.0.1:11411'


def _state():
    path = RUNTIME / 'state.json'
    return json.loads(path.read_text()) if path.is_file() else None


def _owned(item, token):
    if not item or process_start(item['pid']) != item['start']:return False
    try:env = Path('/proc/%d/environ' % item['pid']).read_bytes().split(b'\0')
    except OSError:return False
    return ('V2_RUNTIME_TOKEN=' + token).encode() in env


def _spawn(command, token, name):
    stream = (RUNTIME / (name + '.log')).open('a')
    process = subprocess.Popen(command, cwd=str(PACKAGE), env=dict(os.environ, V2_RUNTIME_TOKEN=token),
                               stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
    stream.close()
    for _ in range(20):
        start = process_start(process.pid)
        if start:return dict(pid=process.pid,start=start,command=command)
        time.sleep(.05)
    raise RuntimeError('PROCESS_START_FAILED: ' + name)


def _wait(condition, seconds, label):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if condition():return
        time.sleep(.3)
    raise RuntimeError(label + '_TIMEOUT; see ' + str(RUNTIME))


def _ready(timeout=8):
    if check_master(URI) != 'REACHABLE':return False
    try:
        import xmlrpc.client
        params = xmlrpc.client.ServerProxy(URI)
        for key in ('/robot_description','/robot_description_semantic','/move_group/planning_pipelines/ompl/default_workspace_bounds'):
            if not params.hasParam('/v2_runtime', key)[2]:return False
        import rosgraph
        master=rosgraph.Master('/v2_runtime')
        services = {name for name,_ in master.getSystemState()[2]}
        return all(s in services for s in ('/plan_kinematic_path','/get_planning_scene','/check_state_validity'))
    except Exception:return False


def status():
    state = _state()
    master = check_master(URI)
    if not state:
        return dict(status='STOPPED' if master != 'REACHABLE' else 'INCOMPATIBLE_UNOWNED_MASTER',uri=URI)
    if not _owned(state.get('roscore'),state['token']) or master != 'REACHABLE':
        return dict(status='BROKEN_OR_FOREIGN_MASTER',uri=URI)
    if not _owned(state.get('launch'),state['token']) or not _ready():
        return dict(status='PARTIAL',uri=URI)
    return dict(status='READY',uri=URI,map=state['map'],default_workspace_bounds=state['default_workspace_bounds'])


def start(config,for_display=False):
    os.environ['ROS_HOME']=str(RUNTIME/'ros_home')
    if os.environ.get('ROS_DISTRO') != 'noetic' or os.environ.get('ROS_MASTER_URI') != URI:
        raise RuntimeError('CLEAN_NOETIC_ENV_REQUIRED')
    map_path = (PACKAGE / config['static_octomap']).resolve(strict=True)
    if not map_path.is_file() or map_path.suffix != '.bt':raise ValueError('STATIC_BT_MAP_MISSING')
    model_map_hash = sha256(map_path.read_bytes()).hexdigest()
    extent = float(config.get('default_workspace_bounds',10.))
    old = _state();state = status()
    if old:
        if state['status'] != 'READY':raise RuntimeError('V2_RUNTIME_PARTIAL: run stop_v2.sh; no automatic takeover')
        if old['map'] != str(map_path) or old['map_sha256'] != model_map_hash or (not for_display and old['default_workspace_bounds'] != extent):
            raise RuntimeError('V2_RUNTIME_INCOMPATIBLE: stop_v2.sh before changing map/workspace fallback')
        print(json.dumps(state,ensure_ascii=False));return state
    if state['status'] != 'STOPPED':raise RuntimeError('V2_PORT_OCCUPIED_BY_UNKNOWN_MASTER: ' + URI)
    RUNTIME.mkdir(mode=0o700,parents=True,exist_ok=True)
    (RUNTIME/'ros_home').mkdir(mode=0o700,exist_ok=True)
    # ROS normally writes ~/.ros; keep these process logs out of the result tree
    # and avoid altering the user's private ROS directory.
    os.environ['ROS_HOME']=str(RUNTIME/'ros_home')
    token=uuid.uuid4().hex
    record=dict(token=token,uri=URI,map=str(map_path),map_sha256=model_map_hash,default_workspace_bounds=extent)
    try:
        record['roscore']=_spawn(['roscore','-p','11411'],token,'roscore')
        (RUNTIME/'state.json').write_text(json.dumps(record,indent=2))
        _wait(lambda:check_master(URI)=='REACHABLE',15,'ROS_MASTER')
        record['launch']=_spawn(['roslaunch',str(V2/'launch/v2_headless_test.launch'),
                                 'static_octomap:='+str(map_path),'default_workspace_bounds:='+str(extent)],token,'move_group')
        (RUNTIME/'state.json').write_text(json.dumps(record,indent=2))
        _wait(_ready,float(config.get('service_timeout_s',45))+20,'MOVEIT_SERVICES')
        with tempfile.TemporaryDirectory(prefix='v2_scene_') as scratch:
            def probe():
                return subprocess.run([sys.executable,'-B','-m','v2.scripts.ros_probe','--output',scratch,
                                       '--timeout','5'],cwd=str(PACKAGE),capture_output=True,text=True,timeout=20)
            result=probe()
            if result.returncode and 'STATIC_OBSTACLE_MISSING' in result.stderr:
                loaded=subprocess.run([sys.executable,'-B','-m','v2.scripts.scene_loader','--timeout','20'],
                                      cwd=str(PACKAGE),capture_output=True,text=True,timeout=30)
                if loaded.returncode:raise RuntimeError('SCENE_LOAD_FAILED: '+loaded.stderr)
                result=probe()
            if result.returncode:raise RuntimeError('SCENE_PROBE_FAILED: '+result.stderr)
            scene=json.loads((Path(scratch)/'scene_probe.json').read_text())
        record['scene_sha256']=scene['scene_sha256'];record['model_sha256']=scene['urdf_sha256']
        (RUNTIME/'state.json').write_text(json.dumps(record,indent=2))
        print(json.dumps(status(),ensure_ascii=False));return status()
    except Exception:
        # State remains so only stop_v2.sh can safely clean up owned processes.
        raise


def stop():
    record=_state()
    if not record:
        print('STOPPED; no owned V2 runtime');return
    for name in ('launch','roscore'):
        item=record.get(name)
        if _owned(item,record['token']):
            os.killpg(os.getpgid(item['pid']),signal.SIGINT)
            print('SIGINT_SENT',name,item['pid'])
    # Retain state if a process is still alive; never delete state while ownership is unresolved.
    _wait(lambda:all(not _owned(record.get(n),record['token']) for n in ('launch','roscore')),12,'OWNED_SHUTDOWN')
    (RUNTIME/'state.json').unlink()
    print('STOPPED',URI)


def main():
    import yaml
    parser=argparse.ArgumentParser();parser.add_argument('action',choices=['start','status','stop'])
    parser.add_argument('--config',default=str(V2/'configs/v2_workspace_single_goal.yaml'));args=parser.parse_args()
    if args.action=='status':print(json.dumps(status(),ensure_ascii=False))
    elif args.action=='stop':stop()
    else:start(yaml.safe_load(Path(args.config).read_text()))

if __name__=='__main__':main()
