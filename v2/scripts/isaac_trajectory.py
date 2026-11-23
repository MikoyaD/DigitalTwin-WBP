"""Read V2 P1 trajectories for future Isaac integration; never execute motion."""
import json
import contextlib
import io
from datetime import datetime
import os
import subprocess
import tempfile
from dataclasses import dataclass
from pathlib import Path

from ..adapters.trajectory_viewer import NAMES, REGISTRY, Session, read_csv, trajectory_kind
from .runtime_v2 import V2


@dataclass(frozen=True)
class IsaacTrajectory:
    source: str
    time_s: tuple
    q: tuple
    velocity: tuple
    acceleration: tuple
    direction: tuple
    joint_order: tuple
    frame: str
    map_config: dict


def map_config(usd=None):
    return {'name': 'warehouse',
            'gazebo_world': 'package://aws_robomaker_small_warehouse_world/worlds/no_roof_small_warehouse.world',
            'octomap': str(V2.parent / 'map/warehouse_cropped.bt'),
            'isaac_usd': str(Path(usd).expanduser().resolve()) if usd else None,
            'isaac_usd_status': 'user_selected_unverified' if usd else 'not_provided'}


def locate(source):
    path = Path(source).expanduser().resolve(strict=True)
    if path.is_dir():
        candidates = (path / 'result/PRP/P1/recommended/trajectory.csv',
                      path / 'PRP/P1/recommended/trajectory.csv',
                      path / 'trajectory.csv')
        path = next((p for p in candidates if p.is_file()), None)
        if path is None:
            raise ValueError('P1_RECOMMENDED_CSV_NOT_FOUND')
    if not path.is_file():
        raise ValueError('NOT_A_FILE')
    return path


def load_trajectory(source, usd=None):
    """Return original sample values; do not resample or launch a simulator."""
    path = locate(source)
    times, q, v, a, direction = read_csv(path, include_motion=True)
    return IsaacTrajectory(str(path), times, q, v, a, direction, tuple(NAMES), 'world', map_config(usd))


def summary(data):
    return {'status': 'PASS', 'source': data.source, 'samples': len(data.time_s),
            'time_start_s': data.time_s[0], 'time_end_s': data.time_s[-1],
            'max_interval_s': max(b-a for a, b in zip(data.time_s, data.time_s[1:])),
            'position_shape': [len(data.q), 9], 'velocity_shape': [len(data.velocity), 9],
            'acceleration_shape': [len(data.acceleration), 9], 'first_q': data.q[0],
            'last_q': data.q[-1], 'joint_order': data.joint_order,
            'direction_values': sorted(set(data.direction)), 'frame': data.frame,
            'map': data.map_config, 'scope': 'read only; no execution or new trajectory validation'}


def read_and_print(source, usd=None):
    try:
        data = load_trajectory(source, usd)
        print('ISAAC_TRAJECTORY_READ', json.dumps(summary(data), ensure_ascii=False), flush=True)
        return 0
    except (OSError, ValueError, TypeError, KeyError) as exc:
        print('ISAAC_TRAJECTORY_ERROR', str(exc), flush=True)
        return 1


def launch_playback(source, stage, gui=False, max_duration=None):
    source = str(locate(source))
    stage = str(Path(stage).expanduser().resolve(strict=True))
    evidence = Path('/home/robot/digitaltwin/research_management/evidence')
    prefix=('ISAAC_VISUAL_FIX_' if max_duration is not None else 'ISAAC_P1_PLAYBACK_')+datetime.now().strftime('%Y%m%d_%H%M%S')+'_'
    run = Path(tempfile.mkdtemp(prefix=prefix, dir=str(evidence)))
    libdir = Path(tempfile.gettempdir()) / 'v2_isaac_lib'
    libdir.mkdir(exist_ok=True)
    link = libdir / 'libcuda.so'
    if not link.exists():
        link.symlink_to('/usr/lib/x86_64-linux-gnu/libcuda.so.1')
    env = {'HOME':os.environ['HOME'], 'USER':os.environ.get('USER','robot'),
           'LOGNAME':os.environ.get('LOGNAME','robot'), 'PATH':'/usr/local/bin:/usr/bin:/bin',
           'DISPLAY':os.environ.get('DISPLAY',''), 'XAUTHORITY':os.environ.get('XAUTHORITY',str(Path.home()/'.Xauthority')),
           'LD_LIBRARY_PATH':str(libdir), 'PYTHONPATH':str(V2.parent), 'PYTHONUNBUFFERED':'1',
           'OMP_NUM_THREADS':'4', 'OPENBLAS_NUM_THREADS':'1'}
    command=['/home/robot/isaacsim/python.sh',str(V2/'scripts/isaac_playback.py'),
             '--trajectory',source,'--stage',stage,'--output',str(run)]
    if max_duration is not None:command.extend(['--max-duration',str(max_duration)])
    if gui:command.extend(['--gui','--hold-open','--wait-for-start'])
    print('ISAAC_OUTPUT',str(run),flush=True)
    with (run/'run.log').open('x') as log:
        log.write('COMMAND '+repr(command)+'\n');log.flush()
        process=subprocess.Popen(command,cwd=str(V2.parent),env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,bufsize=1)
        try:
            for line in process.stdout:
                print(line,end='',flush=True);log.write(line);log.flush()
            code=process.wait()
            result_path=run/'metrics.json'
            if code==0 and (not result_path.is_file() or json.loads(result_path.read_text()).get('status')!='PASS'):
                code=1
        except KeyboardInterrupt:
            process.terminate();process.wait();raise
    print('ISAAC_PLAYBACK_EXIT',code,'OUTPUT',str(run),flush=True)
    return code,run


def _final_p1_evidence(path):
    """Return selection and validation facts only with explicit PRP evidence."""
    path=Path(path)
    if path.parent.name=='task':
        root=path.parent.parent
        task=json.loads((path.parent/'result.json').read_text())
        result=json.loads((root/'result.json').read_text())
        segments=result.get('segments',[])
        if (result.get('status')!='SIGNED_FIVE_GOAL_TASK_PASS' or task.get('status')!='TASK_PASS'
            or len(segments)!=5 or any(x.get('status')!='PASS' for x in segments)
            or result.get('task',{}).get('numerical')!='PASS'
            or 'FAIL' in str(result.get('task',{}).get('collision','')).upper()
            or task.get('collision',{}).get('status')=='FAIL'
            or not task.get('source_parent')):return None
        first=Path(segments[0]['directory'])/'result/P/parent_path.json'
        if (path.parent/task['source_parent']).resolve()!=first.resolve():return None
        return '多目标完整','PASS',str(result['task'].get('collision','UNKNOWN'))
    if path.parent.name!='recommended' or path.parent.parent.name!='P1':return None
    run=path.parents[4]
    overall=json.loads((run/'result/result.json').read_text())
    numeric=json.loads((path.parent/'result.json').read_text())
    collision=json.loads((run/'result/PRP/collision.json').read_text())
    if (overall.get('status')!='FIXED_SIGNED_R_SINGLE_PASS' or numeric.get('status')!='PASS'
        or collision.get('status')=='FAIL'):return None
    kind='单目标'
    # An attempt belongs to a multi-task only if its parent task explicitly
    # selects this attempt; an orphan attempt is never a standalone result.
    if 'segments' in path.parts or any(x.startswith('attempt_') for x in path.parts):
        kind='多目标组分';selected=False
        for ancestor in run.parents:
            result_file=ancestor/'result.json'
            if not result_file.is_file():continue
            try:parent=json.loads(result_file.read_text())
            except (ValueError,OSError):continue
            segments=parent.get('segments')
            if not isinstance(segments,list):continue
            for segment in segments:
                directory=segment.get('directory')
                if directory and Path(directory).resolve()==run.resolve() and segment.get('status')=='PASS':
                    selected=True;break
            break
        if not selected:return None
    return kind,'PASS',str(collision.get('status','UNKNOWN'))


def scan_final_p1():
    """Reuse the V2 Session scanner and select only verified final P1 CSVs."""
    from .gazebo_interactive import GazeboSession
    with contextlib.redirect_stdout(io.StringIO()):
        session=Session(REGISTRY,lambda _:None,runs_dir=V2.joinpath('outputs/runs').resolve(),summary_only=True)
    found=[]
    for name,item in session.available.items():
        try:
            path=GazeboSession.csv_path(item)
            status=item.get('summary',{}).get('validation_status','UNVALIDATED') if isinstance(item,dict) else json.loads((item/'metadata.json').read_text()).get('validation_status','UNVALIDATED')
            if status not in ('VERIFIED','DENSE_SOURCE_VERIFIED_DISPLAY_ONLY','ARCHIVED_NUMERICAL_AND_DISCRETE_COLLISION_PASS','COLLISION_PENDING','COLLISION_NOT_ATTEMPTED','COLLISION_UNKNOWN'):
                continue
            evidence=_final_p1_evidence(path)
            if evidence is None:continue
            kind,numerical,collision=evidence
            # The Isaac reader requires q/v/a and signed direction; do not list
            # older preview formats that cannot be consumed by this worker.
            import csv
            with path.open(newline='') as stream:header=next(csv.reader(stream))
            required={'time_s','direction'} | {prefix+n for prefix in ('q_','v_','a_') for n in NAMES}
            if not required.issubset(header):continue
            duration=(item['summary'] if isinstance(item,dict) else json.loads((item/'metadata.json').read_text())).get('duration_s')
            if duration is None:
                data=load_trajectory(path);duration=data.time_s[-1]-data.time_s[0]
            found.append({'name':name,'kind':kind,'path':str(path),'duration_s':duration,'status':status,'selection':'FINAL','numerical':numerical,'collision':collision,'display_only':status=='DENSE_SOURCE_VERIFIED_DISPLAY_ONLY'})
        except (OSError,ValueError,KeyError,StopIteration):
            continue
    return found


def interactive_isaac():
    print('Isaac Sim：1 轨迹回放 | 2 轨迹执行 | q 返回',flush=True)
    while True:
        try:
            choice=input('isaac> ').strip().lower()
            if choice in ('q','quit'):return
            if choice=='2':
                print('轨迹执行功能待开发。',flush=True);continue
            if choice!='1':
                print('请选择 1、2 或 q。',flush=True);continue
            tracks=scan_final_p1()
            for i,item in enumerate(tracks,1):
                print('%d. %s | %s | %.6f s | 最终:%s 数值:%s 碰撞:%s 展示:%s | %s'%(i,item['name'],item['kind'],item['duration_s'],item['selection'],item['numerical'],item['collision'],item['status'],item['path']),flush=True)
            counts={kind:sum(x['kind']==kind for x in tracks) for kind in ('单目标','多目标组分','多目标完整')}
            print('最终 P1 数量：'+json.dumps(counts,ensure_ascii=False),flush=True)
            selected=input('选择编号；m 输入 CSV 绝对路径；q 返回 > ').strip()
            if selected.lower() in ('q','quit'):continue
            if selected.lower()=='m':
                selected=input('CSV 绝对路径 > ').strip()
                if not Path(selected).is_absolute():raise ValueError('请输入 CSV 绝对路径')
                name=Path(selected).name;kind='手动输入'
            elif selected.isdigit() and 1<=int(selected)<=len(tracks):
                item=tracks[int(selected)-1];selected=item['path'];name=item['name'];kind=item['kind']
            else:raise ValueError('无效轨迹选择')
            data=load_trajectory(selected)
            print('已选择：%s | %s | %s | %.6f s'%(name,kind,data.source,data.time_s[-1]-data.time_s[0]),flush=True)
            default_stage=str(V2/'assets/isaac_import_20261124/warehouse_v3.usd')
            stage_choice=input('Stage：1 默认仓储 | 2 手动输入 USD（默认 1）> ').strip() or '1'
            if stage_choice=='1':stage=default_stage
            elif stage_choice=='2':
                stage=input('Stage USD 绝对路径 > ').strip()
                if not Path(stage).is_absolute():raise ValueError('请输入 Stage 绝对路径')
            else:raise ValueError('无效 Stage 选择')
            value=input('回放秒数（留空完整轨迹）> ').strip();limit=float(value) if value else None
            if limit is not None and (not __import__('math').isfinite(limit) or limit<=0):raise ValueError('回放时长必须为正数')
            print('运动学直接置位；XY/yaw 单位变换，Z=0.034224 m 为临时视觉高度。',flush=True)
            launch_playback(data.source,stage,gui=True,max_duration=limit)
        except EOFError:return
        except (OSError,ValueError,KeyError) as exc:
            print('ISAAC_TRAJECTORY_ERROR',str(exc),flush=True)
