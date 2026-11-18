"""P/H/G staged V2 interface. T/J deliberately unavailable; no robot execution."""
import argparse
import datetime as dt
import io
import json
import os
import re
import sys
import tempfile
import time
import traceback
from dataclasses import asdict
from hashlib import sha256
from pathlib import Path
import numpy as np
import yaml
from ..core.contracts import NAMES, PathRecord, StateSchema, Quantity, Status, require
from ..core.limits import load_limits
from ..core.json_io import write_exclusive as write
from ..geometry.phase_segmenter import build_geometry, parameterize_geometry
from ..geometry.frozen import freeze, thaw
from ..timing.feasible_profile import sample, concatenate
from ..validation.numerical import validate, validate_junctions
from ..adapters.planning_scene_checker import PlanningSceneChecker
from .diagnose_g import diagnose
from .planning_request import build
from .runtime_v2 import PACKAGE, RUNTIME, V2, start as start_runtime

OUTPUTS = V2/'outputs'
CONFIG = V2/'configs/v2_workspace_single_goal.yaml'
BEIJING = dt.timezone(dt.timedelta(hours=8))
os.environ.setdefault('ROS_HOME',str(RUNTIME/'ros_home'))
HEADER = ['time_s']+[prefix+name for prefix in ('q_','v_','a_','jerk_') for name in NAMES]


def digest(path):return sha256(Path(path).read_bytes()).hexdigest()
def read(path):return json.loads(Path(path).read_text())
def clean_collision(result):
    return {key:value for key,value in result.items() if key not in
            ('scene_snapshot_hex','robot_description','robot_description_semantic')}


def new_dir(config,task,suffix=''):
    taskid=task['id'];planner=config['planner_id']
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,79}',taskid) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,79}',planner):
        raise ValueError('UNSAFE_TASK_OR_PLANNER_ID')
    OUTPUTS.mkdir(parents=True,exist_ok=True)
    base='V2-'+dt.datetime.now(BEIJING).strftime('%Y%m%d-%H%M%S')+'-'+taskid+'-'+planner+suffix
    for number in range(1,1000):
        dest=OUTPUTS/(base if number==1 else base+'-%02d'%number)
        try:dest.mkdir(exist_ok=False);return dest
        except FileExistsError:continue
    raise RuntimeError('OUTPUT_NAME_EXHAUSTED')


def select_input(value,required):
    if value is None:raise ValueError('INPUT_REQUIRED: --input DIR or latest')
    if value=='latest':
        if not OUTPUTS.is_dir():raise ValueError('NO_V2_RESULTS')
        # Most recent directory with the prerequisite file, not most recent failure.
        choices=sorted((p for p in OUTPUTS.iterdir() if p.is_dir() and p.name.startswith('V2-')
                        and (p/required).is_file()),key=lambda p:(p.stat().st_mtime_ns,p.name),reverse=True)
        if not choices:raise ValueError('NO_SUCCESSFUL_INPUT_FOR_'+required)
        print('LATEST_RESOLVED='+str(choices[0]),flush=True);return choices[0]
    path=Path(value).resolve(strict=True)
    if path.is_file():raise ValueError('INPUT_MUST_BE_DIRECTORY')
    if (path/'P'/required).is_file():path=path/'P'
    if path.name.startswith('G') and path.parent.name.startswith('V2-'):path=path.parent
    if not (path/required).is_file():raise ValueError('MISSING_REQUIRED_STAGE: '+str(path/required))
    return path


def context(config,expected=None):
    start_runtime(config)
    from .experiment import check_master
    if check_master(os.environ['ROS_MASTER_URI'])!='REACHABLE':raise RuntimeError('MASTER_NOT_REACHABLE')
    with tempfile.TemporaryDirectory(prefix='v2_scene_check_') as temp:
        from .ros_probe import probe
        result=probe(Path(temp),float(config.get('service_timeout_s',45)))
    if expected is not None:
        for key in ('scene_sha256','urdf_sha256','srdf_sha256'):
            if result[key]!=expected[key]:raise RuntimeError('SCENE_OR_MODEL_CHANGED: '+key)
    return result


def load_parent(root):
    parent=read(root/'parent_path.json');data=parent['path']
    path=PathRecord(data['trial_id'],data['planner_id'],data['parent_path_hash'],data['input_stage'],
                    StateSchema(**data['schema']),data['q'],data['actual_time_s'],
                    data['requested_start'],data['requested_goal'],data['metadata'])
    return parent,path


def limits_for(parent):
    conf=parent['config']
    research={k:Quantity(**v) for k,v in conf['research_limits'].items()}
    return load_limits(PACKAGE/'config/gazebo_combined_robot.urdf',PACKAGE/'config/joint_limits.yaml',
                       PACKAGE/'config/combined_robot.srdf',research,velocity_scaling=conf['velocity_scaling'],
                       acceleration_scaling=conf['acceleration_scaling'],backend_overrides=conf.get('backend_limits'),
                       profile=parent.get('experiment_limit_profile',conf.get('constraint_profile')))


def assert_sources(parent):
    sources=parent['sources']
    for name in ('map','urdf','srdf','joint_limits'):
        entry=sources[name]
        if digest(entry['path'])!=entry['sha256']:raise RuntimeError('SOURCE_CHANGED: '+name)


def error(root,stage,command,exc,config=None):
    dest=root/'error.log'
    lines=['stage='+stage,'command='+repr(command),'error='+repr(exc),
           'task='+str((config or {}).get('tasks','saved input')),
           'planner='+str((config or {}).get('planner_id','saved input')),
           'hint=Inspect stage, input, MoveIt code and /tmp/cr5_v2_runtime_<uid> logs; retry this stage without deleting successful parents.',
           '',traceback.format_exc()]
    dest.write_text('\n'.join(lines))


def planning(config,task,command,out=None,context_fn=None):
    root=new_dir(config,task) if out is None else Path(out)
    if out is not None:root.mkdir(parents=True,exist_ok=False)
    try:
        scene=(context_fn or context)(config)
        payload=build(config,task,PACKAGE)
        payload['metadata'].update(scene_sha256=scene['scene_sha256'],model_sha256=scene['urdf_sha256'])
        with tempfile.TemporaryDirectory(prefix='v2_plan_') as scratch:
            request=Path(scratch)/'request.json';write(request,payload)
            capture=Path(scratch)/'capture'
            done=__import__('subprocess').run([sys.executable,'-B','-m','v2.adapters.capture_plan',str(request),
                                              '--output-dir',str(capture)],cwd=str(PACKAGE),capture_output=True,text=True,
                                             timeout=float(config['allowed_planning_time_s'])+30)
            if done.returncode:
                detail=read(capture/'capture_result.json') if (capture/'capture_result.json').is_file() else {}
                raise RuntimeError('MOVEIT_CAPTURE_FAILED code=%s output=%s detail=%s'%(done.returncode,done.stdout+done.stderr,detail.get('error')))
            record=read(capture/'capture_result.json');raw=read(capture/'parent_path.json')
            from moveit_msgs.msg import MotionPlanResponse
            response=MotionPlanResponse();response.deserialize(bytes.fromhex(record['response_hex']))
            state=io.BytesIO();response.trajectory_start.serialize(state)
        metadata={k:v for k,v in raw['metadata'].items() if k not in ('raw_robot_trajectory_hex','request')}
        raw['metadata']=metadata
        sources={name:dict(path=str(file),sha256=digest(file)) for name,file in
                 [('map',(PACKAGE/config['static_octomap']).resolve()),('urdf',PACKAGE/'config/gazebo_combined_robot.urdf'),
                  ('srdf',PACKAGE/'config/combined_robot.srdf'),('joint_limits',PACKAGE/'config/joint_limits.yaml')]}
        parent=dict(version=1,created_beijing=dt.datetime.now(BEIJING).isoformat(),task_id=task['id'],
                    planner_id=config['planner_id'],path=raw,config=config,
                    request=payload['motion_plan_request'],start_robot_state_hex=state.getvalue().hex(),
                    planning=dict(status='PASS',reported_planning_time_s=record['reported_planning_time_s'],
                                  service_wall_s=record['front_wall_s'],error_code=record['error_code'],
                                  endpoint_checks=record.get('endpoint_checks')),
                    scene={k:scene[k] for k in ('scene_sha256','urdf_sha256','srdf_sha256','octomap_id','octomap_data_bytes')},
                    sources=sources)
        np.savetxt(root/'parent_path.csv',np.asarray(raw['q']),fmt='%.17g',delimiter=',',
                   header=','.join(NAMES),comments='')
        write(root/'parent_path.json',parent)
        print('P_PASS',root,raw['parent_path_hash'],flush=True)
        return root
    except Exception as exc:
        error(root,'P',command,exc,config);raise


def geometry(root,command):
    if (root/'geometry.json').exists():raise RuntimeError('H_ALREADY_SUCCESSFUL: immutable geometry')
    try:
        parent,path=load_parent(root);assert_sources(parent)
        limits=limits_for(parent)
        original=build_geometry(path,limits,parent['config'].get('hold_duration'))
        geo=freeze(path,original)
        if limits.metadata.get('constraint_profile'):
            geo['construction'] = dict(algorithm_version='legacy_hg_v1',
                h_build_s=float(original['geometry_s']),
                effective_limit_identity=limits.metadata['constraint_profile']['effective_limit_identity'])
        # Exact saved geometry, no random candidate choice on downstream G/T/J.
        reconstructed=thaw(geo)
        for source,restored in zip(original['phases'],reconstructed['phases']):
            for s in (np.linspace(0,1,11),):
                for name in ('q','qs','qss','qsss'):
                    if not np.array_equal(getattr(source.evaluate(s),name),getattr(restored.evaluate(s),name)):
                        raise RuntimeError('GEOMETRY_ROUNDTRIP_MISMATCH: '+name)
        write(root/'geometry.json',geo)
        (root/'error.log').unlink(missing_ok=True)
        print('H_PASS',root,flush=True)
    except Exception as exc:error(root,'H',command,exc);raise


def commit_g(folder,result,matrix,report):
    """Publish exactly four files only after the complete candidate is writable."""
    folder=Path(folder)
    if folder.exists() and sorted(p.name for p in folder.iterdir())!=['error.log']:
        raise RuntimeError('G_OUTPUT_EXISTS_WITH_UNEXPECTED_CONTENT')
    with tempfile.TemporaryDirectory(prefix='.v2_g_',dir=str(folder.parent)) as temporary:
        candidate=Path(temporary)/'G';candidate.mkdir()
        np.savetxt(candidate/'trajectory.csv',matrix,delimiter=',',fmt='%.17g',
                   header=','.join(HEADER),comments='')
        write(candidate/'result.json',result)
        write(candidate/'diagnostics.json',diagnose(candidate))
        (candidate/'report.md').write_text(report)
        if folder.exists():
            (folder/'error.log').unlink();folder.rmdir()
        candidate.rename(folder)


def g(root,variant,command):
    folder=root/('G' if variant is None else 'G-'+variant)
    if folder.exists() and (folder/'result.json').exists():raise RuntimeError('G_ALREADY_SUCCESSFUL: choose --variant')
    try:
        parent,path=load_parent(root);assert_sources(parent)
        frozen=read(root/'geometry.json')
        if frozen.get('contract_version')==2:
            raise RuntimeError('USE_PLAN_UNIFIED_V2_FOR_CONTRACT_2')
        if frozen['parent_path_hash']!=path.parent_path_hash:raise RuntimeError('H_PARENT_MISMATCH')
        scene=context(parent['config'],parent['scene'])
        out=parameterize_geometry(path,limits_for(parent),thaw(frozen),parent['config'].get('hold_duration'))
        import rospy
        if not rospy.core.is_initialized():rospy.init_node('v2_stage_g_scene_check',anonymous=True,disable_signals=True)
        collision=clean_collision(PlanningSceneChecker().check(out['phases'],out['output']))
        if collision['status']!='PASS' or collision.get('scene_sha256')!=scene['scene_sha256']:
            raise RuntimeError('G_COLLISION_NOT_PASS: '+str(collision.get('reason'))+
                               ' checked_states='+str(collision.get('checked_states'))+
                               ' first_failure='+repr((collision.get('failures') or [None])[0]))
        phases=[dict(id=p.phase_id,kind=p.phase_type.value,start=p.start.tolist(),end=p.end.tolist(),
                     duration_s=float(r.time_s[-1]-r.time_s[0]),samples=len(r.time_s),
                     validation=asdict(v)) for p,r,v in zip(out['phases'],out['records'],out['validation'])]
        result=dict(version=1,algorithm='G',variant=variant or 'default',status='PASS',
                    numerical_status='PASS',collision=collision,
                    parent_path_hash=path.parent_path_hash,geometry_sha256=digest(root/'geometry.json'),
                    duration_s=out['duration_s'],xy_length_m=out['xy_length_m'],
                    planning_time_s=parent['planning']['reported_planning_time_s'],
                    backend_s=out['backend_s'],planner_id=path.planner_id,
                    phases=phases,limits=asdict(limits_for(parent)),
                    time_law='10u^3-15u^4+6u^5 per phase; analytic derivatives')
        matrix=np.column_stack([out['output'][key] for key in ('time_s','q','velocity','acceleration','jerk')])
        report='# %s / %s / G\n\n规划器：%s；父路径：`%s`。\n\n参考执行时间：%.9f s；XY路长：%.9f m；后端计算：%.9f s。\n\n数值验收：PASS；当前场景离散碰撞验收：PASS（%d个状态）。\n\n'%(
             parent['task_id'],variant or 'default',path.planner_id,path.parent_path_hash,
             out['duration_s'],out['xy_length_m'],out['backend_s'],collision['checked_states'])
        commit_g(folder,result,matrix,report)
        (root/'error.log').unlink(missing_ok=True)
        print('G_PASS',folder,flush=True)
    except Exception as exc:
        folder.mkdir(exist_ok=True);error(folder,'G',command,exc);raise


def recover_profile_records(path,phases,phase_info,limits,hold_duration=None):
    """Recover exact phase-local durations lost to global timestamp subtraction."""
    from ..timing.feasible_profile import parameterize
    records=[];checks=[];offset=0.
    for info,phase in zip(phase_info,phases):
        record,check=parameterize(phase,path,limits,offset,hold_duration)
        if len(record.time_s)!=int(info['samples']):raise RuntimeError('RECOVERED_SAMPLE_COUNT_MISMATCH')
        records.append(record);checks.append(check);offset=record.time_s[-1]
    if any(x.numerical_status!=Status.PASS for x in checks):raise RuntimeError('RECOVERED_NUMERICAL_FAILURE')
    return concatenate(records)


def independent_validate(root,variant,command):
    folder=root/('G' if variant is None else 'G-'+variant)
    try:
        parent,path=load_parent(root);assert_sources(parent)
        result=read(folder/'result.json');frozen=read(root/'geometry.json')
        if frozen.get('contract_version')==2:
            raise RuntimeError('USE_PLAN_UNIFIED_V2_FOR_CONTRACT_2')
        if result['parent_path_hash']!=path.parent_path_hash or result['geometry_sha256']!=digest(root/'geometry.json'):
            raise RuntimeError('VALIDATION_INPUT_CHANGED')
        phases=thaw(frozen)['phases'];limits=limits_for(parent);records=[];checks=[];offset=0.
        for info,phase in zip(result['phases'],phases):
            record=sample(phase,path,float(info['duration_s']),int(info['samples']),offset)
            records.append(record);checks.append(validate(record,limits,phase.start,phase.end));offset=record.time_s[-1]
        seam=validate_junctions(records) if len(records)>1 else None
        if any(x.numerical_status!=Status.PASS for x in checks) or (seam and seam.numerical_status!=Status.PASS):
            raise RuntimeError('INDEPENDENT_NUMERICAL_FAILURE')
        merged=concatenate(records)
        expected=np.column_stack([merged[key] for key in ('time_s','q','velocity','acceleration','jerk')])
        saved=np.loadtxt(folder/'trajectory.csv',delimiter=',',skiprows=1)
        if saved.shape!=expected.shape or not np.allclose(saved,expected,atol=1e-10,rtol=0):
            # Legacy output stores each duration as (global end - global start),
            # which can lose a few ulps. On the opt-in profile only, recover the
            # original phase durations by the same deterministic scaling, then
            # require the unchanged strict full-array comparison and revalidate.
            # Existing legacy contracts retain their original failure behavior.
            if not limits.metadata.get('constraint_profile'):
                raise RuntimeError('CSV_NOT_EQUAL_TO_ANALYTIC_RECONSTRUCTION')
            merged=recover_profile_records(path,phases,result['phases'],limits,parent['config'].get('hold_duration'))
            expected=np.column_stack([merged[key] for key in ('time_s','q','velocity','acceleration','jerk')])
            if saved.shape!=expected.shape or not np.allclose(saved,expected,atol=1e-10,rtol=0):
                raise RuntimeError('CSV_NOT_EQUAL_TO_ANALYTIC_RECONSTRUCTION')
            print('VALIDATE_DURATION_RECOVERED: global-offset floating-point cancellation; strict 1e-10 unchanged',flush=True)
        context(parent['config'],parent['scene'])
        import rospy
        if not rospy.core.is_initialized():rospy.init_node('v2_stage_validate',anonymous=True,disable_signals=True)
        collision=clean_collision(PlanningSceneChecker().check(phases,merged))
        if collision['status']!='PASS' or collision.get('scene_sha256')!=result['collision']['scene_sha256']:
            raise RuntimeError('VALIDATE_COLLISION_NOT_PASS: '+str(collision.get('reason')))
        (root/'error.log').unlink(missing_ok=True)
        print('VALIDATE_PASS',folder,'states='+str(collision['checked_states']),flush=True)
    except Exception as exc:error(root,'validate',command,exc);raise


def main(argv=None):
    parser=argparse.ArgumentParser(description='P→H→G V2 reference-only stages; T/J not implemented')
    mode=parser.add_mutually_exclusive_group()
    mode.add_argument('--stage',choices=['planning','geometry','g','validate','all','t','j'])
    mode.add_argument('--until',choices=['p','h','r','g0','g1'],help='new P then TM H/G0/G1; does not execute robot')
    parser.add_argument('--config');parser.add_argument('--input',help='existing V2 directory or latest')
    parser.add_argument('--task',help='task ID if YAML has several tasks');parser.add_argument('--variant',help='G variant label')
    parser.add_argument('--from-r',help='saved signed R directory; run P0 or P1')
    parser.add_argument('--from-p',help='saved P directory or parent_path.json')
    parser.add_argument('--reconstruction',choices=['signed'],help='reconstruction algorithm for --from-p')
    parser.add_argument('--export-csv',help='completed compact YAML run directory')
    parser.add_argument('--csv-out',help='new CSV path outside the result directory')
    parser.add_argument('--out',help='new output directory, required with --from-r')
    parser.add_argument('--scene-snapshot',help='optional matching PlanningScene snapshot with --from-r')
    args=parser.parse_args(argv)
    if args.export_csv:
        if args.until or args.stage or args.config or not args.csv_out:parser.error('--export-csv requires --csv-out and no planning arguments')
        from .prp_from_r import display_csv
        root=Path(args.export_csv).resolve(strict=True);dest=Path(args.csv_out).resolve()
        if root==dest or root in dest.parents:parser.error('--csv-out must be outside the budgeted result directory')
        data=display_csv(root/'result/PRP/P1/recommended',root/'result/R',root/'result/PRP/P0',dest)
        print(json.dumps(data,indent=2));return 0
    if args.csv_out:parser.error('--csv-out requires --export-csv')
    if not args.stage and not args.until:
        if not args.config or args.input or args.task or args.variant or args.from_r or args.from_p or args.out or args.reconstruction or args.scene_snapshot:
            parser.error('use --config global.yaml or specify --stage/--until')
        from . import prp_global
        return prp_global.run(Path(args.config))
    if args.from_p:
        if args.until not in ('r','g0','g1') or args.reconstruction!='signed' or not args.out or args.from_r or args.input or args.variant or args.task or args.config:
            parser.error('--from-p requires --reconstruction signed, --until r/g0/g1 and --out; do not combine with other task inputs')
        from . import prp_from_p
        command=['--parent',args.from_p,'--out',args.out,'--until',args.until]
        if args.scene_snapshot:command+=['--scene-snapshot',args.scene_snapshot]
        return prp_from_p.main(command)
    if args.reconstruction:parser.error('--reconstruction requires --from-p')
    if args.from_r:
        if args.until not in ('g0','g1') or not args.out or args.input or args.variant or args.task or args.config:
            parser.error('--from-r requires --until g0/g1 and --out; do not combine with --input, --config, --task or --variant')
        from . import prp_from_r
        command=['--signed-r',args.from_r,'--out',args.out,'--until',args.until]
        if args.scene_snapshot:command+=['--scene-snapshot',args.scene_snapshot]
        return prp_from_r.main(command)
    if args.out or args.scene_snapshot:parser.error('--out and --scene-snapshot require --from-r or --from-p')
    if args.until=='r':parser.error('--until r requires --from-p')
    if args.config is None:
        args.config=str(V2/'configs/v2_workspace_single_goal_9d_va_only.yaml') if args.until else str(CONFIG)
    if args.until:
        if args.input or args.variant:parser.error('--until does not accept --input or --variant')
        config=yaml.safe_load(Path(args.config).read_text())
        tasks=[t for t in config['tasks'] if args.task is None or t['id']==args.task]
        if len(tasks)!=1:parser.error('--until requires exactly one task; select with --task')
        if args.until!='p':
            profile=config.get('constraint_profile',{})
            disabled={'xy_speed','xy_acceleration','tangential_acceleration','wheel_velocity','wheel_acceleration','spin_jerk'}
            enabled=profile.get('enabled',{})
            if profile.get('identity')!='V2_NINE_DOF_VA_ONLY' or profile.get('velocity')!=[.5]*9 or profile.get('acceleration')!=[1.]*9 or set(enabled)!=disabled or any(enabled.values()):
                parser.error('--until h/g0/g1 requires V2_NINE_DOF_VA_ONLY, velocity 0.5, acceleration 1.0 and disabled extra limits')
        if args.until=='g1':
            from . import retry_phg1
            root=retry_phg1.run(config,tasks[0],['plan_v2.sh']+(argv if argv is not None else sys.argv[1:]))
            return 0 if read(root/'summary.json')['status']=='PASS' else 1
        root=planning(config,tasks[0],['plan_v2.sh']+(argv if argv is not None else sys.argv[1:]))
        if args.until=='p':return 0
        from . import phg1_run
        return phg1_run.main(['--input',str(root),'--out',str(root)+'-PHG1','--until',args.until])
    if args.stage in ('t','j'):
        print('NOT_IMPLEMENTED: '+args.stage.upper()+' requires a frozen H/T interface and solver',file=sys.stderr);return 3
    if args.variant and not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,39}',args.variant):parser.error('invalid --variant')
    config=yaml.safe_load(Path(args.config).read_text());command=['plan_v2.sh']+(argv if argv is not None else sys.argv[1:])
    if args.stage in ('planning','all'):
        tasks=[t for t in config['tasks'] if args.task is None or t['id']==args.task]
        if not tasks:parser.error('unknown task')
        if args.stage=='all' and args.input:parser.error('--input cannot be used with all')
        for task in tasks:
            root=planning(config,task,command)
            if args.stage=='all':geometry(root,command);g(root,args.variant,command);independent_validate(root,args.variant,command)
        return 0
    required={'geometry':'parent_path.json','g':'geometry.json','validate':'G/result.json'}[args.stage]
    root=select_input(args.input,required)
    if args.stage=='geometry':geometry(root,command)
    elif args.stage=='g':g(root,args.variant,command)
    else:independent_validate(root,args.variant,command)
    return 0

if __name__=='__main__':
    try:raise SystemExit(main())
    except Exception as exc:
        print('FAILED',repr(exc),file=sys.stderr);raise SystemExit(2)
