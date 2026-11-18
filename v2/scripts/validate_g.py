"""Independently recheck saved G arrays and scene, without replanning or reoptimizing."""
import argparse
import json
from pathlib import Path
from hashlib import sha256
import numpy as np
from ..core.contracts import (NAMES,StateSchema,PhaseType,PhasePath,PathJet,TrajectoryRecord,Quantity,ContractError,Status)
from ..core.math_tools import QuinticSpline,arm_only_jet,spin_jet,hold_jet
from ..core.limits import load_limits
from ..geometry.spline_builder import LiftedSpline
from ..validation.numerical import validate,validate_junctions
from ..timing.feasible_profile import concatenate
from ..adapters.planning_scene_checker import PlanningSceneChecker


def reconstruct_phase(info,schema):
    start=np.asarray(info['start'],float);end=np.asarray(info['end'],float);kind=PhaseType(info['kind'])
    if kind==PhaseType.SPIN:
        ev=lambda s:spin_jet(s,start,float(end[2]-start[2]))
    elif kind==PhaseType.HOLD:ev=lambda s:hold_jet(start,len(s))
    else:
        spline=QuinticSpline(info['controls'],info['knots'])
        if kind==PhaseType.ARM_ONLY:
            ev=lambda s:arm_only_jet(start[:3],[spline.evaluate(s,k)[:,2:] for k in range(4)])
        else:
            lift=LiftedSpline(spline,float(start[2]))
            ev=lift
        ev.geometry_spline=spline
    return PhasePath(info['id'],kind,schema,start,end,ev,'reconstructed from frozen G result')


def audit(g_dir,source,with_scene):
    root=Path(__file__).resolve().parents[1];package=root.parent
    result=json.loads((g_dir/'result.json').read_text());data=json.loads(source.read_text())
    if result['source_input_sha256']!=sha256(source.read_bytes()).hexdigest():raise ContractError('INPUT_HASH_MISMATCH','frozen parent input changed')
    schema=StateSchema(**result['path']['schema']);path=result['path'];phases=[reconstruct_phase(p,schema) for p in result['phases']]
    limits=load_limits(package/'config/gazebo_combined_robot.urdf',package/'config/joint_limits.yaml',package/'config/combined_robot.srdf',
                       {k:Quantity(**v) for k,v in data['research_limits'].items()},
                       velocity_scaling=data['velocity_scaling'],acceleration_scaling=data['acceleration_scaling'],backend_overrides=data.get('backend_limits'))
    archive=np.load(str(g_dir/'phase_arrays.npz'),allow_pickle=False);records=[];out=[]
    for i,(info,phase) in enumerate(zip(result['phases'],phases)):
        d={key:archive['phase_%03d_%s'%(i,key)] for key in ('time_s','q','velocity','acceleration','jerk')}
        r=TrajectoryRecord(path['trial_id'],phase.phase_id,phase.phase_type,path['planner_id'],path['parent_path_hash'],schema,
                           d['time_s'],d['q'],d['velocity'],d['acceleration'],d['jerk'],'saved analytic arrays rechecked')
        records.append(r);out.append(validate(r,limits,phase.start,phase.end))
    junction=validate_junctions(records) if len(records)>1 else None
    merged=concatenate(records)
    csv=np.loadtxt(str(g_dir/'trajectory.csv'),delimiter=',',skiprows=1)
    expected=np.column_stack([merged[key] for key in ('time_s','q','velocity','acceleration','jerk')])
    if csv.shape!=expected.shape or not np.array_equal(csv,expected):raise ContractError('EXPORT_MISMATCH','CSV must roundtrip exact saved phase arrays')
    numerical=all(x.numerical_status==Status.PASS for x in out) and (junction is None or junction.numerical_status==Status.PASS)
    collision={'status':'UNKNOWN','reason':'live scene not requested'}
    if numerical and with_scene:
        import rospy
        rospy.init_node('v2_validate_saved_g',anonymous=True,disable_signals=True)
        collision=PlanningSceneChecker().check(phases,merged)
        expected_scene=data['metadata'].get('scene_sha256')
        if expected_scene and collision.get('scene_sha256')!=expected_scene:
            collision.update(status='UNKNOWN',reason='PARENT_SCENE_MISMATCH',expected_scene_sha256=expected_scene)
    return dict(numerical_status='PASS' if numerical else 'FAIL',collision=collision,
                overall_status='PASS' if numerical and collision['status']=='PASS' else ('FAIL' if not numerical or collision['status']=='FAIL' else 'UNKNOWN'),
                phase_checks=[dict(status=v.numerical_status.value,failures=v.failure_codes,checks=v.checks,metrics=v.metrics) for v in out],
                junction=None if junction is None else junction.checks,
                source_input_sha256=sha256(source.read_bytes()).hexdigest(),g_result_sha256=sha256((g_dir/'result.json').read_bytes()).hexdigest(),
                csv_sha256=sha256((g_dir/'trajectory.csv').read_bytes()).hexdigest(),phase_arrays_sha256=sha256((g_dir/'phase_arrays.npz').read_bytes()).hexdigest())


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--g-dir',required=True);parser.add_argument('--source',required=True)
    parser.add_argument('--output',required=True);parser.add_argument('--check-scene',action='store_true');a=parser.parse_args()
    output=Path(a.output);output.parent.mkdir(parents=True,exist_ok=True)
    try:result=audit(Path(a.g_dir),Path(a.source),a.check_scene)
    except Exception as e:result=dict(overall_status='FAIL',failure=repr(e))
    with output.open('x') as f:json.dump(result,f,indent=2)
    print(output,result['overall_status'])
    return 0 if result['overall_status']=='PASS' else 2

if __name__=='__main__':raise SystemExit(main())
