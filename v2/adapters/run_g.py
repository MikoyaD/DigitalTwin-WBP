"""Independent JSON/RobotTrajectory entry. Never executes or publishes motion.

python3 -B -m v2.adapters.run_g input.json --check-scene
Input keys: robot_trajectory_hex, metadata (capture contract), research_limits
{key:{value,unit,kind,source}}, velocity_scaling, acceleration_scaling.
"""
import argparse
import json
import uuid
from pathlib import Path
from hashlib import sha256
from dataclasses import asdict
import numpy as np
from ..core.contracts import Quantity,require
from ..core.limits import load_limits
from .moveit_input import capture
from .planning_scene_checker import PlanningSceneChecker
from ..geometry.phase_segmenter import build_g


def plain(x):
    if isinstance(x,np.ndarray):return x.tolist()
    if isinstance(x,np.generic):return x.item()
    raise TypeError(type(x).__name__)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('input');parser.add_argument('--check-scene',action='store_true')
    parser.add_argument('--output-dir',help='new, nonexisting V2 run subdirectory');args=parser.parse_args()
    root=Path(__file__).resolve().parents[1];package=root.parent
    folder=Path(args.output_dir).resolve() if args.output_dir else root/'outputs'/('g_'+uuid.uuid4().hex)
    folder.mkdir(parents=True,exist_ok=False)
    data=json.loads(Path(args.input).read_text());result={'source_input_sha256':sha256(Path(args.input).read_bytes()).hexdigest(),'status':'FAIL'}
    try:
        from moveit_msgs.msg import RobotTrajectory
        msg=RobotTrajectory();msg.deserialize(bytes.fromhex(data['robot_trajectory_hex']))
        path=capture(msg,data['metadata'])
        research={k:Quantity(**v) for k,v in data['research_limits'].items()}
        limits=load_limits(package/'config/gazebo_combined_robot.urdf',package/'config/joint_limits.yaml',package/'config/combined_robot.srdf',research,
                           velocity_scaling=data['velocity_scaling'],acceleration_scaling=data['acceleration_scaling'],backend_overrides=data.get('backend_limits'))
        out=build_g(path,limits,hold_duration=data.get('hold_duration'))
        collision={'status':'UNKNOWN','reason':'scene checking not requested'}
        if args.check_scene:
            import rospy
            rospy.init_node('v2_g_readonly_check',anonymous=True,disable_signals=True)
            require(sha256(rospy.get_param('/robot_description').encode()).hexdigest()==sha256((package/'config/gazebo_combined_robot.urdf').read_bytes()).hexdigest(),
                    'MODEL_MISMATCH','review runtime/model numeric limits before acceptance')
            collision=PlanningSceneChecker().check(out['phases'],out['output'])
            expected=data['metadata'].get('scene_sha256')
            if expected and collision.get('scene_sha256') != expected:
                collision.update(status='UNKNOWN',reason='PARENT_SCENE_MISMATCH',expected_scene_sha256=expected)
        columns=np.column_stack([out['output'][k] for k in ('time_s','q','velocity','acceleration','jerk')])
        np.savetxt(str(folder/'trajectory.csv'),columns,delimiter=',',fmt='%.17g',header='time_s,'+','.join(prefix+n for prefix in ('q_','v_','a_','jerk_') for n in path.schema.joint_names),comments='')
        phase_arrays={}
        phases=[]
        for i,(p,r,v) in enumerate(zip(out['phases'],out['records'],out['validation'])):
            for field in ('time_s','q','velocity','acceleration','jerk'):
                phase_arrays['phase_%03d_%s'%(i,field)]=getattr(r,field)
            spline=getattr(p.evaluator,'geometry_spline',None)
            phases.append(dict(id=p.phase_id,kind=p.phase_type,start=p.start,end=p.end,duration_s=float(r.time_s[-1]-r.time_s[0]),
                               validation=asdict(v),controls=None if spline is None else spline.controls,knots=None if spline is None else spline.knots))
        np.savez_compressed(str(folder/'phase_arrays.npz'),**phase_arrays)
        result.update(status=collision['status'],numerical_status='PASS',collision=collision,phases=phases,
                      duration_s=out['duration_s'],xy_length_m=out['xy_length_m'],backend_s=out['backend_s'],decisions=out['decisions'],
                      path=asdict(path),limits=asdict(limits),time_law='10u^3-15u^4+6u^5 per phase; analytic derivatives; seam jerk has two one-sided values')
    except Exception as e:result.update(error=repr(e))
    result['code_hashes']={str(p.relative_to(root)):sha256(p.read_bytes()).hexdigest() for p in root.rglob('*.py')}
    with (folder/'result.json').open('x') as f:json.dump(result,f,indent=2,default=plain)
    print(folder);print(result['status'],result.get('error',''))
    return 0 if result['status']=='PASS' else 2

if __name__=='__main__':raise SystemExit(main())
