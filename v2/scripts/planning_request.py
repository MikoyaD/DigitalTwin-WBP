"""Create explicit whole-group request from archived waypoint candidates."""
import json
import math
from pathlib import Path
from hashlib import sha256
from ..core.contracts import NAMES,require


def waypoint_q(record):
    st=record['state'];tr=st['world_car']['translation'];rot=st['world_car']['rotation']
    require(abs(tr[2])<1e-9 and abs(rot[0])+abs(rot[1])<1e-9,'WAYPOINT_NONPLANAR','archived waypoint')
    return [tr[0],tr[1],2*math.atan2(rot[2],rot[3])]+[st[n] for n in NAMES[3:]]


def transform(q):
    return dict(translation=dict(x=q[0],y=q[1],z=0),rotation=dict(x=0,y=0,z=math.sin(q[2]/2),w=math.cos(q[2]/2)))


def build(config,task,package):
    require(config.get('group_name','whole')=='whole','GROUP','group_name must be whole')
    require(0<float(config['allowed_planning_time_s'])<=120,'PLANNING_TIME','seconds in (0,120]')
    if 'start_q' in task:
        raw=json.dumps(task,sort_keys=True).encode();start=task['start_q'];goal=task.get('goal_q',start)
    else:
        source=package/config['waypoints_source'];raw=source.read_bytes();waypoints={x['id']:x for x in json.loads(raw)}
        start=waypoint_q(waypoints[task['start_waypoint_id']]);goal=waypoint_q(waypoints[task['goal_waypoint_id']])
    import numpy as np
    require(np.asarray(start).shape==(9,) and np.asarray(goal).shape==(9,) and np.all(np.isfinite(start)) and np.all(np.isfinite(goal)),'STATE','finite nine-dimensional states required')
    tol=task.get('goal_tolerance',[.03,.03,.05]+[.05]*6)
    require(len(tol)==9 and np.all(np.isfinite(tol)) and min(tol)>0,'TOLERANCE','nine positive tolerances')
    req=dict(group_name='whole',pipeline_id='ompl',planner_id=config['planner_id'],num_planning_attempts=1,
             allowed_planning_time=config['allowed_planning_time_s'],max_velocity_scaling_factor=config['velocity_scaling'],
             max_acceleration_scaling_factor=config['acceleration_scaling'],
             start_state=dict(is_diff=False,joint_state=dict(name=list(NAMES[3:]),position=start[3:]),
                              multi_dof_joint_state=dict(header=dict(frame_id=config['frame_id']),joint_names=['world_car'],transforms=[transform(start)])),
             goal_constraints=[dict(joint_constraints=[dict(joint_name=name,position=value,tolerance_above=t,tolerance_below=t,weight=1.)
                                                       for name,value,t in zip(NAMES,goal,tol)])])
    from ..adapters.workspace_check import parse_workspace
    workspace=parse_workspace(config)
    if workspace is not None:req['workspace_parameters']=workspace
    if 'pose_goal' in task:
        pose=task['pose_goal'];pos=pose['position'];quat=pose['orientation_xyzw']
        require(len(pos)==3 and len(quat)==4 and np.all(np.isfinite(pos+quat)) and abs(np.linalg.norm(quat)-1)<1e-6,'POSE','finite normalized quaternion')
        pt=float(pose['position_tolerance_m']);at=float(pose['orientation_tolerance_rad'])
        require(np.isfinite(pt) and np.isfinite(at) and pt>0 and at>0,'POSE_TOLERANCE','positive SI tolerances')
        xyz=dict(zip(('x','y','z'),pos));rotation=dict(zip(('x','y','z','w'),quat));header=dict(frame_id=pose['frame_id'])
        req['goal_constraints']=[dict(position_constraints=[dict(header=header,link_name=pose['link_name'],weight=1.,constraint_region=dict(primitives=[dict(type=2,dimensions=[pt])],primitive_poses=[dict(position=xyz,orientation=dict(w=1.))]))],orientation_constraints=[dict(header=header,link_name=pose['link_name'],orientation=rotation,absolute_x_axis_tolerance=at,absolute_y_axis_tolerance=at,absolute_z_axis_tolerance=at,weight=1.)])]
    return dict(backend_limits=config.get('backend_limits',{}),motion_plan_request=req,metadata=dict(frame_id=config['frame_id'],requested_start=start,requested_goal=goal,
               goal_tolerance=tol,input_stage=config['input_stage'],waypoint_source_sha256=sha256(raw).hexdigest(),task_id=task['id']),
               research_limits=config['research_limits'],velocity_scaling=config['velocity_scaling'],acceleration_scaling=config['acceleration_scaling'])
