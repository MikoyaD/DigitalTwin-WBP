"""Read a designated RobotTrajectory; never subscribe to latest caches."""
import io
from hashlib import sha256
import numpy as np
from ..core.contracts import *


def capture(msg,metadata):
    required=('trial_id','planner_id','input_stage','frame_id','requested_start','requested_goal','request','seed_status')
    require(all(k in metadata for k in required),'MISSING_METADATA',str(required))
    jt,mt=msg.joint_trajectory,msg.multi_dof_joint_trajectory
    require(set(jt.joint_names)==set(NAMES[3:]) and len(jt.joint_names)==6,'JOINT_ORDER','six named arm joints required')
    require(list(mt.joint_names)==['world_car'],'JOINT_ORDER','one planar world_car transform required')
    require(mt.header.frame_id==metadata['frame_id'],'FRAME_MISMATCH','explicit transform needed')
    at=times([p.time_from_start.to_sec() for p in jt.points]);bt=times([p.time_from_start.to_sec() for p in mt.points])
    require(np.array_equal(at,bt),'TIME_GRID_MISMATCH','explicitly reject asynchronous streams; no index merge')
    arms=np.asarray([p.positions for p in jt.points])[:,[list(jt.joint_names).index(n) for n in NAMES[3:]]]
    base=[]
    for p in mt.points:
        require(len(p.transforms)==1,'DIMENSION','single base transform')
        tr=p.transforms[0];r=tr.rotation;t=tr.translation
        require(abs(r.x)<1e-9 and abs(r.y)<1e-9 and abs(t.z)<1e-9,'NONPLANAR_INPUT','only planar axle pose')
        require(abs(r.z*r.z+r.w*r.w-1)<1e-6,'QUATERNION','unit quaternion required')
        base.append([t.x,t.y,2*np.arctan2(r.z,r.w)])
    base=np.asarray(base);base[:,2]=np.unwrap(base[:,2])
    require(np.max(np.abs(np.diff(base[:,2])))<np.pi-1e-6,'AMBIGUOUS_WINDING','dense source orientation required')
    q=np.column_stack((base,arms));schema=StateSchema(NAMES,POSITION_UNITS,metadata['frame_id'],'car_link','s')
    raw=io.BytesIO();msg.serialize(raw)
    details=dict(metadata);details['raw_robot_trajectory_hex']=raw.getvalue().hex();details['time_grid_policy']='reject_unaligned'
    # Goal orientation is compared modulo 2pi; preserve requested physical pose.
    start=np.asarray(metadata['requested_start'],float).copy();goal=np.asarray(metadata['requested_goal'],float).copy()
    goal_tolerance=np.asarray(metadata.get('goal_tolerance',np.full(9,1e-6)),float)
    require(goal_tolerance.shape==(9,) and np.all(np.isfinite(goal_tolerance)) and np.all(goal_tolerance>0),
            'GOAL_TOLERANCE','nine positive tolerances required')
    for e,target,tolerance in [(q[0],start,np.full(9,1e-6)),(q[-1],goal,goal_tolerance)]:
        error=e-target;error[2]=np.arctan2(np.sin(error[2]),np.cos(error[2]))
        require(np.all(np.abs(error)<=tolerance+1e-9),'INPUT_ENDPOINT_MISMATCH','planning response outside request region')
    details['requested_start_original']=start.tolist();details['requested_goal_original']=goal.tolist()
    start=q[0].copy();goal=q[-1].copy()
    details['goal_tolerance']=goal_tolerance.tolist()
    return PathRecord(metadata['trial_id'],metadata['planner_id'],sha256(raw.getvalue()).hexdigest(),
                      metadata['input_stage'],schema,q,at,start,goal,details)
