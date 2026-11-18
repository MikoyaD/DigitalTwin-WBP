"""Capture one explicit GetMotionPlan request, never execute or publish trajectory.

Input JSON: motion_plan_request (ROS dictionary), metadata, research_limits,
velocity_scaling, acceleration_scaling. Output is a unique directory.
"""
import argparse
import io
import json
import time
import uuid
from pathlib import Path
from hashlib import sha256


def main():
    import rospy
    from genpy.message import fill_message_args
    from moveit_msgs.srv import GetMotionPlan,GetMotionPlanRequest
    parser=argparse.ArgumentParser();parser.add_argument('request_json')
    parser.add_argument('--output-dir',help='new, nonexisting V2 run subdirectory')
    args=parser.parse_args()
    data=json.loads(Path(args.request_json).read_text());root=Path(__file__).resolve().parents[1]
    folder=Path(args.output_dir).resolve() if args.output_dir else root/'outputs'/('capture_'+uuid.uuid4().hex)
    folder.mkdir(parents=True,exist_ok=False)
    record=dict(status='FAIL',source_request_sha256=sha256(Path(args.request_json).read_bytes()).hexdigest())
    try:
        req=GetMotionPlanRequest();fill_message_args(req.motion_plan_request,[data['motion_plan_request']])
        if not req.motion_plan_request.planner_id:raise ValueError('explicit planner_id required')
        if req.motion_plan_request.group_name!='whole':raise ValueError('whole nine-dimensional group required')
        if req.motion_plan_request.num_planning_attempts!=1:raise ValueError('one frontend attempt per capture')
        if not 0<req.motion_plan_request.allowed_planning_time<=120:raise ValueError('short preflight budget must be <=120 seconds')
        if req.motion_plan_request.start_state.is_diff:raise ValueError('explicit complete start state required')
        rospy.init_node('v2_capture_plan',anonymous=True,disable_signals=True)
        rospy.wait_for_service('/plan_kinematic_path',timeout=3.)
        call=rospy.ServiceProxy('/plan_kinematic_path',GetMotionPlan)
        raw=io.BytesIO();req.serialize(raw);record['request_hex']=raw.getvalue().hex()
        if 'workspace_parameters' in data['motion_plan_request']:
            from .workspace_check import endpoint_report
            report=endpoint_report(req,data)
            with (folder/'endpoint_checks.json').open('x') as f:json.dump(report,f,indent=2)
            record['endpoint_checks']=report
            with (folder/'request_wire.bin').open('xb') as f:f.write(raw.getvalue())
            import subprocess
            probe=root/'outputs/workspace_tools/workspace_probe'
            if not probe.is_file():raise ValueError('WORKSPACE_PROBE_MISSING: compile v2/adapters/workspace_probe.cpp')
            done=subprocess.run([str(probe),str(folder/'request_wire.bin'),str(folder/'effective_ompl_bounds.json')],capture_output=True,text=True,timeout=15)
            (folder/'workspace_probe.log').write_text(done.stdout+done.stderr)
            if done.returncode:raise ValueError('WORKSPACE_PROBE_FAILED: '+str(done.returncode))
            effective=json.loads((folder/'effective_ompl_bounds.json').read_text());record['effective_ompl_bounds']=effective
            for variable in effective['variables']:
                if variable['name'] in ('world_car/x','world_car/y'):
                    axis=variable['name'][-1];w=data['motion_plan_request']['workspace_parameters']
                    if variable['lower']!=w['min_corner'][axis] or variable['upper']!=w['max_corner'][axis]:raise ValueError('EFFECTIVE_OMPL_WORKSPACE_MISMATCH')
            for endpoint in ('start','goal'):
                if report[endpoint]['bounds']['status']!='PASS':raise ValueError('INVALID_BOUNDS: '+endpoint)
                if report[endpoint]['collision']['status']!='PASS':raise ValueError(report[endpoint]['collision']['status']+': '+endpoint)
            # Ask the real move_group to log the planning volume used after adapters.
            try:
                from roscpp.srv import GetLoggers,SetLoggerLevel
                loggers=rospy.ServiceProxy('/move_group/get_loggers',GetLoggers)().loggers
                logger=next(x.name for x in loggers if x.name.endswith('model_based_planning_context'))
                rospy.ServiceProxy('/move_group/set_logger_level',SetLoggerLevel)(logger,'DEBUG')
                record['planning_volume_debug_logger']=logger
            except Exception as exc:record['planning_volume_debug_logger']='UNKNOWN: '+str(exc)

        started=time.perf_counter();response=call(req).motion_plan_response;record['front_wall_s']=time.perf_counter()-started
        record.update(error_code=response.error_code.val,reported_planning_time_s=response.planning_time,planning_failure_class={-6:'PLANNING_TIMEOUT',-10:'START_IN_COLLISION',-12:'GOAL_IN_COLLISION',-26:'INVALID_BOUNDS_OR_START_STATE'}.get(response.error_code.val,'SUCCESS' if response.error_code.val==1 else 'UNKNOWN'))
        wire=io.BytesIO();response.serialize(wire);record['response_hex']=wire.getvalue().hex()
        if response.error_code.val!=1:raise ValueError('MoveIt planning failed: '+str(response.error_code.val))
        raw=io.BytesIO();response.trajectory.serialize(raw)
        meta=dict(data['metadata']);meta.update(trial_id=folder.name,planner_id=req.motion_plan_request.planner_id,
                                                request=data['motion_plan_request'],seed_status='uncontrolled')
        payload=dict(data);payload.update(robot_trajectory_hex=raw.getvalue().hex(),metadata=meta)
        # Validate source alignment/endpoints now. No latest cache or auto targets.
        if any(c.position_constraints for c in req.motion_plan_request.goal_constraints):raise ValueError('POSE_CAPTURE_NOT_SUPPORTED: nine-dimensional endpoint contract required for G; pose request saved only')
        from .moveit_input import capture
        path=capture(response.trajectory,meta)
        with (folder/'g_input.json').open('x') as f:json.dump(payload,f,indent=2)
        from dataclasses import asdict
        import numpy as np
        def plain(value):
            if isinstance(value,np.ndarray):return value.tolist()
            raise TypeError(type(value).__name__)
        with (folder/'parent_path.json').open('x') as f:json.dump(asdict(path),f,indent=2,default=plain)
        record['parent_path_hash']=path.parent_path_hash
        record['status']='CAPTURED_NOT_COLLISION_VERIFIED'
    except Exception as e:record['error']=repr(e)
    with (folder/'capture_result.json').open('x') as f:json.dump(record,f,indent=2)
    print(folder);print(record['status'],record.get('error',''))
    return 0 if record['status']=='CAPTURED_NOT_COLLISION_VERIFIED' else 2

if __name__=='__main__':raise SystemExit(main())
