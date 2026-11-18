"""Explicit workspace parsing and independent preplanning endpoint diagnostics."""
import copy,json,math
from pathlib import Path
from xml.etree import ElementTree as ET
from ..core.contracts import NAMES


def parse_workspace(config):
    w=config.get('workspace')
    if w is None:return None
    if w.get('frame_id')!='world' or config.get('frame_id')!='world':raise ValueError('WORKSPACE_FRAME_MUST_BE_WORLD')
    lo=w.get('min_corner');hi=w.get('max_corner')
    if not isinstance(lo,list) or not isinstance(hi,list) or len(lo)!=3 or len(hi)!=3:raise ValueError('WORKSPACE_REQUIRES_TWO_3D_CORNERS')
    if not all(isinstance(v,(float,int)) and not isinstance(v,bool) and math.isfinite(v) for v in lo+hi) or not all(a<b for a,b in zip(lo,hi)):raise ValueError('INVALID_WORKSPACE_BOUNDS')
    return dict(header=dict(frame_id='world'),min_corner=dict(zip(('x','y','z'),lo)),max_corner=dict(zip(('x','y','z'),hi)))


def bounds(q,urdf,workspace):
    model=ET.fromstring(urdf);out=[];violations=[]
    for i,(name,value) in enumerate(zip(NAMES,q)):
        if i<2:
            key=('x','y')[i];lo=workspace['min_corner'][key];hi=workspace['max_corner'][key];kind='planning_workspace'
        elif i==2:
            out.append(dict(variable=name,value=value,canonical=math.atan2(math.sin(value),math.cos(value)),position_bounded=False,range_representation=[-math.pi,math.pi],status='PERIODIC',note='input heading preserved; no physical angular position clamp'));continue
        else:
            limit=model.find("joint[@name='%s']/limit"%name)
            if limit is None:raise ValueError('MODEL_JOINT_LIMIT_MISSING: '+name)
            lo=float(limit.attrib['lower']);hi=float(limit.attrib['upper']);kind='model_joint_position'
        valid=lo<=value<=hi;item=dict(variable=name,value=value,lower=lo,upper=hi,kind=kind,status='PASS' if valid else 'INVALID_BOUNDS');out.append(item)
        if not valid:violations.append(item)
    return dict(status='PASS' if not violations else 'INVALID_BOUNDS',dimensions=out,violations=violations)


def endpoint_report(req,data):
    import rospy
    from moveit_msgs.srv import GetPlanningScene,GetPlanningSceneRequest,GetStateValidity,GetStateValidityRequest
    from .display_g import _transform
    result=dict(frame_id=data['metadata']['frame_id'],request_workspace=data['motion_plan_request'].get('workspace_parameters'),default_parameters={})
    for name in ('/move_group/default_workspace_bounds','/move_group/planning_pipelines/ompl/default_workspace_bounds'):
        result['default_parameters'][name]=rospy.get_param(name,None)
    workspace=result['request_workspace']
    if workspace is None:
        extent=result['default_parameters']['/move_group/planning_pipelines/ompl/default_workspace_bounds'] or 10.
        workspace=dict(header=dict(frame_id='world'),min_corner=dict.fromkeys(('x','y','z'),-extent/2),max_corner=dict.fromkeys(('x','y','z'),extent/2))
    result['effective_workspace_expected']=workspace
    urdf=rospy.get_param('/robot_description')
    for label,key in [('start','requested_start'),('goal','requested_goal')]:result[label]=dict(bounds=bounds(data['metadata'][key],urdf,workspace),collision=dict(status='UNKNOWN'))
    try:
        rospy.wait_for_service('/get_planning_scene',timeout=3);rospy.wait_for_service('/check_state_validity',timeout=3)
        call=GetPlanningSceneRequest();call.components.components=1023;scene=rospy.ServiceProxy('/get_planning_scene',GetPlanningScene)(call).scene
        if not scene.world.octomap.octomap.data:raise ValueError('STATIC_OBSTACLE_MISSING')
        if scene.robot_state.multi_dof_joint_state.header.frame_id!='world':raise ValueError('SCENE_FRAME_UNKNOWN')
        valid=rospy.ServiceProxy('/check_state_validity',GetStateValidity)
        for label,key in [('start','requested_start'),('goal','requested_goal')]:
            q=data['metadata'][key];query=GetStateValidityRequest();query.robot_state=copy.deepcopy(scene.robot_state);query.group_name=''
            js=query.robot_state.joint_state;lookup={n:i for i,n in enumerate(js.name)};js.position=list(js.position)
            for name,value in zip(NAMES[3:],q[3:]):js.position[lookup[name]]=value
            md=query.robot_state.multi_dof_joint_state;md.transforms[list(md.joint_names).index('world_car')]=_transform(q);query.robot_state.is_diff=False
            response=valid(query)
            status='PASS' if response.valid else (label.upper()+'_IN_COLLISION' if response.contacts else 'UNKNOWN_STATE_INVALID')
            result[label]['collision']=dict(status=status,valid=response.valid,contacts=[dict(body1=c.contact_body_1,body2=c.contact_body_2) for c in response.contacts])
    except Exception as exc:
        result['scene_query_error']=repr(exc)
    return result
