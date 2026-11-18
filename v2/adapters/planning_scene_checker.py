"""Full-state discrete scene checks. No ROS import on numerical code path."""
import copy
import time
import io
from hashlib import sha256
import numpy as np
from ..core.contracts import Status,ContractError,require,NAMES


def wire(msg):
    b=io.BytesIO();msg.serialize(b);return b.getvalue()


def sample_phases(phases,max_states=100000):
    states=[];locations=[]
    for phase in phases:
        spline=getattr(getattr(phase,'evaluator',None),'geometry_spline',None)
        grid=np.unique(np.r_[np.linspace(0,1,65),spline.knots if spline is not None else []])
        for retry in range(16):
            mid=(grid[:-1]+grid[1:])/2;q=phase.evaluate(grid).q;qm=phase.evaluate(mid).q
            scale=np.array([.005,.005]+[.01]*7)
            bad=(np.linalg.norm(np.diff(q[:,:2],axis=0),axis=1)>.005)|(np.max(np.abs(np.diff(q,axis=0))/scale,axis=1)>1)|(np.max(np.abs(qm-(q[:-1]+q[1:])/2)/scale,axis=1)>.25)
            if not np.any(bad):
                # Required half-spacing second pass, with exact curve evaluations.
                grid=np.sort(np.r_[grid,mid]);break
            grid=np.sort(np.r_[grid,mid[bad]])
            require(len(grid)+len(states)<max_states,'COLLISION_SAMPLE_BUDGET','adaptive refinement')
        else:raise ContractError('COLLISION_SAMPLE_BUDGET','subdivision exhausted')
        qq=phase.evaluate(grid).q;states.extend(qq);locations.extend((phase.phase_id,float(s)) for s in grid)
    require(len(states)<=max_states,'COLLISION_SAMPLE_BUDGET','all phases')
    return np.asarray(states),locations


class PlanningSceneChecker:
    def __init__(self,namespace=''):
        self.namespace=namespace.rstrip('/')

    def check(self,phases,export=None,include_geometry=True):
        result=dict(status='UNKNOWN',reason='not connected',checked_states=0,failures=[],
                    coverage='discrete full RobotState; not continuous collision guarantee',resolution=dict(xy_m=.005,angle_rad=.01,half_spacing_pass=True))
        valid=None
        try:
            import rospy
            from moveit_msgs.srv import GetPlanningScene,GetPlanningSceneRequest,GetStateValidity,GetStateValidityRequest
            from geometry_msgs.msg import Transform
            for name in ('get_planning_scene','check_state_validity'):
                rospy.wait_for_service(self.namespace+'/'+name,timeout=3.)
            get=rospy.ServiceProxy(self.namespace+'/get_planning_scene',GetPlanningScene)
            # Reuse one TCPROS connection for the same serial query sequence.
            # Failure still returns UNKNOWN; no skipped states or relaxed budget.
            valid=rospy.ServiceProxy(self.namespace+'/check_state_validity',GetStateValidity,persistent=True)
            result['service_transport']='persistent TCPROS; no automatic retry'
            # All defined PlanningSceneComponents through OBJECT_COLORS (1023).
            scene_request=GetPlanningSceneRequest();scene_request.components.components=1023
            scene=get(scene_request).scene
            urdf=rospy.get_param('/robot_description');srdf=rospy.get_param('/robot_description_semantic')
            require(bool(urdf) and bool(srdf),'MODEL_MISSING','runtime model parameters')
            require(not scene.is_diff,'SCENE_INCOMPLETE','full snapshot required')
            octomap = scene.world.octomap.octomap
            require(bool(octomap.id) and len(octomap.data) > 0,
                    'STATIC_OBSTACLE_MISSING', 'warehouse OctoMap is absent from PlanningScene')
            scene_frame = scene.robot_state.multi_dof_joint_state.header.frame_id
            require(scene_frame == phases[0].schema.frame_id,
                    'FRAME_MISMATCH','scene planar frame/TF unavailable: %s' % scene_frame)
            original=wire(scene);digest=sha256(original).hexdigest()
            result.update(scene_sha256=digest,scene_snapshot_hex=original.hex(),model_sha256=sha256((urdf+'\n'+srdf).encode()).hexdigest(),
                          robot_description=urdf,robot_description_semantic=srdf)
            if include_geometry:
                states,locations=sample_phases(phases)
            else:
                require(export is not None,'COLLISION_INPUT_MISSING','geometry and export both absent')
                states=np.empty((0,9));locations=[]
            if export is not None:
                extra=np.asarray(export['q']);mid=(extra[:-1]+extra[1:])/2
                states=np.vstack((states,extra,mid));locations+= [('export',i) for i in range(len(extra))]+[('export_linear_midpoint',i) for i in range(len(mid))]
            require(len(states)<=200000,'COLLISION_SAMPLE_BUDGET','export and analytic sample total')
            started=time.monotonic()
            for q,location in zip(states,locations):
                require(time.monotonic()-started<60,'COLLISION_TIME_BUDGET','60 seconds discrete query budget')
                req=GetStateValidityRequest();req.robot_state=copy.deepcopy(scene.robot_state);req.group_name=''
                js=req.robot_state.joint_state;lookup={n:i for i,n in enumerate(js.name)};positions=list(js.position)
                require(all(n in lookup for n in NAMES[3:]),'STATE_INCOMPLETE','arm joints missing in scene')
                for n,value in zip(NAMES[3:],q[3:]):positions[lookup[n]]=float(value)
                js.position=positions;md=req.robot_state.multi_dof_joint_state
                require('world_car' in md.joint_names,'STATE_INCOMPLETE','planar joint missing')
                tr=Transform();tr.translation.x=q[0];tr.translation.y=q[1];tr.rotation.z=np.sin(q[2]/2);tr.rotation.w=np.cos(q[2]/2)
                md.transforms[list(md.joint_names).index('world_car')]=tr
                req.robot_state.is_diff=False
                response=valid(req);result['checked_states']+=1
                if not response.valid:
                    contacts=[dict(body1=c.contact_body_1,body2=c.contact_body_2,type1=c.body_type_1,type2=c.body_type_2,
                                   category='environment' if 1 in (c.body_type_1,c.body_type_2) else 'self_or_attached') for c in response.contacts]
                    result['failures'].append(dict(location=location,q=q.tolist(),contacts=contacts))
                    result.update(status='FAIL',reason='invalid complete state');return result
            # Live service uses current scene: conservatively require byte-identical snapshot.
            require(wire(get(scene_request).scene)==original,'SCENE_CHANGED','cannot claim frozen-scene check')
            require(rospy.get_param('/robot_description')==urdf and rospy.get_param('/robot_description_semantic')==srdf,'MODEL_CHANGED','cannot reuse geometry checks')
            result.update(status='PASS',reason='all sampled complete states valid in unchanged scene')
        except Exception as e:
            result.update(status='UNKNOWN',reason=type(e).__name__+': '+str(e))
        finally:
            if valid is not None:valid.close()
        return result
