from types import SimpleNamespace as NS, ModuleType
import sys
import numpy as np
from v2.adapters import planning_scene_checker as mod

def setup_fake(monkeypatch,changed=False):
    class Robot:
        def __init__(self):
            self.joint_state=NS(name=list(mod.NAMES[3:]),position=[0.]*6)
            self.multi_dof_joint_state=NS(header=NS(frame_id='world'),joint_names=['world_car'],transforms=[None])
    scene=NS(is_diff=False,robot_state=Robot(),world=NS(octomap=NS(octomap=NS(id='OcTree',data=[1]))))
    class SceneRequest:
        def __init__(self):self.components=NS(components=0)
    class ValidRequest:
        def __init__(self):self.robot_state=None;self.group_name=None
    class Transform:
        def __init__(self):self.translation=NS(x=0.,y=0.);self.rotation=NS(z=0.,w=1.)
    queries=[];params={'/robot_description':'urdf','/robot_description_semantic':'srdf'};reads=[]
    def param(k):
        reads.append(k)
        return 'changed' if changed and k=='/robot_description' and reads.count(k)>1 else params[k]
    class Proxy:
        def __init__(self,name,*args,**kwargs):self.name=name
        def __call__(self,req):
            if self.name.endswith('get_planning_scene'):return NS(scene=scene)
            queries.append(req);return NS(valid=True,contacts=[])
        def close(self):pass
    rospy=ModuleType('rospy');rospy.wait_for_service=lambda *a,**kw:None;rospy.ServiceProxy=Proxy;rospy.get_param=param
    moveit=ModuleType('moveit_msgs');srv=ModuleType('moveit_msgs.srv');srv.GetPlanningScene=object;srv.GetPlanningSceneRequest=SceneRequest;srv.GetStateValidity=object;srv.GetStateValidityRequest=ValidRequest
    geometry=ModuleType('geometry_msgs');msg=ModuleType('geometry_msgs.msg');msg.Transform=Transform
    for name,m in [('rospy',rospy),('moveit_msgs',moveit),('moveit_msgs.srv',srv),('geometry_msgs',geometry),('geometry_msgs.msg',msg)]:monkeypatch.setitem(sys.modules,name,m)
    monkeypatch.setattr(mod,'wire',lambda _:b'stable-scene')
    monkeypatch.setattr(mod,'sample_phases',lambda p:(np.zeros((2,9)),[('geometry',0),('geometry',1)]))
    return queries

def test_geometry_toggle_preserves_export_and_midpoint(monkeypatch):
    queries=setup_fake(monkeypatch);phase=NS(schema=NS(frame_id='world'));q=np.array([np.arange(9),np.arange(9)+1.])
    result=mod.PlanningSceneChecker().check([phase],{'q':q});assert result['status']=='PASS' and len(queries)==5
    queries.clear();result=mod.PlanningSceneChecker().check([phase],{'q':q},include_geometry=False)
    assert result['status']=='PASS' and len(queries)==3
    assert [x.robot_state.joint_state.position for x in queries]==[q[0,3:].tolist(),q[1,3:].tolist(),((q[0,3:]+q[1,3:])/2).tolist()]

def test_changed_model_is_unknown(monkeypatch):
    setup_fake(monkeypatch,True);phase=NS(schema=NS(frame_id='world'))
    result=mod.PlanningSceneChecker().check([phase],{'q':np.zeros((2,9))},include_geometry=False)
    assert result['status']=='UNKNOWN' and 'MODEL_CHANGED' in result['reason']

def test_changed_scene_is_unknown(monkeypatch):
    setup_fake(monkeypatch);calls=iter((b'start',b'changed'))
    monkeypatch.setattr(mod,'wire',lambda _:next(calls));phase=NS(schema=NS(frame_id='world'))
    result=mod.PlanningSceneChecker().check([phase],{'q':np.zeros((2,9))},include_geometry=False)
    assert result['status']=='UNKNOWN' and 'SCENE_CHANGED' in result['reason']
