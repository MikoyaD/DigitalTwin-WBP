import unittest
from types import SimpleNamespace as S,ModuleType
from unittest.mock import patch
import numpy as np
from numpy.testing import assert_allclose
from v2.core.contracts import *
from v2.adapters.moveit_input import capture
from v2.adapters.planning_scene_checker import PlanningSceneChecker

class Message:
    def __init__(self):
        stamp=lambda t:S(to_sec=lambda:t)
        self.joint_trajectory=S(joint_names=list(NAMES[3:])[::-1],points=[S(time_from_start=stamp(t),positions=[t*.1]*6) for t in (0,1)])
        self.multi_dof_joint_trajectory=S(joint_names=['world_car'],header=S(frame_id='world'),points=[S(time_from_start=stamp(t),transforms=[S(translation=S(x=t,y=0,z=0),rotation=S(x=0,y=0,z=0,w=1))]) for t in (0,1)])
    def serialize(self,b):b.write(b'explicit synthetic message')

def metadata():
    q=np.zeros(9);goal=q.copy();goal[0]=1;goal[3:]=.1
    return dict(trial_id='mock',planner_id='RRTstar',input_stage='post_TOTG',frame_id='world',requested_start=q,requested_goal=goal,request={'source':'mock'},seed_status='uncontrolled')

class AdapterTests(unittest.TestCase):
    def test_explicit_input(self):
        p=capture(Message(),metadata());assert_allclose(p.q[-1],metadata()['requested_goal']);self.assertEqual(p.metadata['time_grid_policy'],'reject_unaligned')
    def test_reorders_names(self):
        m=Message();m.joint_trajectory.points[-1].positions=[.6,.5,.4,.3,.2,.1];meta=metadata();meta['requested_goal'][3:]=np.arange(1,7)*.1
        assert_allclose(capture(m,meta).q[-1,3:],np.arange(1,7)*.1)
    def test_time_mismatch(self):
        m=Message();m.joint_trajectory.points[-1].time_from_start=S(to_sec=lambda:.9)
        with self.assertRaisesRegex(ContractError,'TIME_GRID_MISMATCH'):capture(m,metadata())
    def test_frame_mismatch(self):
        m=Message();m.multi_dof_joint_trajectory.header.frame_id='odom'
        with self.assertRaisesRegex(ContractError,'FRAME_MISMATCH'):capture(m,metadata())
    def test_missing_identity(self):
        meta=metadata();del meta['request']
        with self.assertRaisesRegex(ContractError,'MISSING_METADATA'):capture(Message(),meta)
    def test_missing_stream(self):
        m=Message();m.multi_dof_joint_trajectory.points=[]
        with self.assertRaises(ContractError):capture(m,metadata())
    def test_nonplanar(self):
        m=Message();m.multi_dof_joint_trajectory.points[0].transforms[0].translation.z=.2
        with self.assertRaisesRegex(ContractError,'NONPLANAR_INPUT'):capture(m,metadata())
    def test_unknown_service(self):
        # Test unavailable service without importing installed ROS into numeric tests.
        ros=ModuleType('rospy');ros.wait_for_service=lambda *a,**k:(_ for _ in ()).throw(RuntimeError('unavailable'))
        srv=ModuleType('moveit_msgs.srv')
        for name in ('GetPlanningScene','GetPlanningSceneRequest','GetStateValidity','GetStateValidityRequest'):setattr(srv,name,object)
        geo=ModuleType('geometry_msgs.msg');geo.Transform=object
        with patch.dict('sys.modules',{'rospy':ros,'moveit_msgs.srv':srv,'geometry_msgs.msg':geo}):
            out=PlanningSceneChecker().check([])
        self.assertEqual(out['status'],'UNKNOWN');self.assertEqual(out['checked_states'],0)
    def test_scene_success_and_self_environment_failure_contracts(self):
        from v2.tests.support import SCHEMA
        from v2.core.math_tools import hold_jet
        state=np.zeros(9)
        ph=PhasePath('hold',PhaseType.HOLD,SCHEMA,state,state,lambda s:hold_jet(state,len(s)),'synthetic scene contract')
        class Scene:
            is_diff=False
            def __init__(self):
                self.world=S(octomap=S(octomap=S(id='OcTree',data=[1])))
                self.robot_state=S(joint_state=S(name=list(NAMES[3:]),position=[0.]*6),multi_dof_joint_state=S(header=S(frame_id='world'),joint_names=['world_car'],transforms=[None]),is_diff=False)
            def serialize(self,b):b.write(b'synthetic frozen scene')
        scene=Scene();ros=ModuleType('rospy');ros.wait_for_service=lambda *a,**k:None;ros.get_param=lambda n:'synthetic model'
        srv=ModuleType('moveit_msgs.srv');srv.GetPlanningScene=object;srv.GetStateValidity=object
        srv.GetPlanningSceneRequest=lambda:S(components=S(components=0));srv.GetStateValidityRequest=lambda:S()
        geo=ModuleType('geometry_msgs.msg');geo.Transform=lambda:S(translation=S(),rotation=S())
        def run(response,transport_failure=False):
            flags=[];closed=[]
            class ValidityProxy:
                def __call__(self,req):
                    if transport_failure:raise RuntimeError('injected transport failure')
                    return response
                def close(self):closed.append(True)
            def proxy(name,kind,**kwargs):
                if name.endswith('get_planning_scene'):return lambda req:S(scene=scene)
                flags.append(kwargs.get('persistent'));return ValidityProxy()
            ros.ServiceProxy=proxy
            with patch.dict('sys.modules',{'rospy':ros,'moveit_msgs.srv':srv,'geometry_msgs.msg':geo}):
                result=PlanningSceneChecker().check([ph])
            self.assertEqual(flags,[True]);self.assertEqual(closed,[True])
            return result
        good=run(S(valid=True,contacts=[]));self.assertEqual(good['status'],'PASS');self.assertGreater(good['checked_states'],0)
        for bodytype,category in [(0,'self_or_attached'),(1,'environment')]:
            bad=run(S(valid=False,contacts=[S(contact_body_1='a',contact_body_2='b',body_type_1=0,body_type_2=bodytype)]))
            self.assertEqual(bad['status'],'FAIL');self.assertEqual(bad['failures'][0]['contacts'][0]['category'],category)
        failed=run(S(valid=True,contacts=[]),transport_failure=True)
        self.assertEqual(failed['status'],'UNKNOWN');self.assertEqual(failed['checked_states'],0)
