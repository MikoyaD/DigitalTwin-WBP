import copy,subprocess,unittest
from pathlib import Path
import yaml,numpy as np
from v2.scripts.planning_request import build
from v2.core.limits import load_limits
from v2.core.contracts import ContractError
from v2.tests.support import PACKAGE,research_limits,record
from v2.timing.feasible_profile import ratio
from v2.scripts.diagnose_g import diagnose

class ConfigLiteTests(unittest.TestCase):
 def test_request_fields_and_group(self):
  c=yaml.safe_load((PACKAGE/'v2/configs/single_goal_template.yaml').read_text());t=c['tasks'][0]
  c.update(planner_id='RRTConnect',allowed_planning_time_s=7.)
  r=build(c,t,PACKAGE)['motion_plan_request'];self.assertEqual(r['planner_id'],'RRTConnect');self.assertEqual(r['allowed_planning_time'],7.);self.assertEqual(len(r['goal_constraints'][0]['joint_constraints']),9)
  self.assertEqual(r['start_state']['joint_state']['position'],t['start_q'][3:])
  c['group_name']='arm'
  with self.assertRaises(ContractError):build(c,t,PACKAGE)
 def test_limits_reach_ratio_and_reject(self):
  args=(PACKAGE/'config/gazebo_combined_robot.urdf',PACKAGE/'config/joint_limits.yaml',PACKAGE/'config/combined_robot.srdf',research_limits())
  l=load_limits(*args,velocity_scaling=.5,acceleration_scaling=.5)
  q=dict(value=[.02,.02],unit='m/s',kind='research_limit',source='test explicit reduction')
  reduced=load_limits(*args,velocity_scaling=.5,acceleration_scaling=.5,backend_overrides={'xy_component_velocity':q})
  self.assertGreater(ratio(record(),reduced),ratio(record(),l))
  q=dict(value=[.1]*6,unit='rad/s',kind='research_limit',source='test')
  arm=load_limits(*args,velocity_scaling=.5,acceleration_scaling=.5,backend_overrides={'arm_velocity':q});np.testing.assert_allclose(arm.arm_velocity.value,.1)
  for value in ([9]*6,[-1]*6):
   q['value']=value
   with self.assertRaises(ContractError):load_limits(*args,velocity_scaling=.5,acceleration_scaling=.5,backend_overrides={'arm_velocity':q})
 def test_saved_diagnostic(self):
  folder=PACKAGE/'v2/outputs/runs/run_20261022T092444Z_adb7f2f5/g_archived_1_to_2_turn_arm'
  d=diagnose(folder);self.assertAlmostEqual(sum(p['duration_s'] for p in d['phases']),d['duration_s']);self.assertAlmostEqual(sum(d['time_fraction'].values()),1.)
 def test_pose_ros_constraints(self):
  code="""
import yaml
from genpy.message import fill_message_args
from moveit_msgs.srv import GetMotionPlanRequest
from v2.scripts.planning_request import build
from pathlib import Path
p=Path.cwd();c=yaml.safe_load((p/'v2/configs/single_goal_template.yaml').read_text());t=c['tasks'][0];t['pose_goal']=dict(frame_id='world',link_name='Link6',position=[1.,0.,1.],orientation_xyzw=[0.,0.,0.,1.],position_tolerance_m=.02,orientation_tolerance_rad=.1)
r=GetMotionPlanRequest();fill_message_args(r.motion_plan_request,[build(c,t,p)['motion_plan_request']]);g=r.motion_plan_request.goal_constraints[0];assert g.position_constraints[0].constraint_region.primitives[0].type==2;assert g.orientation_constraints[0].link_name=='Link6';assert not g.joint_constraints
"""
  r=subprocess.run(['bash','--noprofile','--norc','-c','source v2/scripts/env_noetic.sh; exec python3 -B -c "$1"','_',code],cwd=PACKAGE,capture_output=True,text=True)
  self.assertEqual(r.returncode,0,r.stdout+r.stderr)
