import copy,subprocess,unittest
from pathlib import Path
from xml.etree import ElementTree as ET
import yaml
from v2.adapters.workspace_check import parse_workspace,bounds
from v2.scripts.planning_request import build
from v2.tests.support import PACKAGE

class WorkspaceTests(unittest.TestCase):
 def config(self):return yaml.safe_load((PACKAGE/'v2/configs/v2_workspace_single_goal.yaml').read_text())
 def test_explicit_workspace_request_and_legacy(self):
  c=self.config();r=build(c,c['tasks'][0],PACKAGE)
  self.assertEqual(r['motion_plan_request']['workspace_parameters']['min_corner']['y'],-20.)
  self.assertEqual(r['metadata']['requested_start'][1],-7.7584)
  c.pop('workspace');self.assertNotIn('workspace_parameters',build(c,c['tasks'][0],PACKAGE)['motion_plan_request'])
 def test_invalid_workspace(self):
  for field,value in [('min_corner',[0,0,0]),('max_corner',[-20,20,5]),('max_corner',[20,float('nan'),5]),('frame_id','odom')]:
   c=self.config();c['workspace'][field]=value
   if field=='min_corner':c['workspace']['max_corner']=[0,0,0]
   with self.assertRaises(ValueError):parse_workspace(c)
 def test_endpoint_specific_dimensions(self):
  c=self.config();w=parse_workspace(c);u=(PACKAGE/'config/gazebo_combined_robot.urdf').read_text()
  self.assertEqual(bounds(c['tasks'][0]['start_q'],u,w)['status'],'PASS')
  self.assertEqual(bounds(c['tasks'][0]['goal_q'],u,w)['status'],'PASS')
  old=copy.deepcopy(w)
  for k in ('x','y'):old['min_corner'][k]=-5;old['max_corner'][k]=5
  for key in ('start_q','goal_q'):
   out=bounds(c['tasks'][0][key],u,old);self.assertEqual([v['variable'] for v in out['violations']],['world_car/y'])
  q=list(c['tasks'][0]['goal_q']);q[3]=4.
  self.assertEqual(bounds(q,u,w)['violations'][0]['variable'],'joint1')
 def test_ros_wire_and_launch_namespace(self):
  launch=ET.parse(PACKAGE/'v2/launch/v2_headless_test.launch').getroot()
  move=next(n for n in launch.findall('node') if n.attrib['name']=='move_group')
  self.assertIn('planning_pipelines/ompl/default_workspace_bounds',[p.attrib['name'] for p in move.findall('param')])
  code="""
import yaml,io
from pathlib import Path
from genpy.message import fill_message_args
from moveit_msgs.srv import GetMotionPlanRequest
from v2.scripts.planning_request import build
p=Path.cwd();c=yaml.safe_load((p/'v2/configs/v2_workspace_single_goal.yaml').read_text());r=GetMotionPlanRequest();fill_message_args(r.motion_plan_request,[build(c,c['tasks'][0],p)['motion_plan_request']]);b=io.BytesIO();r.serialize(b);d=GetMotionPlanRequest();d.deserialize(b.getvalue());assert d.motion_plan_request.workspace_parameters.header.frame_id=='world';assert d.motion_plan_request.workspace_parameters.min_corner.y==-20.;assert d.motion_plan_request.workspace_parameters.max_corner.z==5.
"""
  result=subprocess.run(['bash','--noprofile','--norc','-c','source v2/scripts/env_noetic.sh; exec python3 -B -c "$1"','_',code],cwd=PACKAGE,capture_output=True,text=True)
  self.assertEqual(result.returncode,0,result.stdout+result.stderr)
