"""Exact user configuration and read-only, bounded run discovery regression."""
import subprocess,unittest
from pathlib import Path
import numpy as np,yaml
from v2.tests.support import PACKAGE,research_limits
from v2.core.contracts import Quantity
from v2.core.limits import load_limits
from v2.scripts.planning_request import build

class SingleGoalDiscoveryTests(unittest.TestCase):
 def test_exact_user_states_and_half_limits(self):
  c=yaml.safe_load((PACKAGE/'v2/configs/v2_single_goal_05.yaml').read_text());task=c['tasks'][0]
  self.assertEqual(task['id'],'single_goal_05');self.assertEqual(task['start_q'],[-.0002,-.0001,.0044,.0003,-.0001,0.,.0001,0.,0.]);self.assertEqual(task['goal_q'],[4.9879,.0261,-3.1532,-.8356,.2963,-2.3206,2.0796,.7188,.0743])
  l=load_limits(PACKAGE/'config/gazebo_combined_robot.urdf',PACKAGE/'config/joint_limits.yaml',PACKAGE/'config/combined_robot.srdf',{k:Quantity(**v) for k,v in c['research_limits'].items()},velocity_scaling=c['velocity_scaling'],acceleration_scaling=c['acceleration_scaling'],backend_overrides=c['backend_limits'])
  for key in c['backend_limits']:np.testing.assert_allclose(getattr(l,key).value,.5)
  req=build(c,task,PACKAGE)['motion_plan_request'];self.assertEqual(req['goal_constraints'][0]['joint_constraints'][2]['position'],-3.1532)
  self.assertAlmostEqual(np.arctan2(np.sin(-3.1532),np.cos(-3.1532)),3.1299853071795862)
 def test_discovery_contract_in_noetic_child(self):
  code=r'''
import json,os,tempfile,shutil
from pathlib import Path
from v2.adapters.display_g import DEFAULT_RUN,digest
from v2.adapters.run_discovery import discover_runs
from v2.adapters.trajectory_viewer import Session
with tempfile.TemporaryDirectory() as tmp:
 root=Path(tmp)/'runs';root.mkdir();registry=Path(tmp)/'registry';registry.mkdir()
 def clone(name):
  dest=root/name;dest.mkdir();m=json.loads((DEFAULT_RUN/'manifest.json').read_text())
  for key in m['artifacts']:
   p=DEFAULT_RUN/key
   if p.is_file():
    target=dest/key;target.parent.mkdir(parents=True,exist_ok=True);
    try:os.link(p,target)
    except OSError:shutil.copyfile(p,target)
  index=dest/'g_index.json';data=json.loads(index.read_text());index.unlink()
  for item in data:
   item['source']=item['source'].replace(str(DEFAULT_RUN),str(dest));item['g_dir']=item['g_dir'].replace(str(DEFAULT_RUN),str(dest))
  index.write_text(json.dumps(data));m['artifacts']['g_index.json']['sha256']=digest(index);m['parent_run_id']=None;m['run_id']=name
  (dest/'manifest.json').write_text(json.dumps(m));return dest
 a=clone('run_alpha');found,errors=discover_runs(root);assert len(found)==2,(found,errors)
 assert len(Session(Path(tmp)/'no_manual_registry',lambda message:None,runs_dir=root).available)==2
 sent=[];s=Session(registry,sent.append,runs_dir=root);assert len(s.available)==2
 b=clone('run_beta');assert len(s.available)==2;s.command('refresh');assert len(s.available)==4
 assert len({name.split('/')[0] for name in s.available})==2
 s.command('play run_alpha/archived_5_to_1_joint_base/G');assert len(sent)==1
 msg=sent[0];assert len(msg.trajectory[0].joint_trajectory.points)==len(msg.trajectory[0].multi_dof_joint_trajectory.points)==1001
 # Register the same source without copying it; duplicate gets an alias, not a second row.
 from types import SimpleNamespace
 from v2.adapters.trajectory_viewer import register
 register(SimpleNamespace(registry=str(registry),name='alias',run_dir=str(a),task='5to1',algorithm=None))
 s.command('refresh');assert len(s.available)==4 and 'alias' in s.aliases
 s.command('play alias');assert len(sent)==2
 missing=a/'g_archived_5_to_1_joint_base/trajectory.csv';missing.unlink()
 found,errors=discover_runs(root);assert len(found)==3 and errors
 damaged=b/'g_archived_1_to_2_turn_arm/trajectory.csv';original=damaged.read_bytes();damaged.unlink();damaged.write_bytes(original+b'\n')
 found,errors=discover_runs(root);assert len(found)==2 and any('HASH' in e for _,e in errors)
print('DISCOVERY_FIXTURES_PASS')
'''
  r=subprocess.run(['bash','--noprofile','--norc','-c','source v2/scripts/env_noetic.sh; exec python3 -B -c "$1"','_',code],cwd=PACKAGE,capture_output=True,text=True,timeout=60)
  self.assertEqual(r.returncode,0,r.stdout+r.stderr);self.assertIn('DISCOVERY_FIXTURES_PASS',r.stdout)
