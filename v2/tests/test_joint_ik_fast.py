"""Focused regression for local FK and posterior-only early acceptance."""
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from scipy.spatial.transform import Rotation
from v2.core.kdl_fk import LocalFK
from v2.scripts.joint_ik import JointIK

ROOT=Path(__file__).resolve().parents[2]

def test_kdl_matches_archived_moveit_fk_for_five():
 data=json.loads((ROOT.parents[2]/'research_management/evidence/IK_FIVE_GEOMETRIC_20261110/results.json').read_text())
 fk=LocalFK((ROOT/'config/gazebo_combined_robot.urdf').read_text())
 for row in data['results']:
  p,r=fk.pose(row['reference_q']);target=row['target'];assert np.linalg.norm(p-target['xyz_m'])<1e-6
  assert np.linalg.norm((Rotation.from_quat(target['quaternion_xyzw']).inv()*r).as_rotvec())<1e-6

def test_early_candidate_requires_posterior_scene_pass():
 ik=JointIK.__new__(JointIK);ik.p=np.zeros(3);ik.rot=Rotation.from_quat([0,0,0,1]);ik.radius=2.;ik.mount=[0,0,0];ik.lo=np.full(9,-1.);ik.hi=np.full(9,1.)
 ik.local_fk=SimpleNamespace(pose=lambda q:(np.zeros(3),Rotation.from_quat([0,0,0,1])))
 checks=[]
 def check(q):
  checks.append(q.copy());return {'ik_feasible':len(checks)>1,'scene_valid':len(checks)>1}
 ik.check=check
 def ls(fun,q,**kwargs):
  # A rejected finite-difference-like residual call must not trigger acceptance.
  fun(q+1e-6);return SimpleNamespace(x=q.copy(),nfev=2,success=False,status=0,message='budget')
 with patch('v2.scripts.joint_ik.least_squares',side_effect=ls):result=ik.solve(np.zeros(9),1)
 assert len(checks)==2 and result['ik_feasible'] and result['termination_reason']=='TASK_FEASIBLE_POSTERIOR_PASS'
 assert result['solver_success'] is False
