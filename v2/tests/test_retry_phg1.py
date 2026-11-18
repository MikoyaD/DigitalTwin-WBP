import json,sys,types
from pathlib import Path
from unittest.mock import patch
from v2.scripts import retry_phg1

def setup(tmp_path, outcomes, ik_fail=False):
 root=tmp_path/'V2-test';root.mkdir();calls=[]
 def planning(config,task,command,out):
  calls.append('P');out.mkdir();index=len(calls)
  if outcomes[index-1]=='P':
   (out/'error.log').write_text('planning failed');raise ValueError('P_FAILED')
  (out/'parent_path.json').write_text(json.dumps({'scene':{'scene_sha256':'scene'},'path':{'parent_path_hash':str(index)}}));return out
 def run(args):
  work=Path(args[args.index('--out')+1]);fail=outcomes[len(calls)-1];record={'status':'PASS' if fail=='PASS' else 'COLLISION_NOT_PASS' if fail=='collision' else 'FAIL','error':{'H':'H_FAILED','G0':'G0_NUMERICAL_FAILURE','G1':'G1_NUMERICAL_FAILURE'}.get(fail,'')}
  (work/'result.json').write_text(json.dumps(record))
  if fail=='PASS':
   for n in ('H','G0','g1/recommended'):(work/n).mkdir(parents=True,exist_ok=True)
   (work/'g1/recommended/trajectory.csv').write_text('ok')
  return 0 if fail=='PASS' else 1
 class FakeIK:
  def __init__(self,*a):pass
  def target(self):return {'position':[0,0,0]}
  def solve(self,start,n):return {'ik_feasible':False,'q':[0]*9}
 fake=types.ModuleType('v2.scripts.joint_ik');fake.JointIK=FakeIK
 task={'id':'unit','start_q':[0]*9,'goal_q':[0]*9}
 if ik_fail:task['pose_goal']={'position':[0,0,0]}
 with patch.object(retry_phg1.stage_v2,'new_dir',return_value=root),patch.object(retry_phg1.stage_v2,'planning',side_effect=planning),patch.object(retry_phg1.stage_v2,'context',return_value={'scene_sha256':'scene'}),patch.object(retry_phg1.phg1_run,'main',side_effect=run),patch.dict(sys.modules,{'v2.scripts.joint_ik':fake}):
  retry_phg1.run({},task,[])
 return root,json.loads((root/'summary.json').read_text()),calls

def test_p_failure_then_success(tmp_path):
 root,s,c=setup(tmp_path,['P','PASS']);assert c==['P','P'];assert s['failures']['P']==1 and s['success_attempt']==2 and s['total_attempts']==2;assert (root/'attempt_P_01/failure.json').is_file();assert (root/s['final_trajectory']).is_file()

def test_h_g_collision_replan_and_never_publish(tmp_path):
 root,s,c=setup(tmp_path,['H','G0','collision']);assert len(c)==3 and s['status']=='FAIL' and s['final_trajectory'] is None;assert all(s['failures'][x]==1 for x in ('H','G0','collision'));assert not (root/'g1/recommended/trajectory.csv').exists()

def test_ik_fails_three_times_no_planning(tmp_path):
 root,s,c=setup(tmp_path,[],True);assert not c and s['failures']['IK']==3 and s['total_attempts']==3;assert (root/'attempt_IK_01/failure.json').is_file();assert s['final_trajectory'] is None

def test_hidden_failed_work_is_not_discovered(tmp_path,monkeypatch):
 from v2.adapters import run_discovery
 root=tmp_path/'outputs';folder=root/'V2-demo';hidden=folder/'.attempt_work/g1/recommended';hidden.mkdir(parents=True);(hidden/'trajectory.csv').write_text('fake')
 visible=folder/'g1/recommended';visible.mkdir(parents=True);(visible/'trajectory.csv').write_text('fake')
 def fake(csv):return ({'name':str(csv.relative_to(root))},None)
 monkeypatch.setattr(run_discovery,'load_preview_csv',fake)
 found,errors=run_discovery.discover_minimal(root)
 assert len(found)==1 and '.attempt_work' not in next(iter(found)) and not errors
