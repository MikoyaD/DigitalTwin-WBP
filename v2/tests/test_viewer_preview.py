"""Preview is independent from validation state; full initial state stays required."""
import subprocess
from pathlib import Path

from v2.adapters.run_discovery import preview_status


def test_preview_statuses():
    assert preview_status({'status':'NUMERICAL_PASS_COLLISION_PENDING'}, {}) == 'COLLISION_PENDING'
    assert preview_status({'status':'FAIL'}, {'status':'PASS'}) == 'NUMERICAL_FAIL'
    assert preview_status({}, {}) == 'UNVALIDATED'


def test_real_saved_preview_and_direct_file():
    pkg=Path(__file__).resolve().parents[2]
    code='''
from pathlib import Path
from v2.adapters.run_discovery import load_preview_csv
from v2.adapters.trajectory_viewer import Session
from v2.adapters.display_g import TOPIC
paths=['V2-20261104-194613-single_goal_05_TM_PIECEWISE-RRTstar/G','V2-20261104-PHG1-P02-ENDTOEND/g1/recommended','V2-20261025-224610-single_goal_05-RRTstar/G']
sent=[];s=Session(Path('/tmp/nonexistent_v2_preview_registry'),sent.append)
for name,want in zip(paths,['COLLISION_PENDING','VERIFIED','VERIFIED']):
 p=(Path('v2/outputs')/name/'trajectory.csv').resolve();m,d=load_preview_csv(p)
 assert m['validation_status']==want
 assert s.command('play-file '+str(p)) and len(sent)>0
 msg=sent[-1];j=msg.trajectory[0].joint_trajectory;md=msg.trajectory[0].multi_dof_joint_trajectory
 assert len(j.points)==len(md.points)==len(d.q)
 assert j.joint_names==['joint1','joint2','joint3','joint4','joint5','joint6']
 assert md.joint_names==['world_car'] and j.points[-1].time_from_start==md.points[-1].time_from_start
 assert set(['wheel_left_joint','wheel_right_joint']).issubset(msg.trajectory_start.joint_state.name)
s.command('replay');assert len(sent)==4
assert TOPIC=='/v2/display_trajectory'
print('PREVIEW_REAL_SAVED_PASS')
'''
    r=subprocess.run(['bash','--noprofile','--norc','-c','source v2/scripts/env_noetic.sh >/dev/null; exec python3 -B -c "$1"','_',code],cwd=pkg,capture_output=True,text=True,timeout=90)
    assert r.returncode==0,r.stdout+r.stderr
    assert 'PREVIEW_REAL_SAVED_PASS' in r.stdout
