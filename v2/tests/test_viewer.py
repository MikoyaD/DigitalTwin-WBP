"""Small registry tests, real ROS message tests isolated from numerical core."""
import os
import subprocess
import tempfile
import unittest
from pathlib import Path
from v2.adapters.trajectory_viewer import read_csv, canonical


class ViewerTests(unittest.TestCase):
    def test_csv_fields_time_and_finiteness(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'a.csv';header='time_s,x,y,theta,joint1,joint2,joint3,joint4,joint5,joint6\n'
            for body in ('0,0,0,0,0,0,0,0,0,0\n0,1,0,0,0,0,0,0,0,0\n',
                         '0,nan,0,0,0,0,0,0,0,0\n1,1,0,0,0,0,0,0,0,0\n'):
                p.write_text(header+body)
                with self.assertRaises(ValueError):read_csv(p)
            p.write_text('time_s,x\n0,0\n1,1\n')
            with self.assertRaises(ValueError):read_csv(p)
            p.write_text(header+'2,0,0,0,0,0,0,0,0,0\n3,1,0,0,0,0,0,0,0,0\n')
            self.assertEqual(read_csv(p)[0],(2.,3.))

    def test_missing_and_symlink(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'none'
            with self.assertRaises(FileNotFoundError):canonical(p)
            p.write_text('x');link=Path(tmp)/'link';link.symlink_to(p)
            with self.assertRaises(ValueError):canonical(link)

    def test_registry_registration_refresh_switch_errors_and_shared_message(self):
        code=r'''
import contextlib,io,json,tempfile
from pathlib import Path
from types import SimpleNamespace as Args
from v2.adapters.trajectory_viewer import register,load_entry,scan,Session,REGISTRY
from v2.adapters.display_g import DEFAULT_RUN,digest,TOPIC,load_existing,build_message
source=DEFAULT_RUN/'g_archived_5_to_1_joint_base/trajectory.csv';before=digest(source)
with tempfile.TemporaryDirectory() as tmp:
 root=Path(tmp)/'registry';root.mkdir()
 def reg(name,task):
  register(Args(registry=str(root),name=name,run_dir=str(DEFAULT_RUN),task=task,algorithm=None))
 reg('one','5to1');reg('two','1to2')
 try:reg('one','5to1');raise AssertionError('overwrite accepted')
 except ValueError:pass
 sent=[];session=Session(root,sent.append)
 session.command('play one');session.command('play 2');session.command('replay')
 assert len(sent)==3 and [len(m.trajectory[0].joint_trajectory.points) for m in sent]==[1001,5519,5519]
 assert all(len(m.trajectory)==1 for m in sent)
 for msg in sent:
  j=msg.trajectory[0].joint_trajectory;b=msg.trajectory[0].multi_dof_joint_trajectory
  assert [p.time_from_start for p in j.points]==[p.time_from_start for p in b.points]
 csv=Path(tmp)/'external.csv';csv.write_text('time_s,x,y,theta,joint1,joint2,joint3,joint4,joint5,joint6\n0,0,0,0,0,0,0,0,0,0\n1,1,0,1,0,0,0,0,0,0\n')
 args=Args(registry=str(root),name='external',run_dir=None,csv=str(csv),state_from='one',frame='world',algorithm='T')
 register(args);assert 'external' not in session.available
 session.command('list');assert 'external' in session.available
 meta,data=load_entry(root/'external');assert meta['validation_status']=='VISUALIZATION_ONLY'
 session.command('play external');assert len(sent[-1].trajectory[0].joint_trajectory.points)==2
 assert not sent[-1].trajectory[0].joint_trajectory.points[0].velocities
 args.name='badframe';args.frame='odom'
 try:register(args);raise AssertionError('wrong frame accepted')
 except ValueError:pass
 csv.write_text(csv.read_text()+'2,2,0,0,0,0,0,0,0,0\n')
 try:load_entry(root/'external');raise AssertionError('changed hash accepted')
 except ValueError:pass
 csv.unlink()
 session.command('play external');session.command('play unknown');assert session.command('list')
 assert session.command('quit') is False
 import sys
 from v2.adapters.trajectory_viewer import main
 sys.argv=['viewer','--registry',str(root),'unregister','external']
 assert main()==0 and not (root/'external').exists()
 # Initial-state source and robot description cannot be omitted or invented.
 p=root/'one/metadata.json';meta=json.loads(p.read_text());meta['model_id']='wrong';p.write_text(json.dumps(meta))
 try:load_entry(root/'one');raise AssertionError('wrong model accepted')
 except ValueError:pass
assert digest(source)==before
assert TOPIC=='/v2/display_trajectory'
assert len(build_message(load_existing(DEFAULT_RUN,'5to1')).trajectory)==1
print('REGISTRY_SWITCH_CONTRACT_PASS')
'''
        pkg=Path(__file__).resolve().parents[2];setup=pkg/'v2/scripts/env_noetic.sh'
        r=subprocess.run(['bash','--noprofile','--norc','-c','source "$1"; cd "$2"; exec python3 -B -c "$3"','_',str(setup),str(pkg),code],
                         capture_output=True,text=True,timeout=30,env=os.environ.copy())
        self.assertEqual(r.returncode,0,r.stdout+r.stderr)
        self.assertIn('REGISTRY_SWITCH_CONTRACT_PASS',r.stdout)
