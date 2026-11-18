"""Focused TM tests; synthetic paths are not robot experimental results."""
import unittest
from pathlib import Path
from dataclasses import replace
import numpy as np
import yaml
from v2.geometry.tm_builder import build
from v2.geometry.tm_frozen import freeze,thaw
from v2.timing.tm_g import parameterize
from v2.scripts.stage_v2 import load_parent,limits_for
from v2.validation.numerical import validate

class TMTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        root=Path(__file__).resolve().parents[1]
        src=root/'outputs/V2-20261028-224435-single_goal_05-RRTstar'
        if not src.exists():raise unittest.SkipTest('fixed parent fixture unavailable')
        cls.parent,cls.path=load_parent(src)
        with (root/'configs/v2_workspace_single_goal_9d_va_only.yaml').open() as f:cfg=yaml.safe_load(f)
        cls.parent['experiment_limit_profile']=cfg['constraint_profile'];cls.limits=limits_for(cls.parent)
    def run_path(self,q):
        path=replace(self.path,q=q,actual_time_s=np.arange(len(q),dtype=float),requested_start=q[0],requested_goal=q[-1])
        block,decision=build(path,self.limits);frozen=freeze(block,path,decision,self.parent,self.limits);rebuilt=thaw(frozen)
        u=np.linspace(0,1,55)
        for key in ('q','qs','qss','qsss'):
            np.testing.assert_array_equal(getattr(block.evaluate(u),key),getattr(rebuilt.evaluate(u),key))
        out=parameterize(rebuilt,path,self.limits,decision['progress'])
        self.assertEqual(out['validation'].numerical_status.value,'PASS')
        return path,block,decision,out
    def test_pure_move(self):
        q=np.zeros((4,9));q[:,0]=np.linspace(0,1,4)
        _,_,d,_=self.run_path(q);self.assertEqual([m['kind'] for m in d['modes']],['MOVE'])
    def test_pure_adjust(self):
        q=np.zeros((4,9));q[:,2]=np.linspace(0,.5,4);q[:,3]=np.linspace(0,.3,4)
        _,_,d,o=self.run_path(q);self.assertEqual([m['kind'] for m in d['modes']],['ADJUST'])
        np.testing.assert_allclose(o['record'].q[:,:2],0,atol=0)
    def test_adjust_move(self):
        q=np.zeros((4,9));q[:,0]=[0,0,.5,1];q[:,3]=[0,.2,.3,.4]
        _,_,d,_=self.run_path(q);self.assertEqual([m['kind'] for m in d['modes']],['ADJUST','MOVE'])
    def test_many_mode_changes(self):
        q=np.zeros((6,9));q[:,0]=[0,.3,.3,.7,.7,1];q[:,3]=[0,.1,.2,.3,.4,.5]
        _,_,d,o=self.run_path(q)
        self.assertEqual([m['kind'] for m in d['modes']],['MOVE','ADJUST','MOVE','ADJUST','MOVE'])
        self.assertTrue(all(max(s['q_v_a_error'])<1e-6 for s in o['seams']))
    def test_move_adjust_move_adjust(self):
        q=np.zeros((5,9));q[:,0]=[0,.4,.4,1,1];q[:,3]=[0,.1,.2,.3,.4]
        _,_,d,o=self.run_path(q)
        self.assertEqual([m['kind'] for m in d['modes']],['MOVE','ADJUST','MOVE','ADJUST'])
        self.assertTrue(all(max(s['q_v_a_error'])<1e-6 for s in o['seams']))
    def test_move_adjust_move(self):
        q=np.zeros((4,9));q[:,0]=[0,.5,.5,1];q[:,3]=[0,.1,.3,.4]
        path,block,d,o=self.run_path(q)
        self.assertEqual([m['kind'] for m in d['modes']],['MOVE','ADJUST','MOVE'])
        self.assertTrue(all(s['whole_body_speed']>1e-4 for s in o['seams']))
        self.assertTrue(all(max(s['q_v_a_error'])<1e-6 for s in o['seams']))
        # Perturb only v_y; independent validation must detect sideways motion.
        r=o['record'];v=r.velocity.copy();v[len(v)//3,1]=.1
        bad=replace(r,velocity=v);res=validate(bad,self.limits,block.start,block.end)
        self.assertIn('LATERAL_RESIDUAL',res.failure_codes)
