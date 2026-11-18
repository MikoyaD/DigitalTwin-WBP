import unittest
import numpy as np
from numpy.testing import assert_allclose
from v2.core.contracts import *
from v2.tests.support import SCHEMA,HASH,limits
from v2.geometry.phase_segmenter import build_g
from v2.geometry.spline_builder import fit,LiftedSpline
from v2.timing.feasible_profile import sample,ratio
from v2.adapters.planning_scene_checker import sample_phases

def path(q):return PathRecord('synthetic-II','synthetic',HASH,'synthetic',SCHEMA,q,None,q[0],q[-1],{'source':'synthetic'})
class PhaseIITests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.l=limits()
    def check(self,q,**kw):
        out=build_g(path(q),self.l,**kw)
        self.assertTrue(all(v.numerical_status==Status.PASS for v in out['validation']))
        self.assertEqual(out['collision_status'],Status.UNKNOWN)
        assert_allclose(out['output']['q'][0],q[0],atol=1e-6)
        e=out['output']['q'][-1]-q[-1];e[2]=np.arctan2(np.sin(e[2]),np.cos(e[2]));assert_allclose(e,0,atol=1e-6)
        self.assertTrue(np.all(np.diff(out['output']['time_s'])>0));return out
    def test_line(self):
        q=np.zeros((5,9));q[:,0]=np.linspace(0,1,5);self.assertAlmostEqual(self.check(q)['xy_length_m'],1,places=5)
    def test_arc(self):
        a=np.linspace(0,1,12);q=np.zeros((12,9));q[:,0]=np.sin(a);q[:,1]=1-np.cos(a);q[:,2]=a;self.check(q)
    def test_arm(self):
        q=np.zeros((5,9));q[:,3]=np.linspace(0,.4,5);self.assertEqual(self.check(q)['xy_length_m'],0)
    def test_spin(self):
        q=np.zeros((2,9));q[-1,2]=1;self.check(q)
    def test_hold(self):self.check(np.zeros((3,9)),hold_duration=.1)
    def test_no_invented_wait(self):
        with self.assertRaisesRegex(ContractError,'HOLD_DURATION_REQUIRED'):self.check(np.zeros((2,9)))
    def test_repeated_xy_arm(self):
        q=np.zeros((4,9));q[:,0]=[0,.5,.5,1];q[:,3]=[0,0,.2,.2];self.check(q)
    def test_closed_loop(self):
        a=np.linspace(0,2*np.pi,25);q=np.zeros((25,9));q[:,0]=np.sin(a);q[:,1]=1-np.cos(a);q[:,2]=a;q[-1,:2]=0
        self.assertGreater(self.check(q)['xy_length_m'],5)
    def test_pi_crossing(self):
        a=np.linspace(3,3.4,10);q=np.zeros((10,9));q[:,0]=np.sin(a);q[:,1]=-np.cos(a);q[:,2]=a;self.check(q)
    def test_multiphase(self):
        q=np.zeros((4,9));q[:,0]=[0,0,1,1];q[:,2]=[1,0,0,0];q[:,3]=[0,0,.1,.4];self.check(q)
    def test_synchronous(self):
        q=np.zeros((6,9));q[:,0]=np.linspace(0,1,6);q[:,3:]=q[:,0,None]*.5;self.check(q)
    def test_duplicate(self):
        q=np.zeros((4,9));q[:,0]=[0,0,1,1];self.check(q)
    def test_infeasible(self):
        q=np.zeros((2,9));q[-1,3]=4
        with self.assertRaises(ContractError):self.check(q)
    def test_ambiguous(self):
        q=np.zeros((2,9));q[-1,2:4]=.3
        out=self.check(q)
        self.assertEqual([p.phase_type for p in out['phases']],[PhaseType.SPIN,PhaseType.ARM_ONLY])
        self.assertEqual(out['decisions'][0]['mode'],'SPIN_THEN_ARM')
    def test_opposed_tangent(self):
        q=np.zeros((2,9));q[-1,0]=1;spline=fit(q,self.l,[[1,0],[-1,0]])
        with self.assertRaises(ContractError):LiftedSpline(spline,0)
    def test_time_scaling(self):
        q=np.zeros((2,9));q[-1,0]=1;ph=self.check(q)['phases'][0]
        a=sample(ph,path(q),10,501);b=sample(ph,path(q),20,501)
        assert_allclose(a.q,b.q);assert_allclose(a.velocity/2,b.velocity);assert_allclose(a.acceleration/4,b.acceleration);assert_allclose(a.jerk/8,b.jerk)
        self.assertGreater(ratio(sample(ph,path(q),.1,501),self.l),1)
    def test_collision_sampling(self):
        q=np.zeros((3,9));q[:,0]=[0,1,1];q[:,2]=[1,0,0];q[-1,3]=.2
        states,loc=sample_phases(self.check(q)['phases']);self.assertEqual(len(states),len(loc));self.assertGreater(len(states),100)
    def test_global_heading_queries_not_query_local(self):
        a=np.linspace(0,2*np.pi,25);q=np.zeros((25,9));q[:,0]=np.sin(a);q[:,1]=1-np.cos(a);q[:,2]=a;q[-1,:2]=0
        out=self.check(q);ph=next(p for p in out['phases'] if p.phase_type==PhaseType.TRANSLATE)
        ordered=ph.evaluate(np.linspace(0,1,101));self.assertGreater(ordered.q[-1,2]-ordered.q[0,2],5)
        s=np.array([1.,.1,.9,0.,.1]);v=ph.evaluate(s).q
        assert_allclose(v,ordered.q[[100,10,90,0,10]],atol=1e-10)
    def test_explicit_large_spin_ambiguous(self):
        q=np.zeros((2,9));q[-1,2]=2*np.pi
        with self.assertRaisesRegex(ContractError,'AMBIGUOUS_WINDING'):self.check(q)
