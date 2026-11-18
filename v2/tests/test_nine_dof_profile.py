import unittest
from dataclasses import replace
from pathlib import Path
import numpy as np
import yaml
from v2.core.contracts import PhaseType, Status, PathJet
from v2.core.limits import load_limits
from v2.core.math_tools import spin_jet
from v2.tests.support import PACKAGE, research_limits, record, circle_jet
from v2.timing.feasible_profile import ratio
from v2.validation.numerical import validate


def get_limits(profile=None):
    c=yaml.safe_load((PACKAGE/'v2/configs/v2_workspace_single_goal_9d_va_only.yaml').read_text())
    return load_limits(PACKAGE/'config/gazebo_combined_robot.urdf',
        PACKAGE/'config/joint_limits.yaml',PACKAGE/'config/combined_robot.srdf',research_limits(),
        velocity_scaling=1.,acceleration_scaling=1.,profile=profile)


def profile():
    return yaml.safe_load((PACKAGE/'v2/configs/v2_workspace_single_goal_9d_va_only.yaml').read_text())['constraint_profile']


def diagonal(s):
    q=np.zeros((len(s),9));q[:,0]=q[:,1]=(16./15.)*s;q[:,2]=np.pi/4
    qs=np.zeros_like(q);qs[:,0]=qs[:,1]=16./15.
    return PathJet(q,qs,np.zeros_like(q),np.zeros_like(q))


class NineDofProfileTests(unittest.TestCase):
    def check(self,r,l): return validate(r,l,r.q[0],r.q[-1])

    def test_profile_and_legacy_contract(self):
        old,new=get_limits(),get_limits(profile())
        np.testing.assert_allclose(old.arm_acceleration.value,.5)
        np.testing.assert_allclose(new.arm_acceleration.value,1.)
        np.testing.assert_allclose(new.xy_component_acceleration.value,1.)
        self.assertEqual(float(new.angular_acceleration.value),1.)
        self.assertEqual(new.metadata['constraint_profile']['identity'],'V2_NINE_DOF_VA_ONLY')
        self.assertEqual(old.metadata.get('constraint_profile'),None)

    def test_every_nine_dimension_velocity_and_acceleration_fail(self):
        l=get_limits(profile());base=record()
        for field,limit,code in [('velocity',.501,'VELOCITY'),('acceleration',1.001,'ACCELERATION')]:
            for i in range(9):
                arr=getattr(base,field).copy();arr[len(arr)//2,i]=limit;r=replace(base,**{field:arr})
                self.assertGreater(ratio(r,l),1.)
                expected=('XY_COMPONENT_' if i<2 else 'ANGULAR_' if i==2 else 'ARM_')+code
                self.assertIn('LIMIT_'+expected,self.check(r,l).failure_codes)

    def test_diagonal_norm_and_wheels_disabled_not_accepted_by_old(self):
        r=record(duration=5.,jet_function=diagonal);old,new=get_limits(),get_limits(profile())
        self.assertGreater(ratio(r,old),1.);self.assertLess(ratio(r,new),1.)
        before,after=self.check(r,old),self.check(r,new)
        self.assertIn('LIMIT_XY_SPEED',before.failure_codes)
        self.assertEqual(after.numerical_status,Status.PASS,after.failure_codes)
        self.assertEqual(after.checks['xy_speed']['status'],'DISABLED')
        self.assertEqual(after.checks['wheel_velocity']['status'],'DISABLED')

    def test_wheel_alone_and_switch_reenabled(self):
        r=record(duration=9.4,jet_function=circle_jet);p=profile();l=get_limits(p)
        c=self.check(r,l);self.assertEqual(c.numerical_status,Status.PASS,c.failure_codes)
        self.assertLess(c.checks['xy_speed']['detail']['max_ratio'],1.)
        self.assertGreater(c.checks['wheel_velocity']['detail']['max_ratio'],1.)
        self.assertIn('LIMIT_WHEEL_VELOCITY',self.check(r,get_limits()).failure_codes)
        p['enabled']['wheel_velocity']=True;on=get_limits(p)
        self.assertIn('LIMIT_WHEEL_VELOCITY',self.check(r,on).failure_codes)
        self.assertGreater(ratio(r,on),ratio(r,l))

    def test_spin_jerk_and_nonholonomic_still_enforced(self):
        r=record(kind=PhaseType.SPIN,duration=1.5,jet_function=lambda s:spin_jet(s,np.zeros(9),.35))
        old,new=get_limits(),get_limits(profile());c=self.check(r,new)
        self.assertEqual(c.numerical_status,Status.PASS,c.failure_codes)
        self.assertEqual(c.checks['spin_jerk']['status'],'DISABLED')
        self.assertIn('LIMIT_SPIN_JERK',self.check(r,old).failure_codes)
        self.assertGreater(ratio(r,old),ratio(r,new))
        base=record();q=base.q.copy();q[:,2]=np.pi/2;bad=replace(base,q=q)
        self.assertIn('LATERAL_RESIDUAL',self.check(bad,new).failure_codes)

    def test_new_acceleration_cap_is_one_not_point_five(self):
        r=record(kind=PhaseType.SPIN,duration=1.5,jet_function=lambda s:spin_jet(s,np.zeros(9),.35))
        self.assertGreater(np.max(np.abs(r.acceleration[:,2])),.5)
        self.assertLess(np.max(np.abs(r.acceleration[:,2])),1.)
        self.assertEqual(self.check(r,get_limits(profile())).numerical_status,Status.PASS)

    def test_small_real_exceedance_not_hidden_by_legacy_relative_tolerance(self):
        l=get_limits(profile());base=record()
        for field,value,code in [('velocity',.500001,'LIMIT_XY_COMPONENT_VELOCITY'),
                                 ('acceleration',1.000001,'LIMIT_XY_COMPONENT_ACCELERATION')]:
            arr=getattr(base,field).copy();arr[len(arr)//2,0]=value
            self.assertIn(code,self.check(replace(base,**{field:arr}),l).failure_codes)

    def test_unified_stationary_scope_honors_switch(self):
        import json
        from v2.scripts.stage_v2 import load_parent
        from v2.geometry.unified_block import thaw_v2
        from v2.timing.unified_g import sample_block,validate_v2
        root=PACKAGE/'v2/outputs/V2-20261026-214202-single_goal_05_UHG-RRTstar'
        _,path=load_parent(root);block=thaw_v2(json.loads((root/'geometry.json').read_text()))['blocks'][0]
        r,u=sample_block(block,path,73.27969652156213,2001)
        check=validate_v2(r,get_limits(profile()),block,u)
        self.assertEqual(check.checks['physical_stationary_theta_jerk']['status'],'DISABLED')
        enabled=profile();enabled['enabled']['spin_jerk']=True
        check=validate_v2(r,get_limits(enabled),block,u)
        self.assertEqual(check.checks['physical_stationary_theta_jerk']['status'],'PASS')

    def test_endpoints_and_joint_position_remain_required(self):
        l=get_limits(profile());r=record();goal=r.q[-1].copy();goal[0]+=.1
        self.assertIn('ENDPOINT_GOAL',validate(r,l,r.q[0],goal).failure_codes)
        q=r.q.copy();q[len(q)//2,3]=4.;bad=replace(r,q=q)
        self.assertIn('POSITION_LIMIT',self.check(bad,l).failure_codes)

    @unittest.skipUnless((PACKAGE/'v2/outputs/V2-20261028-224453-single_goal_05-RRTstar/G/result.json').exists(),
                         'recorded P02 integration fixture not installed')
    def test_recorded_global_offset_duration_cancellation_recovery(self):
        import json
        from v2.scripts.stage_v2 import load_parent,limits_for,recover_profile_records
        from v2.geometry.frozen import thaw
        from v2.timing.feasible_profile import sample,concatenate
        root=PACKAGE/'v2/outputs/V2-20261028-224453-single_goal_05-RRTstar'
        parent,path=load_parent(root);result=json.loads((root/'G/result.json').read_text())
        phases=thaw(json.loads((root/'geometry.json').read_text()))['phases'];records=[];offset=0.
        for phase,info in zip(phases,result['phases']):
            r=sample(phase,path,info['duration_s'],info['samples'],offset);records.append(r);offset=r.time_s[-1]
        saved=np.loadtxt(root/'G/trajectory.csv',delimiter=',',skiprows=1)
        def matrix(data):return np.column_stack([data[k] for k in ('time_s','q','velocity','acceleration','jerk')])
        self.assertGreater(float(np.max(np.abs(saved-matrix(concatenate(records))))),1e-10)
        recovered=recover_profile_records(path,phases,result['phases'],limits_for(parent))
        self.assertTrue(np.allclose(saved,matrix(recovered),atol=1e-10,rtol=0))

if __name__=='__main__':unittest.main()
