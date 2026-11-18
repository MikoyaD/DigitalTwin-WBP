import unittest
from dataclasses import replace
import numpy as np
from numpy.testing import assert_allclose
from v2.core.contracts import *
from v2.tests.support import SCHEMA, HASH, limits, research_limits, record, line_jet
from v2.validation.collision import CollisionRequest, CollisionResult, UnknownCollisionChecker


class ContractTests(unittest.TestCase):
    def test_path_metadata_and_no_time_invention(self):
        q=np.zeros((2,9)); q[1,0]=1
        p=PathRecord('trial','RRTstar',HASH,'pre_TOTG',SCHEMA,q,None,q[0],q[-1],{'scene':'unknown'})
        self.assertIsNone(p.actual_time_s)
        with self.assertRaises(ValueError): p.q[0,0]=5

    def test_invalid_joint_order(self):
        with self.assertRaisesRegex(ContractError,'JOINT_ORDER'):
            replace(SCHEMA,joint_names=NAMES[:3]+NAMES[3:][::-1])

    def test_wrong_units(self):
        for change in [dict(time_unit='ms'),dict(position_units=('mm',)*9),
                       dict(velocity_units=('deg/s',)*9),dict(jerk_units=('m/s^3',)*9)]:
            with self.assertRaisesRegex(ContractError,'UNITS'): replace(SCHEMA,**change)

    def test_wrong_reference_point(self):
        with self.assertRaisesRegex(ContractError,'REFERENCE_POINT'): replace(SCHEMA,reference_point='tool0')

    def test_streams_require_same_time_grid(self):
        with self.assertRaisesRegex(ContractError,'TIME_GRID_MISMATCH'):
            merge_aligned_streams([0,1],np.zeros((2,3)),[0,.9],np.zeros((2,6)),NAMES[3:])
        merged=merge_aligned_streams([0,1],np.ones((2,3)),[0,1],np.zeros((2,6)),NAMES[3:])
        assert_allclose(merged[:,:3],1); assert_allclose(merged[:,3:],0)

    def test_nan_shape_and_nonincreasing_time(self):
        r=record()
        for change in [dict(time_s=np.zeros(len(r.time_s))),dict(q=np.full_like(r.q,np.nan)),
                       dict(q=np.zeros((len(r.q),8))),dict(velocity=np.zeros((len(r.q),6)))]:
            with self.assertRaises(ContractError): replace(r,**change)

    def test_model_limits_and_conflict(self):
        l=limits()
        assert_allclose(l.arm_velocity.value,.5); assert_allclose(l.arm_acceleration.value,.5)
        assert_allclose(l.arm_lower.value,-3.14); assert_allclose(l.arm_upper.value,3.14)
        self.assertAlmostEqual(l.wheel_radius.value,.06); self.assertAlmostEqual(l.wheel_separation.value,.3935)
        self.assertAlmostEqual(l.metadata['geometric_wheel_separation_m'],.395)
        self.assertTrue(l.metadata['wheel_separation_conflict'])
        self.assertEqual(l.wheel_velocity.kind,'research_limit'); self.assertEqual(l.wheel_radius.kind,'model')
        assert_allclose(l.metadata['axle_centre_xy_m'],[0,0])

    def test_request_scaling_recorded(self):
        l=limits(vs=.5,acs=.25)
        assert_allclose(l.arm_velocity.value,.25); assert_allclose(l.arm_acceleration.value,.125)
        self.assertEqual(l.metadata['velocity_scaling'],.5)

    def test_missing_wheel_parameter_is_error(self):
        params=research_limits(); del params['wheel_velocity']
        with self.assertRaisesRegex(ContractError,'MISSING_LIMIT'): limits(research=params)
        with self.assertRaisesRegex(ContractError,'MISSING_LIMIT'): replace(limits(),wheel_velocity=None)

    def test_limit_wrong_units_or_negative(self):
        for q in [Quantity([.5,.5],'m/s','research_limit','test'),Quantity([-1,2],'rad/s','synthetic','test')]:
            with self.assertRaises(ContractError): replace(limits(),wheel_velocity=q)

    def test_manifest_unknown_scene_and_seed_explicit(self):
        m=TrialManifest('t','RRTstar',HASH,SCHEMA,'post_TOTG',HASH,None,None,'uncontrolled',
                        {'limits':'research_limit'}, {'module':HASH},(),('scene','tf','request'))
        self.assertIsNone(m.scene_hash)
        with self.assertRaises(ContractError): replace(m,seed_status='controlled')
        with self.assertRaises(ContractError): replace(m,missing_metadata=())

    def test_collision_unknown_without_backend(self):
        req=CollisionRequest(np.zeros((2,9)),SCHEMA,None,HASH,'unspecified continuous interpolator')
        self.assertEqual(UnknownCollisionChecker().check(req).status,Status.UNKNOWN)
        with self.assertRaisesRegex(ContractError,'COLLISION_EVIDENCE_MISSING'):
            CollisionResult(Status.PASS,'unsupported assertion',None,0)

    def test_hold_phase_cannot_move(self):
        j=line_jet(np.array([0.,1.]))
        phase=PhasePath('bad',PhaseType.HOLD,SCHEMA,j.q[0],j.q[-1],line_jet,'synthetic')
        with self.assertRaisesRegex(ContractError,'PHASE_SEMANTICS'): phase.evaluate([0,1])

    def test_missing_schema_is_coded_error(self):
        with self.assertRaisesRegex(ContractError,'MISSING_METADATA'): replace(record(),schema=None)
        with self.assertRaisesRegex(ContractError,'MISSING_METADATA'):
            PathRecord('t','RRTstar',HASH,'pre_TOTG',None,np.zeros((2,9)),None,np.zeros(9),np.zeros(9),{})
        with self.assertRaisesRegex(ContractError,'MISSING_METADATA'):
            CollisionRequest(np.zeros((2,9)),None,None,HASH,'linear')
