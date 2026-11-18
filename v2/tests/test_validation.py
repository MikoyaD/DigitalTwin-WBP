import unittest
from dataclasses import replace
import numpy as np
from numpy.testing import assert_allclose
from v2.core.contracts import *
from v2.core.math_tools import quintic_time_jet, chain_rule, differential_kinematics
from v2.validation.numerical import validate, validate_junctions, integration_errors, jerk_metrics
from v2.tests.support import record, limits, circle_jet, line_jet, SCHEMA, HASH


class ValidationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls): cls.limits=limits()

    def evaluate(self,r,**kwargs):
        return validate(r,self.limits,r.q[0],r.q[-1],**kwargs)

    def test_straight_pass_numerical_but_not_collision(self):
        r=record(); result=self.evaluate(r)
        self.assertEqual(result.numerical_status,Status.PASS,result.failure_codes)
        self.assertEqual(result.overall_status,Status.UNKNOWN)
        self.assertAlmostEqual(result.metrics['xy_length_m'],1.,places=8)

    def test_arc_forward_no_slip(self):
        r=record(duration=20.,jet_function=circle_jet)
        result=self.evaluate(r)
        self.assertEqual(result.numerical_status,Status.PASS,result.failure_codes)
        self.assertLess(result.checks['nonholonomic']['detail']['peak'],1e-14)

    def test_arm_only_spin_hold_numerical(self):
        for kind in (PhaseType.ARM_ONLY,PhaseType.SPIN,PhaseType.HOLD):
            r=record(kind); out=self.evaluate(r)
            self.assertEqual(out.numerical_status,Status.PASS,(kind,out.failure_codes))
            self.assertEqual(out.metrics['xy_length_m'],0.)

    def test_limits_inclusive_at_velocity_and_acceleration(self):
        t=np.linspace(0,1,501); q=np.zeros((len(t),9)); v=q.copy(); a=q.copy(); j=q.copy()
        q[:,0]=.25*t*t; v[:,0]=.5*t; a[:,0]=.5
        r=TrajectoryRecord('t','p',PhaseType.TRANSLATE,'analytic',HASH,SCHEMA,t,q,v,a,j,'analytic quadratic')
        out=self.evaluate(r,require_rest=False)
        self.assertEqual(out.numerical_status,Status.PASS,out.failure_codes)
        self.assertEqual(self.evaluate(r).numerical_status,Status.FAIL)
        v=v.copy(); v[-1,0]+=.01
        self.assertIn('LIMIT_XY_SPEED',self.evaluate(replace(r,velocity=v),require_rest=False).failure_codes)

    def test_endpoint_and_arm_position_violations(self):
        r=record(); target=r.q[-1].copy(); target[0]+=.01
        self.assertIn('ENDPOINT_GOAL',validate(r,self.limits,r.q[0],target).failure_codes)
        q=r.q.copy(); q[:,3]=4.
        self.assertIn('POSITION_LIMIT',self.evaluate(replace(r,q=q)).failure_codes)

    def test_angular_and_wheel_limit_violations(self):
        r=record(PhaseType.SPIN,duration=1.); codes=self.evaluate(r).failure_codes
        self.assertIn('LIMIT_ANGULAR_VELOCITY',codes)
        self.assertIn('LIMIT_ANGULAR_ACCELERATION',codes); self.assertIn('LIMIT_SPIN_JERK',codes)
        limited=replace(self.limits,wheel_velocity=Quantity([.01,.01],'rad/s','synthetic','test'))
        self.assertIn('LIMIT_WHEEL_VELOCITY',validate(record(),limited,np.zeros(9),record().q[-1]).failure_codes)

    def test_arm_velocity_acceleration_limits(self):
        codes=self.evaluate(record(PhaseType.ARM_ONLY,duration=.5)).failure_codes
        self.assertIn('LIMIT_ARM_VELOCITY',codes); self.assertIn('LIMIT_ARM_ACCELERATION',codes)

    def test_export_clipping_inconsistency_detected(self):
        r=record(); fake=replace(r,velocity=np.zeros_like(r.velocity),acceleration=np.zeros_like(r.acceleration),jerk=np.zeros_like(r.jerk))
        self.assertIn('INCONSISTENT_Q_FROM_V',self.evaluate(fake).failure_codes)

    def test_wrong_heading_has_lateral_residual(self):
        r=record(); q=r.q.copy(); q[:,2]=np.pi/2
        self.assertIn('LATERAL_RESIDUAL',self.evaluate(replace(r,q=q)).failure_codes)

    def test_backward_reference_rejected(self):
        r=record(); q=r.q.copy(); q[:,2]=np.pi
        self.assertIn('BACKWARD_MOTION',self.evaluate(replace(r,q=q)).failure_codes)

    def test_wheel_acceleration_body_projection_term(self):
        t=np.linspace(0,1,101); q=np.zeros((len(t),9));q[:,2]=t
        v=np.zeros_like(q); v[:,0]=1; v[:,2]=1
        k=differential_kinematics(q,v,np.zeros_like(q),.06,.3935)
        assert_allclose(k['forward_acceleration'],-np.sin(t),atol=1e-14)

    def test_nonuniform_actual_time_integral(self):
        t=10*np.linspace(0,1,501)**1.6
        s,v,a,j=quintic_time_jet(t,10); jet=line_jet(s)
        r=TrajectoryRecord('t','p',PhaseType.TRANSLATE,'analytic',HASH,SCHEMA,t,jet.q,*chain_rule(jet,v,a,j),'analytic')
        self.assertEqual(self.evaluate(r).numerical_status,Status.PASS)
        false=replace(r,time_s=np.linspace(0,10,len(t)))
        self.assertIn('INCONSISTENT_Q_FROM_V',self.evaluate(false).failure_codes)

    def test_integral_error_converges_quadratically(self):
        e1=integration_errors(record(n=101))['q_from_v'][0]
        e2=integration_errors(record(n=201))['q_from_v'][0]
        self.assertLess(e2,e1*.26)

    def test_chain_derivatives_independent_fd_convergence(self):
        errors=[]
        for n in (401,801):
            r=record(n=n,duration=10.,jet_function=circle_jet); q=r.q; h=r.time_s[1]-r.time_s[0]
            derivatives=[]
            for _ in range(3):
                q=np.gradient(q,h,axis=0,edge_order=2); derivatives.append(q)
            errors.append([np.max(np.abs(x[5:-5]-y[5:-5])) for x,y in zip(derivatives,[r.velocity,r.acceleration,r.jerk])])
        self.assertTrue(np.all(np.array(errors[1]) < .35*np.array(errors[0])),errors)

    def test_jerk_groups_exact_constant_integrals(self):
        r=record(); j=np.zeros_like(r.jerk);j[:,0]=3;j[:,1]=4;j[:,2]=2;j[:,3:]=np.arange(1,7)
        m=jerk_metrics(replace(r,jerk=j))
        self.assertEqual(m['xy']['peak'],5.); self.assertAlmostEqual(m['xy']['rms'],5.)
        self.assertAlmostEqual(m['xy']['integral_squared'],250.)
        self.assertEqual(m['theta']['unit'],'rad/s^3'); assert_allclose(m['arm_each']['rms'],np.arange(1,7))

    def test_continuous_junction_and_discontinuous_rejection(self):
        left=record(); right=record(offset=10.)
        q=right.q.copy(); q[:,0]+=1; right=replace(right,q=q,phase_id='second')
        self.assertEqual(validate_junctions([left,right]).numerical_status,Status.PASS)
        q=q.copy(); q[:,2]+=2*np.pi
        self.assertEqual(validate_junctions([left,replace(right,q=q)]).numerical_status,Status.FAIL)
        v=right.velocity.copy();v[0,0]=.01
        self.assertEqual(validate_junctions([left,replace(right,velocity=v)]).numerical_status,Status.FAIL)
        a=right.acceleration.copy();a[0,0]=.01
        self.assertEqual(validate_junctions([left,replace(right,acceleration=a)]).numerical_status,Status.FAIL)

    def test_junction_time_and_identity(self):
        left=record(PhaseType.HOLD);right=record(PhaseType.HOLD,offset=10.)
        self.assertEqual(validate_junctions([left,right]).numerical_status,Status.PASS)
        self.assertEqual(validate_junctions([left,replace(right,time_s=right.time_s+.001)]).numerical_status,Status.FAIL)
        self.assertEqual(validate_junctions([left,replace(right,parent_path_hash='b'*64)]).numerical_status,Status.FAIL)

    def test_optional_wheel_acceleration_and_required_spin_jerk(self):
        l=replace(self.limits,wheel_acceleration=None,spin_jerk=None)
        r=record(); out=validate(r,l,r.q[0],r.q[-1])
        self.assertEqual(out.checks['wheel_acceleration']['status'],'UNKNOWN')
        r=record(PhaseType.SPIN)
        self.assertIn('MISSING_SPIN_JERK',validate(r,l,r.q[0],r.q[-1]).failure_codes)

    def test_hold_derivative_not_silently_ignored(self):
        r=record(PhaseType.HOLD);v=r.velocity.copy();v[:,3]=.01
        self.assertIn('PHASE_SEMANTICS',self.evaluate(replace(r,velocity=v)).failure_codes)

    def test_all_phase_kinds_join_with_complete_state(self):
        hold=record(PhaseType.HOLD,duration=2.)
        spin=record(PhaseType.SPIN,offset=2.)
        move=record(offset=12.)
        q,v,a,j=[x.copy() for x in (move.q,move.velocity,move.acceleration,move.jerk)]
        for values in (q,v,a,j):
            x=values[:,0].copy();values[:,0]=x*np.cos(1.);values[:,1]=x*np.sin(1.)
        q[:,2]=1.
        move=replace(move,q=q,velocity=v,acceleration=a,jerk=j)
        arm=record(PhaseType.ARM_ONLY,offset=22.)
        q=arm.q.copy();q[:,:3]=move.q[-1,:3];arm=replace(arm,q=q)
        for r in (hold,spin,move,arm):
            self.assertEqual(self.evaluate(r).numerical_status,Status.PASS)
        self.assertEqual(validate_junctions([hold,spin,move,arm]).numerical_status,Status.PASS)

    def test_endpoint_orientation_modulo_but_seams_not_modulo(self):
        r=record();goal=r.q[-1].copy();goal[2]+=2*np.pi
        self.assertEqual(validate(r,self.limits,r.q[0],goal).numerical_status,Status.PASS)

    def test_optional_wheel_acceleration_bound_enforced_when_present(self):
        r=record()
        l=replace(self.limits,wheel_acceleration=Quantity([.01,.01],'rad/s^2','synthetic','test'))
        self.assertIn('LIMIT_WHEEL_ACCELERATION',validate(r,l,r.q[0],r.q[-1]).failure_codes)
