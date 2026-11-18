import unittest
import math
import numpy as np
from numpy.testing import assert_allclose
from v2.core.math_tools import *
from v2.core.contracts import ContractError, PhasePath, PhaseType
from v2.tests.support import line_jet, circle_jet, SCHEMA, record


class MathTests(unittest.TestCase):
    def test_basis_partition_and_all_derivatives(self):
        s = np.linspace(0, 1, 101)
        for order in range(5):
            b = quintic_basis(s, 11, order)
            assert_allclose(b.sum(axis=1), np.ones(len(s)) if order == 0 else 0, atol=2e-9)

    def test_polynomial_fifth_power_exact_through_fourth(self):
        s = np.linspace(0, 1, 81)
        c = np.zeros((6, 2)); c[-1, 0] = 1
        spline = QuinticSpline(c)
        for d in range(5):
            expected = math.factorial(5)/math.factorial(5-d)*s**(5-d)
            assert_allclose(spline.evaluate(s, d)[:, 0], expected, atol=2e-12)

    def test_general_spline_derivative_fd_convergence(self):
        spline = QuinticSpline(np.random.RandomState(17).normal(size=(10, 2)))
        errors = []
        for h in (1e-4, 5e-5):
            s = np.array([.13, .37, .68, .89])
            errors.append([np.max(np.abs((spline.evaluate(s+h,d-1)-spline.evaluate(s-h,d-1))/(2*h)-spline.evaluate(s,d))) for d in range(1,5)])
        self.assertTrue(np.all(np.array(errors[1][:3]) < np.array(errors[0][:3])*.3))
        # Third derivative is quadratic on each span: centred differentiation
        # to fourth order is already exact up to roundoff, so no h^2 ratio.
        self.assertLess(max(errors[0][3], errors[1][3]), 1e-7)

    def test_spline_fourth_derivative_continuity(self):
        spline = QuinticSpline(np.random.RandomState(1).normal(size=(9, 2)))
        for knot in np.unique(spline.knots)[1:-1]:
            assert_allclose(spline.evaluate([knot-1e-10],4), spline.evaluate([knot+1e-10],4), atol=2e-4)

    def test_regular_straight_spline(self):
        spline = QuinticSpline(np.column_stack((np.linspace(0,1,6), np.zeros(6))))
        r = xy_regularity(spline)
        self.assertEqual(r.status, Status.PASS)
        self.assertAlmostEqual(r.minimum_speed, 1.)

    def test_interior_zero_tangent_not_just_endpoints(self):
        # x=(s-.5)^2 in degree-five Bernstein form.
        c = np.array([(.25-i/5+(i*(i-1)/20)) for i in range(6)])
        spline = QuinticSpline(np.column_stack((c, np.zeros(6))))
        r = xy_regularity(spline)
        self.assertEqual(r.status, Status.FAIL)
        self.assertAlmostEqual(r.parameter_at_minimum, .5, places=7)

    def test_stationary_spline_rejected(self):
        self.assertEqual(xy_regularity(QuinticSpline(np.zeros((6, 2)))).status, Status.FAIL)

    def test_duplicate_full_states_and_arm_only_parameter(self):
        q = np.zeros((4, 9)); q[2:, 3] = [.3, .6]
        p = whole_body_parameter(q)
        assert_allclose(p.s, [0, .5, 1]); assert_allclose(p.retained_indices, [0,2,3])
        self.assertEqual(p.xy_polyline_length, 0)

    def test_closed_xy_loop_is_not_stationary(self):
        q = np.zeros((3,9)); q[1,0] = 1
        p = whole_body_parameter(q)
        assert_allclose(p.s, [0,.5,1]); self.assertEqual(p.xy_polyline_length, 2.)

    def test_theta_only_requires_split(self):
        q = np.zeros((2,9)); q[1,2] = .3
        with self.assertRaisesRegex(ContractError, 'NEEDS_PHASE_SPLIT'): whole_body_parameter(q)

    def test_all_duplicates_need_hold(self):
        with self.assertRaisesRegex(ContractError, 'DEGENERATE_PATH'): whole_body_parameter(np.zeros((3,9)))

    def test_arm_not_unwrapped(self):
        q = np.zeros((2,9)); q[:,3] = [3.,-3.]
        self.assertEqual(whole_body_parameter(q).retained_indices.tolist(), [0,1])
        self.assertEqual(q[1,3]-q[0,3], -6.)

    def test_heading_parabola_exact_through_third(self):
        s = np.linspace(0,1,101); zero = np.zeros((len(s),2))
        p1 = np.column_stack((np.ones(len(s)),2*s))
        p2 = np.tile([0.,2.],(len(s),1))
        h = heading_jet(s,p1,p2,zero,zero,anchor=0.)
        D = 1+4*s*s
        for actual, expected in zip(h, [np.arctan(2*s),2/D,-16*s/D**2,(-16+192*s*s)/D**3]):
            assert_allclose(actual, expected, atol=1e-12)

    def test_heading_general_fourth_geometry_derivative(self):
        def jet(s):
            p1=np.column_stack((1+4*s**3,2*s+3*s**2))
            p2=np.column_stack((12*s*s,2+6*s))
            p3=np.column_stack((24*s,np.full_like(s,6)))
            p4=np.column_stack((np.full_like(s,24),np.zeros_like(s)))
            return heading_jet(s,p1,p2,p3,p4,anchor=0.)
        s=np.array([.17,.33,.71]); h=1e-5
        for d in (1,2,3):
            assert_allclose((jet(s+h)[d-1]-jet(s-h)[d-1])/(2*h),jet(s)[d],atol=2e-7,rtol=2e-7)

    def test_circle_heading(self):
        s=np.linspace(0,1,101); jet=circle_jet(s)
        assert_allclose(jet.q[:,2], .4+1.2*s+np.pi/2,atol=1e-13)
        assert_allclose(jet.qs[:,2],1.2,atol=1e-13)
        assert_allclose(jet.qss[:,2],0,atol=1e-13)
        assert_allclose(jet.qsss[:,2],0,atol=1e-12)

    def test_heading_crosses_pi(self):
        s=np.linspace(0,1,11); angle=3.+.4*s
        p1=np.column_stack((np.cos(angle),np.sin(angle)))
        p2=.4*np.column_stack((-np.sin(angle),np.cos(angle)))
        p3=-.4**2*p1; p4=-.4**2*p2
        h=heading_jet(s,p1,p2,p3,p4,anchor=3.)
        assert_allclose(h[0],angle,atol=1e-13); self.assertGreater(h[0][-1],np.pi)

    def test_heading_zero_tangent_rejected(self):
        z=np.zeros((2,2))
        with self.assertRaisesRegex(ContractError,'DEGENERATE_TANGENT'): heading_jet([0,1],z,z,z,z,anchor=0.)

    def test_heading_coarse_grid_rejected(self):
        z=np.zeros((2,2))
        with self.assertRaisesRegex(ContractError,'HEADING_GRID_TOO_COARSE'):
            heading_jet([0,1],[[1,0],[-1,.01]],z,z,z,anchor=0.)

    def test_quintic_boundary_zero_velocity_acceleration(self):
        s,v,a,j=quintic_time_jet([0,2],2)
        assert_allclose(s,[0,1]); assert_allclose(v,0); assert_allclose(a,0)
        assert_allclose(j,[7.5,7.5])  # C2, not a promise of zero endpoint jerk.

    def test_chain_rule_nontrivial_exact(self):
        t=np.linspace(.1,1,51); s=t*t
        q=np.tile((s**3)[:,None],(1,9))
        jet=PathJet(q,np.tile((3*s*s)[:,None],(1,9)),np.tile((6*s)[:,None],(1,9)),np.full((len(s),9),6.))
        v,a,j=chain_rule(jet,2*t,np.full(len(t),2.),np.zeros(len(t)))
        for actual, expected in [(v,6*t**5),(a,30*t**4),(j,120*t**3)]:
            assert_allclose(actual,np.tile(expected[:,None],(1,9)),atol=2e-12)

    def test_time_scaling_all_derivatives(self):
        r=record(); ts,v,a,j=scale_time(r.time_s,r.velocity,r.acceleration,r.jerk,2.5)
        r2=record(duration=25.)
        assert_allclose(ts,r2.time_s); assert_allclose(v,r2.velocity,atol=1e-14)
        assert_allclose(a,r2.acceleration,atol=1e-14); assert_allclose(j,r2.jerk,atol=1e-14)
        assert_allclose(r.q,r2.q,atol=1e-14)

    def test_phase_interface_all_four_types(self):
        s=np.linspace(0,1,21)
        builders={PhaseType.TRANSLATE:line_jet,PhaseType.SPIN:lambda s:spin_jet(s,np.zeros(9),.7),
                  PhaseType.ARM_ONLY:lambda s:arm_only_jet([1,2,.3],[np.tile(s[:,None],(1,6)),np.ones((len(s),6)),np.zeros((len(s),6)),np.zeros((len(s),6))]),
                  PhaseType.HOLD:lambda s:hold_jet(np.ones(9),len(s))}
        for kind,build in builders.items():
            j=build(s); phase=PhasePath(kind.value,kind,SCHEMA,j.q[0],j.q[-1],build,'synthetic analytic')
            assert_allclose(phase.evaluate(s).q,j.q)

    def test_differential_wheels_circle_and_spin(self):
        r=record(jet_function=circle_jet,duration=20.)
        k=differential_kinematics(r.q,r.velocity,r.acceleration,.06,.3935)
        assert_allclose(k['lateral'],0,atol=1e-15)
        assert_allclose(k['wheel_velocity'][:,1]-k['wheel_velocity'][:,0],.3935*r.velocity[:,2]/.06,atol=1e-14)
        spin=record(PhaseType.SPIN)
        k=differential_kinematics(spin.q,spin.velocity,spin.acceleration,.06,.3935)
        assert_allclose(k['wheel_velocity'][:,0],-k['wheel_velocity'][:,1])

    def test_invalid_math_inputs(self):
        for action in [lambda:quintic_basis([1.1],6),lambda:quintic_basis([.5],5),
                       lambda:quintic_time_jet([0],0),lambda:whole_body_parameter(np.full((2,9),np.nan)),
                       lambda:quintic_basis([.5],6,5)]:
            with self.assertRaises(ContractError): action()

    def test_quintic_near_endpoints_stays_monotone(self):
        u=np.r_[0.,np.logspace(-16,-3,30), .5, 1-np.logspace(-3,-16,30),1.]
        s,v,a,j=quintic_time_jet(u,1.)
        self.assertTrue(np.all((s >= 0) & (s <= 1)))
        self.assertTrue(np.all(v >= 0)); self.assertTrue(np.all(np.diff(s) >= 0))
        assert_allclose(s+quintic_time_jet(1-u,1.)[0],1.,atol=1e-15)

    def test_endpoint_time_scaling_preserves_nonzero_origin(self):
        r=record(offset=7.)
        t,*_=scale_time(r.time_s,r.velocity,r.acceleration,r.jerk,.5)
        self.assertEqual(t[0],7.);self.assertEqual(t[-1],12.)
