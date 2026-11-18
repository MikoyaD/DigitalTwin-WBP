"""Focused checks for feedback primitives; full feedback is not yet implemented."""
from pathlib import Path
import numpy as np
import pytest
from v2.geometry.tm_rho_feedback import integral_jacobians, diagnostic_initialize, exact_rho_projection
from v2.core.math_tools import QuinticSpline
from v2.geometry.tm_builder import _rho_matrix

D = Path('/home/robot/digitaltwin/research_management/evidence/R5_TM_P01_RHO_DIAGNOSTIC')

def fixture(n):
    z=np.load(D/('matrix_%s.npz'%n))
    E=z['eq']; b=z['rhs']; J=np.vstack([E[:2],E[4:]]);d=np.r_[b[:2],b[4:]]
    return z,J,d

def test_integrals_and_joint_jacobians_against_directional_difference():
    z,_,_=fixture('24'); h=0.8996129550211912-0.8808245315
    theta=QuinticSpline(z['theta_controls'],z['theta_knots'])
    c=np.linspace(0.1,2,14);rho=QuinticSpline(c[:,None]);x=np.array([0.,.17,.47,1.])
    p,A,K=integral_jacobians(x,theta,rho,h)
    np.testing.assert_allclose(A,_rho_matrix(x,theta,14,h),atol=1e-14,rtol=0)
    rng=np.random.RandomState(19); dc=rng.randn(14); dt=rng.randn(18)
    for eps in (1e-4,1e-5):
        pos=integral_jacobians(x,QuinticSpline(theta.controls+eps*dt[:,None],theta.knots),QuinticSpline((c+eps*dc)[:,None]),h)[0]
        neg=integral_jacobians(x,QuinticSpline(theta.controls-eps*dt[:,None],theta.knots),QuinticSpline((c-eps*dc)[:,None]),h)[0]
        analytic=np.einsum('idn,n->id',A,dc)+np.einsum('idm,m->id',K,dt)
        np.testing.assert_allclose((pos-neg)/(2*eps),analytic,atol=2e-10,rtol=1e-6)

def test_tail_initialization_retains_jets_and_nonnegative():
    z,J,d=fixture('24'); r=diagnostic_initialize(z['A'],z['ref'],z['p1']-z['p0'],J,d,14.)
    assert r['status']=='DIAGNOSTIC_INIT'; assert r['jet_error'] < 1e-6
    assert r['min_rho'] >= -1e-9
    assert r['closure_m'] > 0 or r['tube_excess_m'] > 0

def test_tail_c_conflict_and_middle_exact_projection():
    for name in ('20','22'):
        z,J,d=fixture(name);r=exact_rho_projection(z['A'],z['ref'],z['p1']-z['p0'],J,d)
        assert r['status']=='PROJECTED', (name,r)
    z,J,d=fixture('24'); r=exact_rho_projection(z['A'],z['ref'],z['p1']-z['p0'],J,d)
    assert r['status']=='NUMERICALLY_INFEASIBLE', r

def test_invalid_input_rejected():
    with pytest.raises(ValueError):
        integral_jacobians(np.array([1.1]),QuinticSpline(np.zeros((18,1))),QuinticSpline(np.zeros((14,1))),1.)

def test_short_interval_global_jet_residual_not_hidden():
    from v2.geometry.tm_rho_feedback import global_jet_residual
    J=np.zeros((2,14));J[0,0]=1.;J[1,1]=1.
    c=np.zeros(14);c[:2]=[1e-8,1e-8]
    np.testing.assert_allclose(global_jet_residual(J,np.zeros(2),c,1e-3,[0,1]),[1e-8,1e-5])
    with pytest.raises(ValueError):global_jet_residual(J,np.zeros(2),c,0,[0,1])

def test_true_step_versus_linearization_and_acceptance():
    from v2.geometry.tm_rho_feedback import nonlinear_step_check,accepts_diagnostic_step
    c=np.ones(14);b=np.zeros(18); x=np.array([0.,.5,1.]); h=.2
    p,A,K=integral_jacobians(x,QuinticSpline(b[:,None]),QuinticSpline(c[:,None]),h)
    move=dict(profile=0,segment=0,x=x,h=h,p0=np.zeros(2),p1=np.array([.2,0.]),ref=p)
    result=nonlinear_step_check([move],[b],[c],[np.full(18,.05)],[np.full(14,.5)],1.)
    assert result['status']=='DIAGNOSTIC_STEP'
    assert result['moves'][0]['linearization_error_m']>1e-4
    assert not accepts_diagnostic_step(result,True)
    assert not accepts_diagnostic_step(dict(predicted_decrease=1.,actual_decrease=1.),False)
    assert accepts_diagnostic_step(dict(predicted_decrease=1.,actual_decrease=.1),True)
