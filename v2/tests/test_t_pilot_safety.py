"""Fault injection at the piecewise-G numerical gate; no robot execution."""
from dataclasses import replace
import numpy as np
import pytest
from v2.tests.support import record, limits
from v2.validation.numerical import validate
from v2.scripts.tm_feedback_piecewise_run import nonintegration_failures

@pytest.mark.parametrize('kind,code', [('velocity','LIMIT_ARM_VELOCITY'),('acceleration','LIMIT_ARM_ACCELERATION'),('lateral','LATERAL_RESIDUAL'),('backward','BACKWARD_MOTION'),('q_from_v','INCONSISTENT_Q_FROM_V')])
def test_piecewise_gate_rejects_injected_nonintegration_errors(kind,code):
 r=record(); q=r.q.copy();v=r.velocity.copy();a=r.acceleration.copy()
 if kind=='velocity':v[:,3]=5.
 if kind=='acceleration':a[:,3]=5.
 if kind=='lateral':q[:,2]=np.pi/2
 if kind=='backward':q[:,2]=np.pi
 if kind=='q_from_v':v[:]=0.
 r=replace(r,q=q,velocity=v,acceleration=a)
 result=validate(r,limits(),r.q[0],r.q[-1]); assert code in nonintegration_failures(result.failure_codes),result.failure_codes

def test_objective_weight_and_derivative():
 from v2.timing.t_pilot import weight,weight_d
 x=np.array([.02,.2,.5,.8,.98]);h=1e-6
 np.testing.assert_allclose((weight(x+h)-weight(x-h))/(2*h),weight_d(x),atol=1e-8)
 assert weight(np.array([0.,1.])).tolist()==[0.,0.]

def test_b_spline_factor_derivative():
 from v2.timing.t_pilot import matrices
 k=np.r_[[0.]*4,[.2,.5,.8],[1.]*4];x=np.array([.1,.3,.7,.9]);h=1e-6
 A,D=matrices(x,k);plus,_=matrices(x+h,k);minus,_=matrices(x-h,k)
 np.testing.assert_allclose(D,(plus-minus)/(2*h),atol=1e-7)
