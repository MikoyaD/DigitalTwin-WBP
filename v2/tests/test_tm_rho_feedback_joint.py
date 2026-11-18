"""Joint step is diagnostic only and cannot issue a success trajectory."""
import numpy as np
from v2.geometry.tm_rho_feedback_joint import propose

def test_two_level_joint_proposal_respects_theta_jet_and_rho_bounds():
    n=8;B=np.tile(np.eye(18)[0],(n,1)); theta=np.zeros(18)
    profile=dict(beta=theta,initial=theta,basis=B,reference=np.zeros(n))
    A=np.zeros((n,2,14));A[:,0,:]=np.linspace(0,.05,n)[:,None]
    K=np.zeros((n,2,18));P=np.zeros((n,2));ref=np.zeros((n,2));end=np.zeros(2)
    move=dict(profile=0,controls=np.ones(14),initial=np.ones(14),scale=1.,
              J=np.eye(14)[:1],d=np.ones(1),P=P,A=A,K=K,reference=ref,endpoint=end)
    E=np.eye(18)[:1]; result=propose([profile],[move],E,np.zeros(1))
    assert result['status']=='DIAGNOSTIC_PROPOSAL'
    assert abs(result['delta_theta'][0][0])<1e-7
    assert abs(result['delta_rho'][0][0])<1e-7
    assert np.min(1+result['delta_rho'][0])>=-1e-9
    assert np.max(abs(result['delta_theta'][0]))<=.05+1e-8
    assert np.max(abs(result['delta_rho'][0]))<=.5+1e-8
