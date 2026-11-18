import json
from pathlib import Path
import pytest
from v2.geometry.tm_rho_feedback_frozen import thaw
from v2.validation.tm_feedback_piecewise import integrate

@pytest.fixture(scope='module')
def block():
 p=Path(__file__).resolve().parents[1]/'outputs/V2-20261030-000922-single_goal_05_TM_RHO_FEEDBACK-RRTstar/geometry.json'
 if not p.exists(): pytest.skip('frozen P01 absent')
 return thaw(json.loads(p.read_text()))

def test_real_short_adjust_and_jerk_seams(block):
 r=integrate(block,140.22781714821352)
 assert r['status']=='NUMERIC_INTEGRATION_PASS',r
 assert r['spans']>20 and len(r['seams'])>=4
 assert max(r['global_v'])<1e-3 and max(r['global_a'])<1e-2

def test_budget_does_not_pass(block):
 r=integrate(block,140.22781714821352,max_spans=1)
 assert r['status']=='UNKNOWN' and r['reason']=='INTEGRATION_BUDGET'

def test_wrong_analytic_jerk_is_not_accepted(block):
 from dataclasses import replace
 class BadChart:
  def __init__(self, source):
   self.source=source
   for k in ('s0','s1','rho','theta','arm','profile_interval'):
    if hasattr(source,k):setattr(self,k,getattr(source,k))
  def evaluate_global(self,u):
   jet=self.source.evaluate_global(u)
   return replace(jet,qsss=jet.qsss+1e8)
 class BadBlock:
  def __init__(self,c):self.charts=[c]
 bad=BadBlock(BadChart(block.charts[2]))
 result=integrate(bad,140.22781714821352)
 assert result['status']!='NUMERIC_INTEGRATION_PASS'

def test_synthetic_c2_but_jerk_jump():
 import numpy as np
 from types import SimpleNamespace
 from v2.core.contracts import PathJet
 class Chart:
  def __init__(self,left,right,active):self.s0=left;self.s1=right;self.active=active;self.spline=SimpleNamespace(knots=np.array([0.,.25,.75,1.]))
  def evaluate_global(self,s):
   s=np.asarray(s);z=np.maximum(s-.5,0) if self.active else np.zeros_like(s)
   fields=[np.zeros((len(s),9)) for _ in range(4)]
   if self.active:
    fields[0][:,3]=z**3;fields[1][:,3]=3*z**2;fields[2][:,3]=6*z;fields[3][:,3]=6
   return PathJet(*fields)
 class Block: charts=[Chart(0,.5,False),Chart(.5,1,True)]
 r=integrate(Block(),10.)
 assert r['status']=='NUMERIC_INTEGRATION_PASS'
 assert abs(r['seams'][0]['jerk_left'][3]-r['seams'][0]['jerk_right'][3])>0

def test_nonconvergence_is_unknown(block):
 r=integrate(block,140.22781714821352,convergence=-1.)
 assert r['status']=='UNKNOWN' and r['reason']=='QUADRATURE_NOT_CONVERGED'

def test_local_errors_can_accumulate_to_global_failure(monkeypatch):
 import numpy as np
 from types import SimpleNamespace
 import v2.validation.tm_feedback_piecewise as module
 class Chart:
  def __init__(self,a,b):self.s0=a;self.s1=b;self.spline=SimpleNamespace(knots=np.array([0.,1.]))
  def evaluate_global(self,u):return SimpleNamespace(q=np.zeros((len(u),9)))
 class Block:charts=[Chart(0.,.5),Chart(.5,1.)]
 def derivatives(jet,v,a,j):
  z=np.zeros((len(v),9)); jerk=z.copy();jerk[:,3]=.0012
  return z,z,jerk
 monkeypatch.setattr(module,'chain_rule',derivatives);monkeypatch.setattr(module,'seams',lambda b,t:[])
 r=module.integrate(Block(),10.)
 assert r['status']=='FAIL'
 assert max(r['local_a'])<.01 and max(r['global_a'])>.01

def test_piecewise_replacement_never_drops_limits_nonholonomic_or_position_integral():
 from v2.scripts.tm_feedback_piecewise_run import nonintegration_failures
 codes=['INCONSISTENT_V_FROM_A','INCONSISTENT_A_FROM_J','INCONSISTENT_Q_FROM_V','LIMIT_ARM_VELOCITY','LATERAL_RESIDUAL','BACKWARD_MOTION']
 assert nonintegration_failures(codes)==codes[2:]

def test_actual_output_sampling_budget_is_not_silently_relaxed():
 from v2.scripts.tm_feedback_piecewise_run import dense_sample_count
 assert dense_sample_count(100.,1001)>1001
 with pytest.raises(ValueError,match='TM_TIME_BUDGET'):dense_sample_count(500.,1001)
