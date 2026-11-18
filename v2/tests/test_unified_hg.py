"""Fixed-P and analytic contract-v2 tests; no ROS node or new planning."""
import json
from pathlib import Path
import numpy as np
from v2.core.math_tools import QuinticSpline
from v2.geometry.unified_chart import ExplicitChart,nine_dimensional_progress
from v2.geometry.unified_block import build_unified,thaw_v2
from v2.geometry.frozen import thaw as thaw_v1
from v2.geometry.phase_segmenter import parameterize_geometry
from v2.scripts.stage_v2 import load_parent,limits_for
from v2.timing.unified_g import parameterize_block,sample_block,validate_v2

ROOT=Path(__file__).resolve().parents[1]/'outputs/V2-20261025-224610-single_goal_05-RRTstar'


def test_shared_progress_keeps_pivot_and_arm():
    q=np.zeros((3,9));q[1,2]=.3;q[1,3]=.2;q[2,2]=.5;q[2,3]=.4
    z,s,keep=nine_dimensional_progress(q)
    assert len(z)==3 and np.array_equal(keep,[0,1,2]) and np.all(np.diff(s)>0)


def test_stationary_theta_arm_analytic_and_no_xy_drift():
    c=ExplicitChart(0.,1.,np.array([2.,-3.]),QuinticSpline(np.zeros((6,1))),
                    QuinticSpline(np.array([[0.],[0.],[0.],[1.],[1.],[1.]])),
                    QuinticSpline(np.tile(np.linspace(0,1,6)[:,None],(1,6))),True)
    x=np.linspace(0,1,31);j=c.evaluate_global(x)
    assert np.max(abs(j.q[:,:2]-[2.,-3.]))==0
    assert np.max(abs(j.qs[:,2]))>0 and np.max(abs(j.qs[:,3:]))>0
    assert np.max(abs(j.qs[:,:2]))==0 and np.max(abs(j.qss[:,:2]))==0
    assert np.array_equal(ExplicitChart.thaw(c.freeze()).evaluate_global(x).q,j.q)


def test_fixed_parent_unified_roundtrip_and_g():
    parent,path=load_parent(ROOT);limits=limits_for(parent)
    block,decision=build_unified(path,limits)
    assert decision['stationary_source_index']>=300
    frozen=block.freeze(path,decision,parent['scene'])
    restored=thaw_v2(frozen)['blocks'][0]
    assert len(thaw_v1(frozen)['blocks'])==1  # dispatcher; legacy v1 stays unchanged
    x=np.unique(np.r_[np.linspace(0,1,501),[c.s0 for c in block.charts],[c.s1 for c in block.charts]])
    for key in ('q','qs','qss','qsss'):
        assert np.array_equal(getattr(block.evaluate(x),key),getattr(restored.evaluate(x),key))
    assert np.max(abs(block.start-path.q[0]))<1e-6
    assert np.max(abs(block.end-path.q[-1]))<1e-6
    result=parameterize_block(restored,path,limits)
    assert result['validation'].numerical_status.value=='PASS'
    assert len(restored.charts)==3 and len(result['output']['time_s'])>1000
    assert result['spin_scope_samples']>0
    saved,u=sample_block(restored,path,result['duration_s'],result['uniform_n'])
    assert validate_v2(saved,limits,restored,u).numerical_status.value=='PASS'


def test_old_contract_decoder_unchanged():
    record=json.loads((ROOT/'geometry.json').read_text())
    phases=thaw_v1(record)['phases']
    assert len(phases)==3
    assert [p.phase_type.value for p in phases]==['TRANSLATE','SPIN','ARM_ONLY']
    parent,path=load_parent(ROOT)
    old=parameterize_geometry(path,limits_for(parent),dict(phases=phases,decisions=[]))
    archive=np.loadtxt(ROOT/'G/trajectory.csv',delimiter=',',skiprows=1)
    reconstructed=np.column_stack([old['output'][key] for key in
                                   ('time_s','q','velocity','acceleration','jerk')])
    assert np.array_equal(archive,reconstructed)


def test_historical_heading_budget_parent_has_a_new_h_candidate():
    source=ROOT.parent/'V2-20261025-003230-single_goal_05-RRTstar'
    parent,path=load_parent(source)
    block,decision=build_unified(path,limits_for(parent))
    assert decision['candidate_id'].startswith('tail_')
    assert len(block.charts)==3
    assert np.max(abs(block.end-path.q[-1]))<1e-6
