"""Compare independent profile preparation to stored P01 diagnostic matrices."""
from pathlib import Path
import numpy as np
import yaml
from v2.scripts.stage_v2 import load_parent,limits_for
from v2.geometry.tm_rho_feedback_prepare import prepare_candidate

def test_p01_seed1_profile_and_frozen_mode_sequence():
    root=Path(__file__).resolve().parents[1]
    parent,path=load_parent(root/'outputs/V2-20261028-224435-single_goal_05-RRTstar')
    parent['experiment_limit_profile']=yaml.safe_load((root/'configs/v2_workspace_single_goal_9d_va_only.yaml').read_text())['constraint_profile']
    prepared=prepare_candidate(path,limits_for(parent),1/16,1)
    assert [m['kind'] for m in prepared['modes']]==['MOVE','ADJUST','MOVE','ADJUST']
    assert [m['source_end'] for m in prepared['modes']]==[223,224,317,351]
    evidence=Path('/home/robot/digitaltwin/research_management/evidence/R5_TM_P01_RHO_DIAGNOSTIC')
    for i,n in ((1,'20'),(3,'22'),(5,'24')):
        saved=np.load(evidence/('matrix_'+n+'.npz'))
        np.testing.assert_allclose(prepared['profiles'][i][:,:1],saved['theta_controls'],atol=1e-8,rtol=0)

def test_p01_all_three_move_matrices_match_stored_diagnostic():
    from v2.geometry.tm_rho_feedback_prepare import make_problem
    root=Path(__file__).resolve().parents[1]
    parent,path=load_parent(root/'outputs/V2-20261028-224435-single_goal_05-RRTstar')
    parent['experiment_limit_profile']=yaml.safe_load((root/'configs/v2_workspace_single_goal_9d_va_only.yaml').read_text())['constraint_profile']
    prepared=prepare_candidate(path,limits_for(parent),1/16,1); problem=make_problem(prepared)
    assert [m['segment'] for m in problem['moves']]==[1,3,5]
    ev=Path('/home/robot/digitaltwin/research_management/evidence/R5_TM_P01_RHO_DIAGNOSTIC')
    for move, name in zip(problem['moves'],('20','22','24')):
        z=np.load(ev/('matrix_'+name+'.npz'))
        np.testing.assert_allclose(move['A'],z['A'],atol=1e-12,rtol=0)
        np.testing.assert_allclose(move['ref'],z['ref'],atol=1e-12,rtol=0)
        np.testing.assert_allclose(move['J'],np.vstack([z['eq'][:2],z['eq'][4:]]),atol=1e-12,rtol=0)
        np.testing.assert_allclose(move['d'],np.r_[z['rhs'][:2],z['rhs'][4:]],atol=1e-10,rtol=0)
