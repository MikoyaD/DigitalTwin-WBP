import pytest
from v2.geometry.tm_rho_feedback_build import CANDIDATES,finalize

def test_candidate_order_and_bound():
    assert len(CANDIDATES)==8 and len(set(CANDIDATES))==8
    assert CANDIDATES[0]==(1/16,1)

def test_diagnostic_projection_is_never_h():
    with pytest.raises(ValueError,match='FULL_PROJECTION_REQUIRED'):
        finalize(None,None,None,None,None,[dict(status='DIAGNOSTIC_PROPOSAL')])

def test_v4_decoder_never_accepts_v3_or_unknown():
    from v2.geometry.tm_rho_feedback_frozen import thaw
    with pytest.raises(Exception,match='TM_FEEDBACK_CONTRACT'):
        thaw(dict(contract_version=3,algorithm_variant='V2-TM-HG'))

def test_v4_real_result_roundtrip_and_v3_rejection():
    import json
    from pathlib import Path
    from v2.geometry.tm_rho_feedback_frozen import thaw
    from v2.geometry.tm_frozen import thaw as old_thaw
    path=Path(__file__).resolve().parents[1]/'outputs/V2-20261030-000922-single_goal_05_TM_RHO_FEEDBACK-RRTstar/geometry.json'
    if not path.exists():pytest.skip('real run not installed')
    geo=json.loads(path.read_text());block=thaw(geo)
    assert block.start.shape==(9,) and len(block.charts)>1
    with pytest.raises(Exception,match='TM_CONTRACT'):
        old_thaw(geo)
