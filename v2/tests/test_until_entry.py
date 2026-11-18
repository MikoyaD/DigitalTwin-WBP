"""Entry orchestration only: no planner, ROS master or geometry is started."""
import json
from pathlib import Path
from unittest.mock import patch
import pytest
from v2.scripts import stage_v2, phg1_run

@pytest.mark.parametrize('target', ['p', 'h', 'g0', 'g1'])
def test_until_chains_saved_parent(target, tmp_path):
    parent=tmp_path/'V2-NEW'
    if target=='g1':
        from v2.scripts import retry_phg1
        parent.mkdir();(parent/'summary.json').write_text('{"status":"PASS"}')
        with patch.object(retry_phg1,'run',return_value=parent) as runner:
            assert stage_v2.main(['--until',target])==0
        runner.assert_called_once();return
    with patch.object(stage_v2,'new_dir',return_value=parent), patch.object(stage_v2,'planning',return_value=parent/'P' if target=='g1' else parent) as plan, \
         patch.object(phg1_run,'main',return_value=0) as chain:
        assert stage_v2.main(['--until',target])==0
    plan.assert_called_once()
    if target=='p':
        chain.assert_not_called()
    else:
        assert chain.call_args.args[0]==['--input',str(parent),'--out',str(parent)+'-PHG1','--until',target]

@pytest.mark.parametrize('target,status', [('h','H_PASS'),('g0','G0_NUMERICAL_PASS_COLLISION_PENDING')])
def test_early_stops_do_not_run_g1(target,status,tmp_path):
    out=tmp_path/'out'
    (tmp_path/'parent_path.json').write_text('{}');(tmp_path/'parent_path.csv').write_text('x\n0\n')
    g0=({'status':'NUMERICAL_PASS_COLLISION_PENDING','duration_s':1.0},)
    with patch.object(phg1_run.tm_feedback_hg,'h',return_value=tmp_path/'h'), \
         patch.object(phg1_run.tm_feedback_piecewise_run,'run',return_value=(tmp_path/'g0',g0[0])) as make_g0, \
         patch.object(phg1_run.g1_general,'stage_all') as g1:
        assert phg1_run.main(['--input',str(tmp_path),'--out',str(out),'--until',target])==0
    assert json.loads((out/'result.json').read_text())['status']==status
    assert (out/'P/parent_path.json').is_file()
    assert json.loads((out/'result.json').read_text())['parent']=='P'
    assert make_g0.call_count==(target=='g0')
    g1.assert_not_called()

def test_original_stage_all_keeps_legacy_calls(tmp_path):
    with patch.object(stage_v2,'planning',return_value=tmp_path) as p, \
         patch.object(stage_v2,'geometry') as h, patch.object(stage_v2,'g') as g, \
         patch.object(stage_v2,'independent_validate') as v:
        assert stage_v2.main(['--stage','all'])==0
    assert all(f.call_count==1 for f in (p,h,g,v))
