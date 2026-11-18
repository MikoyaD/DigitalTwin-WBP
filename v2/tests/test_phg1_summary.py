import json
from v2.scripts.phg1_run import print_summary


def test_historical_collision_is_not_pass(capsys):
    from v2.scripts.phg1_run import ROOT
    p = ROOT/'v2/outputs/V2-20261109-163313-single_goal_05-RRTstar-PHG1'
    print_summary(p)
    text = capsys.readouterr().out
    for expected in ('60.545189 s', '36.515260 s', '38.294649 s',
                     'COLLISION_NOT_PASS', '<octomap>', 'car_link', 'chart_0'):
        assert expected in text


def test_missing_collision_and_incomplete_data(tmp_path, capsys):
    print_summary(tmp_path, {'status':'COLLISION_NOT_PASS'})
    text = capsys.readouterr().out
    assert '碰撞验收结果缺失' in text
    assert '最终验收状态： COLLISION_NOT_PASS' in text
    assert '时间 UNKNOWN' in text


def test_check_failed_is_not_reported_as_collision(tmp_path, capsys):
    folder = tmp_path/'g1/recommended'
    folder.mkdir(parents=True)
    (folder/'collision.json').write_text(json.dumps({'status':'UNKNOWN','reason':'service unavailable'}))
    print_summary(tmp_path, {'status':'COLLISION_NOT_PASS'})
    text = capsys.readouterr().out
    assert 'service unavailable' in text
    assert '无接触证据' in text
    assert '检测到碰撞' not in text


def test_pass_reports_actual_collision_evidence(tmp_path, capsys):
    folder = tmp_path/'g1/recommended'
    folder.mkdir(parents=True)
    (folder/'result.json').write_text(json.dumps({'status':'INDEPENDENT_NUMERICAL_PASS','duration_s':12.3}))
    (folder/'independent_export.json').write_text(json.dumps({'status':'PASS'}))
    (folder/'collision.json').write_text(json.dumps({'status':'PASS'}))
    print_summary(tmp_path, {'status':'PASS'})
    text = capsys.readouterr().out
    assert '12.300000 s' in text
    assert '独立复核 PASS；碰撞 PASS' in text
    assert '最终验收状态： PASS' in text

def test_relative_g0_index_after_directory_move(tmp_path, capsys):
    root=tmp_path/'moved_experiment';folder=root/'G0/G';folder.mkdir(parents=True)
    (folder/'result.json').write_text(json.dumps({'status':'NUMERICAL_PASS_COLLISION_PENDING','duration_s':23.0}))
    print_summary(root, {'g0_dir':'G0/G','status':'FAIL'})
    assert '23.000000 s' in capsys.readouterr().out
