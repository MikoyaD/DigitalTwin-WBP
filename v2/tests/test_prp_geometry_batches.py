"""The timeout retry must query the same geometry, never accept UNKNOWN."""
from unittest.mock import patch
import numpy as np
from v2.scripts.prp_from_r import check_geometry_batches


def test_resource_timeout_retries_same_candidate_in_smaller_batches():
    calls = []
    class Checker:
        def check(self, phases, include_geometry, geometry_samples):
            q, loc = geometry_samples
            calls.append(list(loc))
            first = len(calls) == 1
            return {'status':'UNKNOWN' if first else 'PASS',
                    'reason':'COLLISION_TIME_BUDGET' if first else 'ok',
                    'checked_states':100 if first else len(q), 'failures':[],
                    'scene_snapshot_hex':'78','robot_description':'u',
                    'robot_description_semantic':'s'}
    states = np.zeros((9001, 9)); locations = list(range(9001))
    with patch('v2.adapters.planning_scene_checker.sample_phases', return_value=(states, locations)), \
         patch('v2.scripts.prp_from_r.PlanningSceneChecker', Checker):
        result = check_geometry_batches([object()], b'x', 'u', 's')
    assert [len(x) for x in calls] == [8000, 4000, 4000, 1001]
    assert calls[1] + calls[2] + calls[3] == locations
    assert result['status'] == 'PASS' and result['checked_states'] == 9101


def test_scene_mismatch_is_never_pass():
    class Checker:
        def check(self, phases, include_geometry, geometry_samples):
            return {'status':'PASS','reason':'ok','checked_states':len(geometry_samples[0]),
                    'failures':[],'scene_snapshot_hex':'79',
                    'robot_description':'u','robot_description_semantic':'s'}
    with patch('v2.adapters.planning_scene_checker.sample_phases', return_value=(np.zeros((1,9)), [0])), \
         patch('v2.scripts.prp_from_r.PlanningSceneChecker', Checker):
        result = check_geometry_batches([object()], b'x', 'u', 's')
    assert result['status'] == 'UNKNOWN'


def test_geometry_sampling_failure_is_unknown():
    with patch('v2.adapters.planning_scene_checker.sample_phases', side_effect=ValueError('sampling failed')):
        result = check_geometry_batches([object()], b'x', 'u', 's')
    assert result['status'] == 'UNKNOWN'
    assert 'sampling failed' in result['reason']
    assert result['checked_states'] == 0
