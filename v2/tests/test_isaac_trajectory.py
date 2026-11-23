import csv
import pytest
from v2.adapters.trajectory_viewer import NAMES, read_csv
from v2.scripts.isaac_trajectory import load_trajectory


def make_csv(path, times=(0, 1e-10, .2), missing=None):
    fields = ['time_s'] + [prefix + name for prefix in ('q_', 'v_', 'a_') for name in NAMES] + ['direction']
    if missing:
        fields.remove(missing)
    with path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for i, t in enumerate(times):
            row = {field: i + .25 for field in fields}
            row.update(time_s=t)
            if 'direction' in fields: row['direction'] = (-1, 0, 1)[i % 3]
            writer.writerow(row)


def test_read_uneven_motion_and_map(tmp_path):
    path = tmp_path / 'trajectory.csv'
    make_csv(path)
    data = load_trajectory(path, str(tmp_path / 'scene.usd'))
    assert data.time_s == (0, 1e-10, .2)
    assert data.q[1] == data.velocity[1] == data.acceleration[1] == (1.25,) * 9
    assert data.direction == (-1, 0, 1)
    assert data.map_config['isaac_usd_status'] == 'user_selected_unverified'


def test_reader_rejects_missing_and_nonmonotonic(tmp_path):
    path = tmp_path / 'trajectory.csv'
    make_csv(path, missing='direction')
    with pytest.raises(ValueError, match='CSV_REQUIRES_VELOCITY_ACCELERATION_DIRECTION'):
        load_trajectory(path)
    make_csv(path, times=(0, .2, .1))
    with pytest.raises(ValueError, match='TIME_NOT_STRICTLY_INCREASING'):
        load_trajectory(path)


def test_legacy_reader_still_returns_three_values(tmp_path):
    path = tmp_path / 'trajectory.csv'
    make_csv(path, times=(0, .1, .2))
    times, q, columns = read_csv(path)
    assert times == (0, .1, .2) and len(q[0]) == len(columns) == 9
