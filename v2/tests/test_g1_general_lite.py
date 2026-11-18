import csv
import numpy as np

from v2.scripts.g1_general import high_jerk
from v2.timing.g1_general import matrices


def test_constant_density_and_derivatives():
    b, d, e = matrices(np.array([0., .2, .5, .8, 1.]), np.array([0., .5, 1.]))
    np.testing.assert_allclose(b @ np.ones(3), 1.)
    np.testing.assert_allclose(d @ np.ones(3), 0., atol=1e-14)
    np.testing.assert_allclose(e @ np.ones(3), 0., atol=1e-14)


def test_high_jerk_signed_zero_crossing(tmp_path):
    path = tmp_path / 'trajectory.csv'
    with path.open('w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['column'] * 37)
        a = np.zeros(37); a[28] = 6.
        b = np.zeros(37); b[0] = 1.; b[28] = -6.
        w.writerow(a); w.writerow(b)
    got = high_jerk(path, {'seams': []})[0]
    assert got['clusters'] == 2
    np.testing.assert_allclose(got['duration_s'], 1 / 3)
