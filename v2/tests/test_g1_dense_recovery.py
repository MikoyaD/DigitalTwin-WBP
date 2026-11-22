import json
from pathlib import Path
from unittest.mock import patch
import numpy as np
import pytest
from v2.scripts.g1_dense_recovery import read_numeric_rows


def test_compact_requires_and_verifies_dense_reconstruction(tmp_path):
    dense=np.arange(123,dtype=float).reshape(3,41);dense[:,0]=[0.,1.,2.]
    np.savetxt(tmp_path/'trajectory.csv',dense[[0,2]],delimiter=',',header=','.join('c%d'%i for i in range(41)),comments='')
    geometry=tmp_path/'geometry';geometry.mkdir()
    note={'dense_reconstruction':'g1_fixed_geometry_v1','geometry':str(geometry),'source_rows':3}
    (tmp_path/'trajectory.csv.storage.json').write_text(json.dumps(note))
    with patch('v2.scripts.g1_dense_recovery.recover',return_value=dense):
        assert np.array_equal(read_numeric_rows(tmp_path,geometry),dense)
        altered=dense.copy();altered[2,1]+=1
        with patch('v2.scripts.g1_dense_recovery.recover',return_value=altered):
            with pytest.raises(ValueError,match='NO_LONGER_MATCHES'):read_numeric_rows(tmp_path,geometry)
    with pytest.raises(ValueError,match='GEOMETRY_MISMATCH'):read_numeric_rows(tmp_path,tmp_path/'other')


def test_unrecoverable_display_is_not_silently_checked(tmp_path):
    (tmp_path/'trajectory.csv.storage.json').write_text(json.dumps({'method':'original_dense_rows_50hz'}))
    with pytest.raises(ValueError,match='DENSE_TRAJECTORY_UNAVAILABLE'):read_numeric_rows(tmp_path,tmp_path)
