import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
from v2.core.json_io import compatible,write_exclusive
from v2.scripts.stage_v2 import commit_g


class JsonOutputTests(unittest.TestCase):
    def test_arrays_scalars_and_nested_values_roundtrip_without_rounding(self):
        values=np.array([[np.nextafter(1.,2.),-np.pi],[np.finfo(float).tiny,1e200]])
        original={'array':values,'nested':[{'i':np.int64(-7),'u':np.uint64(2**64-1),
                     'f':np.float32(.1),'b':np.bool_(True),'scalar':np.array(2.5)}]}
        with tempfile.TemporaryDirectory() as temp:
            dest=Path(temp)/'result.json';write_exclusive(dest,original)
            result=json.loads(dest.read_text())
            self.assertTrue(np.array_equal(values,np.asarray(result['array'])))
            self.assertEqual(result['nested'][0]['u'],2**64-1)
            self.assertEqual(result['nested'][0]['f'],float(np.float32(.1)))
            self.assertIs(result['nested'][0]['b'],True)

    def test_unsupported_or_nonfinite_never_creates_partial_file(self):
        with tempfile.TemporaryDirectory() as temp:
            folder=Path(temp);dest=folder/'result.json'
            for bad in (object(),np.array([np.nan]),np.complex128(1+2j),np.longdouble(1.)):
                with self.assertRaises((TypeError,ValueError)):
                    write_exclusive(dest,{'good':[1,2,3],'late':{'bad':bad}})
                self.assertFalse(dest.exists())
                self.assertEqual(list(folder.iterdir()),[])

    def test_io_failure_cleans_temporary_and_preserves_existing_target(self):
        with tempfile.TemporaryDirectory() as temp:
            folder=Path(temp);dest=folder/'result.json'
            with patch('v2.core.json_io.os.link',side_effect=OSError('injected publish failure')):
                with self.assertRaises(OSError):write_exclusive(dest,{'ok':1})
            self.assertEqual(list(folder.iterdir()),[])
            dest.write_text('original')
            with self.assertRaises(FileExistsError):write_exclusive(dest,{'replacement':2})
            self.assertEqual(dest.read_text(),'original')
            self.assertEqual(list(folder.iterdir()),[dest])

    def test_g_failed_serialization_is_retryable_atomic_four_file_commit(self):
        with tempfile.TemporaryDirectory() as temp:
            folder=Path(temp)/'G';folder.mkdir();(folder/'error.log').write_text('original failure')
            matrix=np.zeros((2,37));matrix[1,0]=1.
            result={'duration_s':np.float64(1.),'phases':[],
                    'limits':{'vector':np.array([.5,.5])},'late':object()}
            with self.assertRaises(TypeError):commit_g(folder,result,matrix,'report')
            self.assertEqual([p.name for p in folder.iterdir()],['error.log'])
            self.assertEqual((folder/'error.log').read_text(),'original failure')
            self.assertEqual([p.name for p in Path(temp).iterdir()],['G'])
            result.pop('late');commit_g(folder,result,matrix,'report')
            self.assertEqual(sorted(p.name for p in folder.iterdir()),
                             ['diagnostics.json','report.md','result.json','trajectory.csv'])
            self.assertEqual(json.loads((folder/'result.json').read_text())['limits']['vector'],[.5,.5])
            with self.assertRaises(RuntimeError):commit_g(folder,result,matrix,'replacement')
            self.assertEqual((folder/'report.md').read_text(),'report')

    def test_diagnostics_failure_does_not_publish_partial_g(self):
        with tempfile.TemporaryDirectory() as temp:
            folder=Path(temp)/'G'
            with patch('v2.scripts.stage_v2.diagnose',side_effect=ValueError('injected diagnostic failure')):
                with self.assertRaises(ValueError):commit_g(folder,{},np.zeros((2,37)),'report')
            self.assertFalse(folder.exists())
            self.assertEqual(list(Path(temp).iterdir()),[])
