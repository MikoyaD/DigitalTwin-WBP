import unittest
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from v2.geometry.tm_modes import classify
from v2.scripts.stage_v2 import load_parent

class TestModes(unittest.TestCase):
    def test_real_p01_middle_adjust(self):
        root=Path(__file__).resolve().parents[1]/'outputs/V2-20261028-224435-single_goal_05-RRTstar'
        if not root.exists(): self.skipTest('real parent unavailable')
        _,p=load_parent(root); _,_,_,m,_=classify(p)
        self.assertEqual([x['kind'] for x in m],['MOVE','ADJUST','MOVE','ADJUST'])
        self.assertEqual((m[1]['source_start'],m[1]['source_end']),(223,224))
    def test_micro_motion_and_adjust(self):
        q=np.zeros((4,9));q[1,0]=.0001;q[2]=q[1];q[2,2]=.2;q[3]=q[2];q[3,0]+=.0002
        _,_,_,m,_=classify(SimpleNamespace(q=q))
        self.assertEqual([x['kind'] for x in m],['MOVE','ADJUST','MOVE'])
    def test_hold_rejected(self):
        with self.assertRaisesRegex(ValueError,'HOLD_NOT_IMPLEMENTED'):
            classify(SimpleNamespace(q=np.zeros((2,9))),1.)
