"""Structural invariants: frozen geometry and old G mathematics are identical."""
import json
import tempfile
import unittest
from datetime import datetime
from pathlib import Path
from unittest.mock import patch
import numpy as np
from moveit_msgs.msg import RobotTrajectory
from v2.core.contracts import Quantity
from v2.core.limits import load_limits
from v2.adapters.moveit_input import capture
from v2.adapters.run_discovery import discover_minimal
from v2.geometry.phase_segmenter import build_geometry,parameterize_geometry
from v2.geometry.frozen import freeze,thaw
from v2.scripts import stage_v2
from v2.tests.support import PACKAGE


class StructureTests(unittest.TestCase):
    def test_frozen_h_reuses_exact_historical_g_arrays(self):
        run=PACKAGE/'v2/outputs/runs/run_20261023T165050Z_548ef707'
        data=json.loads((run/'planning_single_goal_05/capture/g_input.json').read_text())
        message=RobotTrajectory();message.deserialize(bytes.fromhex(data['robot_trajectory_hex']))
        path=capture(message,data['metadata'])
        limits=load_limits(PACKAGE/'config/gazebo_combined_robot.urdf',PACKAGE/'config/joint_limits.yaml',
                           PACKAGE/'config/combined_robot.srdf',
                           {k:Quantity(**v) for k,v in data['research_limits'].items()},
                           velocity_scaling=data['velocity_scaling'],
                           acceleration_scaling=data['acceleration_scaling'],
                           backend_overrides=data.get('backend_limits'))
        geometry=build_geometry(path,limits)
        frozen=json.loads(json.dumps(freeze(path,geometry)))
        for original,restored in zip(geometry['phases'],thaw(frozen)['phases']):
            for key in ('q','qs','qss','qsss'):
                s=np.linspace(0,1,21)
                self.assertTrue(np.array_equal(getattr(original.evaluate(s),key),
                                               getattr(restored.evaluate(s),key)))
        output=parameterize_geometry(path,limits,thaw(frozen))
        archive=np.load(run/'g_single_goal_05/phase_arrays.npz',allow_pickle=False)
        for i,record in enumerate(output['records']):
            for key in ('time_s','q','velocity','acceleration','jerk'):
                self.assertTrue(np.array_equal(getattr(record,key),archive['phase_%03d_%s'%(i,key)]),
                                'phase %d %s differs from archived result'%(i,key))

    def test_human_name_collision_and_latest_skip_failure(self):
        with tempfile.TemporaryDirectory() as scratch,patch.object(stage_v2,'OUTPUTS',Path(scratch)):
            config=dict(planner_id='RRTstar');task=dict(id='test_task')
            fixed=datetime(2026,10,24,21,36,45)
            with patch.object(stage_v2.dt,'datetime') as clock:
                clock.now.return_value=fixed
                first=stage_v2.new_dir(config,task);second=stage_v2.new_dir(config,task)
            self.assertEqual(first.name,'V2-20261024-213645-test_task-RRTstar')
            self.assertTrue(second.name.endswith('-02'))
            (first/'parent_path.json').write_text('{}')
            self.assertEqual(stage_v2.select_input('latest','parent_path.json'),first)

    def test_failed_minimal_g_not_displayed(self):
        with tempfile.TemporaryDirectory() as scratch:
            folder=Path(scratch)/'V2-20261024-213645-task-RRTstar';(folder/'G').mkdir(parents=True)
            (folder/'G/error.log').write_text('stage=G')
            found,errors=discover_minimal(Path(scratch))
            self.assertEqual(found,{})
            self.assertEqual(errors,[])

    def test_unimplemented_solver_never_creates_output(self):
        with tempfile.TemporaryDirectory() as scratch,patch.object(stage_v2,'OUTPUTS',Path(scratch)):
            self.assertEqual(stage_v2.main(['--stage','t','--input','latest']),3)
            self.assertEqual(list(Path(scratch).iterdir()),[])
