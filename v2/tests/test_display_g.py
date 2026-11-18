"""Saved G display contract; ROS message check runs in an isolated child process."""
import json
import os
import subprocess
import unittest
from pathlib import Path

from v2.adapters.display_g import DEFAULT_RUN, TASKS, load_existing


class DisplayGTests(unittest.TestCase):
    def test_both_saved_g_inputs_are_real_and_monotonic(self):
        expected = {'5to1': 1001, '1to2': 5519}
        for task in TASKS:
            data = load_existing(DEFAULT_RUN, task)
            self.assertEqual(len(data.time_s), expected[task])
            self.assertTrue(all(b > a for a, b in zip(data.time_s, data.time_s[1:])))
            self.assertEqual(data.model_id, 'combined_robot')
            self.assertGreater(len(data.parent_path_hash), 32)
            self.assertGreater(abs(data.q[-1][0] - data.q[0][0]) + abs(data.q[-1][1] - data.q[0][1]), 1)
            self.assertGreater(max(abs(b[3] - a[3]) for a, b in zip(data.q, data.q[1:])), 0)
        turning = load_existing(DEFAULT_RUN, '1to2')
        self.assertEqual([phase for phase, _ in turning.phases],
                         ['SPIN', 'ARM_ONLY', 'TRANSLATE', 'SPIN', 'ARM_ONLY'])
        first_spin_end = next(i for i, t in enumerate(turning.time_s) if t >= turning.phases[0][1] - 1e-8)
        self.assertLess(max(abs(q[0] - turning.q[0][0]) + abs(q[1] - turning.q[0][1])
                            for q in turning.q[:first_spin_end + 1]), 1e-8)
        self.assertGreater(abs(turning.q[first_spin_end][2] - turning.q[0][2]), 0.1)

    def test_constructed_ros_message_has_aligned_whole_body_states(self):
        # Keep rospy/moveit messages out of the numerical regression runner process.
        script = r'''
import io, math
from moveit_msgs.msg import DisplayTrajectory
from v2.adapters.display_g import DEFAULT_RUN, load_existing, build_message, NAMES
for task in ('5to1', '1to2'):
    data=load_existing(DEFAULT_RUN,task);message=build_message(data)
    wire=io.BytesIO();message.serialize(wire)
    received=DisplayTrajectory();received.deserialize(wire.getvalue())
    assert received.model_id=='combined_robot' and len(received.trajectory)==1
    assert received.trajectory_start.is_diff is False
    joint=received.trajectory[0].joint_trajectory
    base=received.trajectory[0].multi_dof_joint_trajectory
    assert tuple(joint.joint_names)==NAMES[3:]
    assert list(base.joint_names)==['world_car']
    assert joint.header.frame_id==base.header.frame_id==data.frame_id=='world'
    assert len(joint.points)==len(base.points)==len(data.time_s)
    jt=[p.time_from_start.to_sec() for p in joint.points]
    mt=[p.time_from_start.to_sec() for p in base.points]
    assert jt==mt and all(b>a for a,b in zip(jt,jt[1:]))
    assert max(abs(a-b) for a,b in zip(jt,data.time_s))<1e-9
    for index in (0,len(data.q)//2,len(data.q)-1):
        q=data.q[index];tr=base.points[index].transforms[0]
        assert abs(tr.translation.x-q[0])<1e-12 and abs(tr.translation.y-q[1])<1e-12
        assert abs(2*math.atan2(tr.rotation.z,tr.rotation.w)-q[2])<1e-9
        assert max(abs(a-b) for a,b in zip(joint.points[index].positions,q[3:]))<1e-12
    assert 'world_car' in received.trajectory_start.multi_dof_joint_state.joint_names
    assert set(NAMES[3:]).issubset(set(received.trajectory_start.joint_state.name))
print('MESSAGE_CONTRACT_PASS')
'''
        setup = Path(__file__).resolve().parents[1] / 'scripts/env_noetic.sh'
        pkg = Path(__file__).resolve().parents[2]
        command = ['bash', '--noprofile', '--norc', '-c',
                   'source "$1"; cd "$2"; exec python3 -B -c "$3"', '_', str(setup), str(pkg), script]
        result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                universal_newlines=True, timeout=20, env=os.environ.copy())
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertIn('MESSAGE_CONTRACT_PASS', result.stdout)


if __name__ == '__main__':
    unittest.main()
