"""Read an existing V2 G CSV and publish a MoveIt display message only.

No planning, trajectory execution, TF publication, or robot state mutation.
ROS imports are confined to message construction/publication.
"""
import argparse
import csv
import json
import math
import time
from dataclasses import dataclass
from hashlib import sha256
from pathlib import Path
from xml.etree import ElementTree


V2 = Path(__file__).resolve().parents[1]
DEFAULT_RUN = V2 / 'outputs/runs/run_20261022T092444Z_adb7f2f5'
TASKS = {'5to1': 'archived_5_to_1_joint_base', '1to2': 'archived_1_to_2_turn_arm'}
NAMES = ('world_car/x', 'world_car/y', 'world_car/theta') + tuple('joint%d' % i for i in range(1, 7))
BASE_JOINT = 'world_car'
TOPIC = '/v2/display_trajectory'


@dataclass(frozen=True)
class DisplayInput:
    run_dir: Path
    task: str
    time_s: tuple
    q: tuple
    velocity: tuple
    acceleration: tuple
    frame_id: str
    model_id: str
    response_hex: str
    phases: tuple
    parent_path_hash: str
    robot_state_hex: str = ''


def digest(path):
    return sha256(Path(path).read_bytes()).hexdigest()


def load_existing(run_dir, task):
    run_dir = Path(run_dir).resolve()
    name = TASKS.get(task, task)
    import re
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]*', name):raise ValueError('invalid task_id')
    folder = run_dir / ('g_' + name)
    csv_path = folder / 'trajectory.csv'
    result_path = folder / 'result.json'
    capture_path = run_dir / ('planning_' + name) / 'capture/capture_result.json'
    for path in (csv_path, result_path, capture_path, run_dir / 'manifest.json'):
        if not path.is_file():
            raise FileNotFoundError(path)
    manifest = json.loads((run_dir / 'manifest.json').read_text())
    if manifest.get('status') != 'PASS':
        raise ValueError('source run is not PASS')
    for path in (csv_path, result_path, capture_path):
        key = str(path.relative_to(run_dir))
        expected = manifest.get('artifacts', {}).get(key, {}).get('sha256')
        if not expected or digest(path) != expected:
            raise ValueError('source artifact hash missing or changed: ' + key)
    result = json.loads(result_path.read_text())
    if result.get('numerical_status') != 'PASS':
        raise ValueError('G numerical result is not PASS')
    schema = result['path']['schema']
    if tuple(schema['joint_names']) != NAMES or schema['reference_point'] != 'car_link':
        raise ValueError('unexpected nine-state order or reference point')
    frame = schema['frame_id']
    if frame != 'world':
        raise ValueError('unexpected base frame: ' + str(frame))
    capture = json.loads(capture_path.read_text())
    if capture.get('parent_path_hash') != result['path']['parent_path_hash']:
        raise ValueError('parent path identity mismatch')
    if not capture.get('response_hex'):
        raise ValueError('complete parent RobotState response missing')
    expected_header = ['time_s'] + [prefix + n for prefix in ('q_', 'v_', 'a_', 'jerk_') for n in NAMES]
    times, qs, vs, accs = [], [], [], []
    with csv_path.open(newline='') as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames != expected_header:
            raise ValueError('unexpected CSV field names/order')
        for row in reader:
            t = float(row['time_s'])
            q = tuple(float(row['q_' + n]) for n in NAMES)
            v = tuple(float(row['v_' + n]) for n in NAMES)
            a = tuple(float(row['a_' + n]) for n in NAMES)
            if not all(math.isfinite(x) for x in (t,) + q + v + a):
                raise ValueError('nonfinite trajectory value')
            if times and t <= times[-1]:
                raise ValueError('time_s must be strictly increasing')
            times.append(t); qs.append(q); vs.append(v); accs.append(a)
    if len(times) < 2 or abs(times[0]) > 1e-9:
        raise ValueError('trajectory must start at zero and contain at least two points')
    if abs(times[-1] - float(result['duration_s'])) > 1e-7:
        raise ValueError('CSV/G duration mismatch')
    model_paths = [run_dir / 'preflight_recovered/robot_description.urdf',
                   run_dir / 'preflight/robot_description.urdf']
    model_path = next((path for path in model_paths if path.is_file()), None)
    if model_path is None:
        raise ValueError('saved RobotModel snapshot missing')
    model_key = str(model_path.relative_to(run_dir))
    model_hash = manifest.get('artifacts', {}).get(model_key, {}).get('sha256')
    if not model_hash or digest(model_path) != model_hash:
        raise ValueError('saved RobotModel snapshot hash missing or changed')
    model_id = ElementTree.parse(model_path).getroot().attrib.get('name')
    if not model_id:
        raise ValueError('saved RobotModel has no name')
    return DisplayInput(run_dir, name, tuple(times), tuple(qs), tuple(vs), tuple(accs), frame,
                        model_id, capture['response_hex'],
                        tuple((p['kind'], float(p['duration_s'])) for p in result['phases']),
                        capture['parent_path_hash'])


def _transform(q):
    from geometry_msgs.msg import Transform
    item = Transform()
    item.translation.x = q[0]
    item.translation.y = q[1]
    item.rotation.z = math.sin(q[2] / 2)
    item.rotation.w = math.cos(q[2] / 2)
    return item


def build_message(data):
    """Construct one complete DisplayTrajectory with aligned joint/MD time grids."""
    import copy
    import rospy
    from geometry_msgs.msg import Twist
    from moveit_msgs.msg import DisplayTrajectory, MotionPlanResponse, RobotTrajectory, RobotState
    from trajectory_msgs.msg import JointTrajectoryPoint, MultiDOFJointTrajectoryPoint

    if data.robot_state_hex:
        start = RobotState()
        start.deserialize(bytes.fromhex(data.robot_state_hex))
    else:
        response = MotionPlanResponse()
        response.deserialize(bytes.fromhex(data.response_hex))
        if response.error_code.val != 1:
            raise ValueError('saved MoveIt response was unsuccessful')
        start = copy.deepcopy(response.trajectory_start)
    arm_lookup = {name: i for i, name in enumerate(start.joint_state.name)}
    if not all(name in arm_lookup for name in NAMES[3:]):
        raise ValueError('saved start state has incomplete arm joints')
    if BASE_JOINT not in start.multi_dof_joint_state.joint_names:
        raise ValueError('saved start state has no world_car virtual joint')
    if start.multi_dof_joint_state.header.frame_id != data.frame_id:
        raise ValueError('saved start state uses a different frame')
    start.joint_state.position = list(start.joint_state.position)
    for i, name in enumerate(NAMES[3:], 3):
        start.joint_state.position[arm_lookup[name]] = data.q[0][i]
    base_index = list(start.multi_dof_joint_state.joint_names).index(BASE_JOINT)
    start.multi_dof_joint_state.transforms[base_index] = _transform(data.q[0])
    start.is_diff = False

    traj = RobotTrajectory()
    jt = traj.joint_trajectory
    md = traj.multi_dof_joint_trajectory
    jt.header.frame_id = data.frame_id
    md.header.frame_id = data.frame_id
    jt.joint_names = list(NAMES[3:])
    md.joint_names = [BASE_JOINT]
    for index, (t, q) in enumerate(zip(data.time_s, data.q)):
        v = data.velocity[index] if data.velocity else None
        a = data.acceleration[index] if data.acceleration else None
        stamp = rospy.Duration.from_sec(t)
        jp = JointTrajectoryPoint()
        jp.positions = q[3:]
        jp.velocities = v[3:] if v is not None else []
        jp.accelerations = a[3:] if a is not None else []
        jp.time_from_start = stamp
        mp = MultiDOFJointTrajectoryPoint()
        mp.transforms = [_transform(q)]
        if v is not None:
            speed = Twist();speed.linear.x = v[0];speed.linear.y = v[1];speed.angular.z = v[2]
            mp.velocities = [speed]
        if a is not None:
            accel = Twist();accel.linear.x = a[0];accel.linear.y = a[1];accel.angular.z = a[2]
            mp.accelerations = [accel]
        mp.time_from_start = stamp
        jt.points.append(jp)
        md.points.append(mp)
    message = DisplayTrajectory()
    message.model_id = data.model_id
    message.trajectory_start = start
    message.trajectory = [traj]
    return message


def describe(data):
    return dict(task=data.task, model_id=data.model_id, frame_id=data.frame_id,
                parent_path_hash=data.parent_path_hash, topic=TOPIC, points=len(data.time_s),
                duration_s=data.time_s[-1], xy_start=data.q[0][:2], xy_end=data.q[-1][:2],
                theta_start=data.q[0][2], theta_end=data.q[-1][2], phases=data.phases)


def main(argv=None):
    parser = argparse.ArgumentParser(description='Display a saved V2 G trajectory in RViz; never execute it')
    parser.add_argument('--run-dir', default=str(DEFAULT_RUN))
    parser.add_argument('--task', default='5to1')
    parser.add_argument('--repeat', type=int, default=1, help='0 repeats until interrupted')
    parser.add_argument('--gap', type=float, default=2.0, help='seconds after trajectory duration before replay')
    parser.add_argument('--wait-subscriber', type=float, default=30.0)
    parser.add_argument('--dry-run', action='store_true', help='construct/serialize only; no ROS master needed')
    args = parser.parse_args(argv)
    if args.repeat < 0 or args.gap < 0 or args.wait_subscriber < 0:
        parser.error('repeat, gap and wait-subscriber must be nonnegative')
    data = load_existing(args.run_dir, args.task)
    message = build_message(data)
    import io
    stream = io.BytesIO();message.serialize(stream)
    info = describe(data)
    info['serialized_bytes'] = len(stream.getvalue())
    print(json.dumps(info, ensure_ascii=False), flush=True)
    if args.dry_run:
        print('MESSAGE_CONSTRUCTION_PASS; ROS_PUBLISH_NOT_ATTEMPTED', flush=True)
        return 0
    import rospy
    from moveit_msgs.msg import DisplayTrajectory
    rospy.init_node('v2_display_g', anonymous=True, disable_signals=False)
    pub = rospy.Publisher(TOPIC, DisplayTrajectory, queue_size=1, latch=True)
    deadline = time.monotonic() + args.wait_subscriber
    while pub.get_num_connections() == 0 and time.monotonic() < deadline and not rospy.is_shutdown():
        time.sleep(0.05)
    if pub.get_num_connections() == 0:
        print('NO_SUBSCRIBER: open RViz Trajectory display on ' + TOPIC, flush=True)
        return 2
    count = 0
    while not rospy.is_shutdown() and (args.repeat == 0 or count < args.repeat):
        pub.publish(message)
        count += 1
        print('PUBLISHED topic=%s task=%s repeat=%d connections=%d' %
              (TOPIC, data.task, count, pub.get_num_connections()), flush=True)
        # Keep the latched publisher alive throughout RViz playback.
        until = time.monotonic() + data.time_s[-1] + args.gap
        while time.monotonic() < until and not rospy.is_shutdown():
            time.sleep(min(0.1, max(0.0, until - time.monotonic())))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
