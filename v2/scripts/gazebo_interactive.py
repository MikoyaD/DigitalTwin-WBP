"""Interactive CSV selection using the existing V2 scanner and isolated replay."""
import os
import contextlib
import csv
import io
import signal
import socket
import subprocess
import sys
import tempfile
import time
import xmlrpc.client
from pathlib import Path

from .runtime_v2 import V2
from ..adapters.trajectory_viewer import REGISTRY, Session, read_csv, NAMES

URI = 'http://127.0.0.1:11511'


def master_state():
    try:
        code, _, state = xmlrpc.client.ServerProxy(URI).getSystemState('/v2_display_menu')
        return state if code == 1 else None
    except (OSError, xmlrpc.client.Error):
        return None


def port_busy():
    with socket.socket() as sock:
        sock.settimeout(.3)
        return sock.connect_ex(('127.0.0.1', 11511)) == 0


def ready():
    state = master_state()
    if not state:
        return False
    pubs, _, services = state
    topics = {name for name, _ in pubs}
    nodes = {node for _, node_list in pubs + state[1] + services for node in node_list}
    srvs = {name for name, _ in services}
    # Refuse to take over an unrelated/partially configured master. The replay
    # coordinator independently checks the live OctoMap, model and controller.
    return (all(s in srvs for s in ('/gazebo/get_model_state', '/gazebo/get_link_state',
                                    '/gazebo/set_model_state', '/get_planning_scene'))
            and all(t in topics for t in ('/clock', '/joint_states', '/arm_controller/state',
                                         '/octomap_point_cloud_centers'))
            and all(n in nodes for n in ('/gazebo', '/move_group', '/v2_replay_octomap',
                                        '/v2_replay_world_to_odom', '/v2_replay_rviz')))


def stop_owned(process):
    if process and process.poll() is None:
        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=30)
        except subprocess.TimeoutExpired:
            # This process group was created by us, never an attached master.
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                return
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                print('本次启动的进程尚未退出，需手动结束 PID', process.pid, flush=True)


class GazeboSession(Session):
    @staticmethod
    def csv_path(item):
        if isinstance(item, dict):
            path = item.get('csv') or (item['run'] / item['summary']['csv']['path'])
        else:
            import json
            meta = json.loads((item / 'metadata.json').read_text())
            path = item / meta['csv']['path']
        return Path(path).resolve(strict=True)

    def refresh(self):
        # Reuse the original registry + run scanner and its deduplication. Only
        # display files that the existing Gazebo CSV reader can interpret.
        with contextlib.redirect_stdout(io.StringIO()):
            super().refresh()
        compatible = {}
        for name, item in self.available.items():
            try:
                path = self.csv_path(item)
                with path.open(newline='') as stream:
                    columns = set(next(csv.reader(stream)))
                base = {'time_s', 'x', 'y', 'theta'} | set(NAMES[3:])
                standard = {'time_s'} | {'q_' + n for n in NAMES}
                if base <= columns or standard <= columns:
                    compatible[name] = item
            except (OSError, ValueError, StopIteration, KeyError):
                continue
        self.available = compatible
        self.aliases = {alias: target for alias, target in self.aliases.items() if target in compatible}
        print('可回放 CSV（与原 V2 扫描器相同来源；NPZ 不支持）：', flush=True)
        for index, (name, item) in enumerate(compatible.items(), 1):
            path = self.csv_path(item)
            parts = [x.lower() for x in path.parts]
            if any(x.startswith(('segment_', 'seg0')) for x in parts):
                kind = '多目标子段/模块，非完整任务'
            elif path.parent.name.lower() == 'task':
                kind = '任务级 CSV，完成状态以结果报告为准'
            else:
                kind = '单段/模块，完成状态以结果报告为准'
            status = item.get('summary', {}).get('validation_status', '注册轨迹') if isinstance(item, dict) else '注册轨迹'
            print('%d. %s | %s | %s | %s' % (index, name, kind, status, path), flush=True)

    def play(self, name):
        name = self.aliases.get(name, name)
        if name not in self.available:
            raise ValueError('UNKNOWN_TRAJECTORY: ' + name)
        path = self.csv_path(self.available[name])
        read_csv(path)  # Same nine-position, monotonically timed CSV contract as the replay.
        output = Path(tempfile.mkdtemp(prefix='manual-replay-', dir=str(V2 / 'outputs')))
        print('GAZEBO_SELECTED', name, str(path), 'OUTPUT', str(output), flush=True)
        command = [sys.executable, '-B', '-m', 'v2.scripts.view_entry',
                   '--gazebo-csv', str(path), '--replay-output', str(output)]
        result = subprocess.run(command, cwd=str(V2.parent), env=os.environ.copy())
        print('GAZEBO_REPLAY_EXIT', result.returncode, flush=True)
        if result.returncode:
            raise RuntimeError('回放失败；请检查以上日志和 ' + str(output))
        self.current = name

    def play_file(self, path):
        raise ValueError('交互联合模式请从扫描列表选择；直接路径使用 --gazebo-csv')


def interactive_gazebo():
    old_uri = os.environ.get('ROS_MASTER_URI')
    old_home = os.environ.get('ROS_HOME')
    os.environ['ROS_MASTER_URI'] = URI
    os.environ['ROS_HOME'] = '/tmp/v2_csv_replay_ros'
    core = launch = None
    log = None
    try:
        state = master_state()
        if state is None:
            if port_busy():
                raise RuntimeError('11511 端口已占用但不是可用 ROS Master，不会接管')
            print('启动本次独立 11511 ROS Master。', flush=True)
            log = tempfile.NamedTemporaryFile(prefix='v2_gazebo_menu_', suffix='.log', delete=False)
            core = subprocess.Popen(['roscore', '-p', '11511'], stdout=log, stderr=subprocess.STDOUT,
                                    start_new_session=True)
            deadline = time.monotonic() + 20
            while time.monotonic() < deadline and master_state() is None and core.poll() is None:
                time.sleep(.25)
            if master_state() is None:
                raise RuntimeError('ROS Master 启动失败，日志：' + log.name)
            print('启动本次 Gazebo + MoveIt + 单 RViz；日志：' + log.name, flush=True)
            launch = subprocess.Popen(['roslaunch', 'cr5_car_moveit', 'gazebo_replay.launch'],
                                      stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic() + 120
            while time.monotonic() < deadline and not ready() and launch.poll() is None:
                time.sleep(.5)
            if not ready():
                raise RuntimeError('联合环境未就绪，日志：' + log.name)
        elif not ready():
            raise RuntimeError('11511 已有 Master，但不是完整联合环境；不接管、不启动第二套节点')
        else:
            print('复用现有 11511 联合环境；退出时不会停止它。', flush=True)
        print('环境就绪；此时机器人不会自动播放。选择 play 编号才开始回放。', flush=True)
        session = GazeboSession(REGISTRY, lambda _: None, runs_dir=V2 / 'outputs/runs', summary_only=True)
        print('命令：list | refresh | play 编号或名称 | replay | quit（返回模式选择）', flush=True)
        while True:
            try:
                line = input('gazebo> ')
            except EOFError:
                break
            if not session.command(line):
                break
    except (RuntimeError, OSError) as exc:
        print('GAZEBO_ENV_ERROR:', exc, flush=True)
    finally:
        stop_owned(launch)
        stop_owned(core)
        if log:
            log.close()
        for key, value in (('ROS_MASTER_URI', old_uri), ('ROS_HOME', old_home)):
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value
