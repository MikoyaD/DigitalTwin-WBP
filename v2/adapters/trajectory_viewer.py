"""Reference-only trajectory registry and one-publisher interactive RViz viewer."""
import argparse
import csv
import io
import json
import math
import os
import re
import time
from pathlib import Path
from xml.etree import ElementTree as ET
from .display_g import V2, DEFAULT_RUN, NAMES, TASKS, TOPIC, DisplayInput, digest, load_existing, build_message

REGISTRY = V2 / 'trajectories'


def canonical(path):
    path = Path(os.path.abspath(str(path)))
    if any(p.is_symlink() for p in (path,) + tuple(path.parents)):
        raise ValueError('SYMLINK_REFUSED: ' + str(path))
    return path.resolve(strict=True)


def reference(path, folder):
    path = canonical(path)
    if not path.is_file():
        raise ValueError('NOT_A_FILE: ' + str(path))
    return dict(path=os.path.relpath(str(path), str(folder)), sha256=digest(path))


def resolve(ref, folder):
    path = canonical(folder / ref['path'])
    if 'sha256' in ref and digest(path) != ref['sha256']:
        raise ValueError('HASH_MISMATCH: ' + str(path))
    return path


def read_csv(path, columns=None):
    with Path(path).open(newline='') as stream:
        reader = csv.DictReader(stream)
        header = reader.fieldnames or []
        if len(set(header)) != len(header):
            raise ValueError('DUPLICATE_CSV_COLUMN')
        columns = columns or (['q_' + n for n in NAMES] if 'q_world_car/x' in header
                              else ['x', 'y', 'theta'] + list(NAMES[3:]))
        if len(columns) != 9 or len(set(columns)) != 9 or any(n not in header for n in ['time_s'] + columns):
            raise ValueError('CSV_REQUIRES_TIME_AND_NINE_POSITIONS')
        times, positions = [], []
        for row in reader:
            t = float(row['time_s']); q = tuple(float(row[n]) for n in columns)
            if not all(math.isfinite(x) for x in (t,) + q):
                raise ValueError('NONFINITE_CSV')
            if times and t <= times[-1]:
                raise ValueError('TIME_NOT_STRICTLY_INCREASING')
            times.append(t); positions.append(q)
    if len(times) < 2 or times[0] < 0 or any(round(b*1e9)<=round(a*1e9) for a,b in zip(times,times[1:])):
        raise ValueError('NONNEGATIVE_ROS_TIME_WITH_TWO_DISTINCT_POINTS_REQUIRED')
    return tuple(times), tuple(positions), columns


def state_and_model(meta, folder):
    from moveit_msgs.msg import MotionPlanResponse
    urdf = resolve(meta['urdf'], folder); srdf = resolve(meta['srdf'], folder)
    model = ET.parse(urdf).getroot(); semantics = ET.parse(srdf).getroot()
    if model.attrib['name'] != meta['model_id']:
        raise ValueError('MODEL_ID_MISMATCH')
    virtual = semantics.find("virtual_joint[@name='world_car']")
    if virtual is None or virtual.attrib.get('type') != 'planar' or virtual.attrib.get('parent_frame') != meta['frame_id'] or virtual.attrib.get('child_link') != 'car_link':
        raise ValueError('PLANAR_JOINT_OR_FRAME_MISMATCH')
    capture = json.loads(resolve(meta['initial_state_source'], folder).read_text())
    response = MotionPlanResponse(); response.deserialize(bytes.fromhex(capture['response_hex']))
    if response.error_code.val != 1:
        raise ValueError('INITIAL_STATE_SOURCE_UNSUCCESSFUL')
    state = response.trajectory_start
    names = list(state.joint_state.name)
    if len(set(names)) != len(names) or len(names) != len(state.joint_state.position):
        raise ValueError('INITIAL_STATE_DIMENSION')
    required = {j.attrib['name'] for j in model.findall('joint')
                if j.attrib['type'] != 'fixed' and j.find('mimic') is None}
    if not required.issubset(set(names)):
        raise ValueError('MISSING_NON_NINE_JOINT_STATE: ' + str(sorted(required - set(names))))
    if not all(math.isfinite(v) for v in state.joint_state.position):
        raise ValueError('NONFINITE_INITIAL_STATE')
    md = state.multi_dof_joint_state
    if md.header.frame_id != meta['frame_id'] or list(md.joint_names) != ['world_car'] or len(md.transforms) != 1:
        raise ValueError('INITIAL_PLANAR_STATE_MISMATCH')
    tr = md.transforms[0]
    values = [tr.translation.x,tr.translation.y,tr.translation.z,tr.rotation.x,tr.rotation.y,tr.rotation.z,tr.rotation.w]
    if not all(math.isfinite(v) for v in values) or abs(sum(v*v for v in values[3:])-1) > 1e-6:
        raise ValueError('INVALID_INITIAL_TRANSFORM')
    wire = io.BytesIO(); state.serialize(wire)
    return wire.getvalue().hex()


def load_entry(folder):
    folder = canonical(folder)
    meta = json.loads(canonical(folder / 'metadata.json').read_text())
    if meta.get('version') != 1 or meta['name'] != folder.name or tuple(meta['joint_order']) != NAMES or meta['base_joint'] != 'world_car':
        raise ValueError('INVALID_REGISTRY_SCHEMA_OR_NAME')
    times, q, _ = read_csv(resolve(meta['csv'], folder), meta['position_columns'])
    state_hex = state_and_model(meta, folder)
    # Audit references are checked too; registration never upgrades CSV safety status.
    for ref in meta.get('audit_sources', {}).values():
        resolve(ref, folder)
    data = DisplayInput(folder, meta['name'], times, q, (), (), meta['frame_id'], meta['model_id'], '', (),
                        meta.get('parent_path_hash') or '', state_hex)
    return meta, data


def scan(registry):
    if not Path(registry).exists():return {},[]
    registry = canonical(registry)
    found, errors, seen = {}, [], set()
    for folder in sorted(registry.iterdir()):
        if folder.name.startswith('.'):
            continue
        try:
            if folder.is_symlink():
                raise ValueError('SYMLINK_REFUSED')
            if not folder.is_dir():
                continue
            meta = json.loads(canonical(folder/'metadata.json').read_text())
            if meta['name'] != folder.name or meta['name'].casefold() in seen:
                raise ValueError('DUPLICATE_OR_MISMATCHED_NAME')
            seen.add(meta['name'].casefold())
            for key in ('csv','urdf','srdf','initial_state_source'):
                resolve(meta[key],folder)
            read_csv(resolve(meta['csv'],folder),meta['position_columns'])
            found[meta['name']] = folder
        except Exception as exc:
            errors.append((folder.name, str(exc)))
    return found, errors


def register(args):
    registry = Path(args.registry).absolute(); registry.mkdir(parents=True, exist_ok=True); registry = canonical(registry)
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,79}', args.name):
        raise ValueError('INVALID_NAME')
    if any(p.name.casefold() == args.name.casefold() for p in registry.iterdir()):
        raise ValueError('NAME_ALREADY_REGISTERED')
    folder = registry / args.name
    meta = dict(version=1, name=args.name, base_joint='world_car', joint_order=list(NAMES),
                time_unit='s', position_units=['m','m']+['rad']*7,
                algorithm=args.algorithm or 'UNKNOWN', validation_status='VISUALIZATION_ONLY',
                has_complete_experiment_metadata=False, audit_sources={})
    if args.run_dir:
        # Reuse historical G validation; arbitrary future backends use register-csv.
        data = load_existing(args.run_dir, args.task)
        root = canonical(args.run_dir); task = data.task
        archive = json.loads((root/'manifest.json').read_text())
        csv_path = root/('g_'+task)/'trajectory.csv'
        modeldir = next(root/p for p in ('preflight_recovered','preflight') if (root/p/'robot_description.urdf').is_file())
        meta.update(frame_id=data.frame_id, model_id=data.model_id, algorithm=args.algorithm or 'G',
                    original_run_id=root.name, parent_path_hash=data.parent_path_hash,
                    validation_status='ARCHIVED_NUMERICAL_AND_DISCRETE_COLLISION_PASS',
                    has_complete_experiment_metadata=True)
        for label, path in [('manifest',root/'manifest.json'),('result',root/('g_'+task)/'result.json'),
                            ('validation',root/('validation_'+task+'.json'))]:
            if label != 'manifest' and archive['artifacts'].get(str(path.relative_to(root)),{}).get('sha256') != digest(path):
                raise ValueError('ARCHIVED_AUDIT_HASH_MISMATCH: '+str(path))
            meta['audit_sources'][label] = reference(path, folder)
        if archive['artifacts'].get(str((modeldir/'robot_description.srdf').relative_to(root)),{}).get('sha256') != digest(modeldir/'robot_description.srdf'):
            raise ValueError('ARCHIVED_SRDF_HASH_MISMATCH')
        validation=json.loads((root/('validation_'+task+'.json')).read_text())
        if validation.get('overall_status') != 'PASS':
            raise ValueError('ARCHIVED_VALIDATION_NOT_PASS')
        meta.update(urdf=reference(modeldir/'robot_description.urdf',folder),
                    srdf=reference(modeldir/'robot_description.srdf',folder),
                    initial_state_source=reference(root/('planning_'+task)/'capture/capture_result.json',folder))
    else:
        if not args.csv or not args.state_from or not args.frame:
            raise ValueError('CSV_REQUIRES_CSV_FRAME_AND_STATE_FROM')
        templates,_ = scan(registry)
        if args.state_from not in templates:
            raise ValueError('UNKNOWN_STATE_TEMPLATE')
        template = templates[args.state_from]
        original, _ = load_entry(template)
        if args.frame != original['frame_id']:
            raise ValueError('TEMPLATE_FRAME_MISMATCH')
        csv_path = canonical(args.csv)
        meta.update(frame_id=args.frame, model_id=original['model_id'], original_run_id=None,
                    state_template=args.state_from)
        for key in ('urdf','srdf','initial_state_source'):
            meta[key] = reference(resolve(original[key],template),folder)
    _, _, columns = read_csv(csv_path)
    meta.update(csv=reference(csv_path,folder), position_columns=columns)
    state_and_model(meta, folder)
    # Validate everything before creating a registration. Exclusive mkdir prevents overwrites.
    folder.mkdir(exist_ok=False)
    try:
        with (folder/'metadata.json').open('x') as stream:
            json.dump(meta,stream,ensure_ascii=False,indent=2)
    except Exception:
        folder.rmdir(); raise
    print('REGISTERED', args.name, meta['validation_status'], flush=True)


class Session:
    def __init__(self, registry, publish, model_check=lambda meta,folder: None, runs_dir=None, summary_only=False):
        self.registry=registry; self.publish=publish; self.model_check=model_check; self.runs_dir=runs_dir; self.summary_only=summary_only; self.aliases={}
        self.available={}; self.current=None; self.refresh()

    def refresh(self):
        self.available,errors=scan(self.registry)
        self.aliases={}
        if self.runs_dir is not None:
            from .run_discovery import discover_runs,discover_minimal
            autos,auto_errors=discover_runs(self.runs_dir);errors+=auto_errors
            minimal,minimal_errors=discover_minimal(self.runs_dir.parent,summary_only=self.summary_only);errors+=minimal_errors
            autos.update(minimal)
            identities={};merged={}
            for name,entry in autos.items():
                m=entry['summary'];identity=str((entry['run']/m['csv']['path']).resolve())
                if identity in identities:
                    self.aliases[name]=identities[identity];continue
                identities[identity]=name;merged[name]=entry
            for name,folder in self.available.items():
                m=json.loads((folder/'metadata.json').read_text());identity=str((folder/m['csv']['path']).resolve())
                if identity in identities and m.get('validation_status')=='ARCHIVED_NUMERICAL_AND_DISCRETE_COLLISION_PASS':
                    self.aliases[name]=identities[identity]
                else:merged[name]=folder
            self.available=dict(sorted(merged.items()))
        print('Available trajectories:',flush=True)
        for i,name in enumerate(self.available,1):
            item=self.available[name]
            if isinstance(item,dict):
                m=item['summary'];created=m.get('created_beijing',m.get('created_utc','UNKNOWN'))
                print('%d. %s | %s | task=%s | variant=%s | %.6fs | %s'%(i,name,created,m['task_id'],m['algorithm'],m['duration_s'],m['validation_status']),flush=True)
            else:
                m=json.loads((item/'metadata.json').read_text())
                times,_,_=read_csv(resolve(m['csv'],item),m['position_columns'])
                print('%d. %s [registered] | time=%s | task=%s | variant=%s | run=%s | %.6fs | %s'%(i,name,m.get('created_utc','UNKNOWN'),m.get('task_id',name),m.get('algorithm','UNKNOWN'),m.get('original_run_id') or 'EXTERNAL',times[-1],m.get('validation_status','VISUALIZATION_ONLY')),flush=True)
        for name,error in errors:
            print('INVALID %s: %s'%(name,error),flush=True)

    def command(self,line):
        parts=line.strip().split()
        if not parts:return True
        if parts==['quit']:return False
        try:
            if parts in (['list'],['refresh']):self.refresh()
            elif parts==['replay']:
                if self.current is None:raise ValueError('NO_CURRENT_TRAJECTORY')
                if isinstance(self.current,tuple):self.play_file(self.current[1])
                else:self.play(self.current)
            elif len(parts)==2 and parts[0]=='play':
                name=parts[1]
                if name.isdigit():name=list(self.available)[int(name)-1] if 1<=int(name)<=len(self.available) else ''
                self.play(name)
            elif len(parts)==2 and parts[0]=='play-file':self.play_file(parts[1])
            else:raise ValueError('commands: list | refresh | play NAME/NUMBER | play-file ABSOLUTE_CSV | replay | quit')
        except Exception as exc:
            print('ERROR '+str(exc),flush=True)
        return True

    def play(self,name):
        name=self.aliases.get(name,name)
        if name not in self.available:raise ValueError('UNKNOWN_TRAJECTORY: '+name)
        item=self.available[name]
        if isinstance(item,dict):
            from .run_discovery import load_auto,load_preview_csv
            folder=item['run']
            if item.get('format')=='preview':meta,data=load_preview_csv(item['csv'])
            else:meta,data=load_auto(folder,item['task'],item['root'])
        else:folder=item;meta,data=load_entry(folder)
        self.model_check(meta,folder)
        self.publish(build_message(data))
        self.current=name
        print('VISUALIZATION_ONLY: playback does not imply numerical or collision validation',flush=True)
        print('PLAY name=%s points=%d duration_s=%.9f status=%s topic=%s'%
              (name,len(data.time_s),data.time_s[-1],meta['validation_status'],TOPIC),flush=True)

    def play_file(self,path):
        if not Path(path).is_absolute():raise ValueError('PLAY_FILE_REQUIRES_ABSOLUTE_PATH')
        from .run_discovery import load_preview_csv
        meta,data=load_preview_csv(path)
        self.model_check(meta,data.run_dir)
        self.publish(build_message(data))
        self.current=('file',str(canonical(path)))
        print('VISUALIZATION_ONLY: playback does not imply numerical or collision validation',flush=True)
        print('PLAY_FILE path=%s points=%d duration_s=%.9f status=%s topic=%s'%
              (path,len(data.time_s),data.time_s[-1],meta['validation_status'],TOPIC),flush=True)


def interactive(args):
    import fcntl
    import hashlib
    import rospy
    from moveit_msgs.msg import DisplayTrajectory
    import rosgraph
    # One interactive publisher per Master, including other viewer processes.
    key=hashlib.sha256(os.environ.get('ROS_MASTER_URI','').encode()).hexdigest()[:16]
    lock=open('/tmp/v2_viewer_'+key+'.lock','a')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:raise ValueError('VIEWER_ALREADY_RUNNING')
    rospy.init_node('v2_trajectory_viewer',anonymous=False)
    master=rosgraph.Master(rospy.get_name())
    def no_competitor():
        pubs=master.getSystemState()[0]
        others=[n for topic,nodes in pubs if topic==TOPIC for n in nodes if n!=rospy.get_name()]
        if others:raise ValueError('COMPETING_DISPLAY_PUBLISHER: '+str(others))
    no_competitor()
    pub=rospy.Publisher(TOPIC,DisplayTrajectory,queue_size=1,latch=True)
    def send(message):
        no_competitor()
        until=time.monotonic()+args.wait_subscriber
        while pub.get_num_connections()==0 and time.monotonic()<until and not rospy.is_shutdown():time.sleep(.05)
        if not pub.get_num_connections():raise ValueError('NO_SUBSCRIBER: open V2_G in RViz')
        pub.publish(message)
        print('PUBLISHED node=%s connections=%d'%(rospy.get_name(),pub.get_num_connections()),flush=True)
    def model_check(meta,folder):
        for parameter,key in (('/robot_description','urdf'),('/robot_description_semantic','srdf')):
            if not rospy.has_param(parameter):raise ValueError('RUNTIME_MODEL_MISSING: '+parameter)
            if rospy.get_param(parameter).strip()!=resolve(meta[key],folder).read_text().strip():
                raise ValueError('RUNTIME_MODEL_MISMATCH: '+parameter)
    session=Session(args.registry,send,model_check,runs_dir=V2/'outputs/runs')
    print('Commands: list | refresh | play NAME/NUMBER | play-file ABSOLUTE_CSV | replay | quit',flush=True)
    try:
        while not rospy.is_shutdown():
            try:line=input('viewer> ')
            except EOFError:break
            if not session.command(line):break
    finally:
        pub.unregister();rospy.signal_shutdown('viewer quit');lock.close()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--registry',default=str(REGISTRY))
    sub=parser.add_subparsers(dest='action',required=True)
    for action in ('register-v2','register-csv'):
        p=sub.add_parser(action);p.add_argument('--name',required=True);p.add_argument('--algorithm')
        if action=='register-v2':
            p.add_argument('--run-dir',required=True);p.add_argument('--task',required=True)
        else:
            p.add_argument('--csv',required=True);p.add_argument('--frame',required=True);p.add_argument('--state-from',required=True)
    sub.add_parser('list')
    p=sub.add_parser('unregister');p.add_argument('name')
    p=sub.add_parser('interactive');p.add_argument('--wait-subscriber',type=float,default=5.)
    args=parser.parse_args()
    try:
        if args.action.startswith('register-'):
            if args.action=='register-csv':args.run_dir=None
            register(args)
        elif args.action=='list':
            Session(args.registry,lambda message:None,runs_dir=V2/'outputs/runs')
        elif args.action=='unregister':
            if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,79}',args.name):raise ValueError('INVALID_NAME')
            folder=canonical(canonical(args.registry)/args.name)
            if sorted(p.name for p in folder.iterdir())!=['metadata.json']:raise ValueError('REGISTRATION_HAS_OTHER_FILES')
            canonical(folder/'metadata.json')
            (folder/'metadata.json').unlink();folder.rmdir()
            print('UNREGISTERED (source files retained)',args.name)
        else:interactive(args)
    except Exception as exc:
        print('ERROR '+str(exc),flush=True);return 2
    return 0


if __name__=='__main__':raise SystemExit(main())
