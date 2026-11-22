"""Bounded, read-only discovery of V2 trajectories for research preview."""
import json
import csv as csv_module
import math
import re
from pathlib import Path
from .display_g import V2,NAMES,DisplayInput,digest
from .trajectory_viewer import canonical,reference,read_csv,state_and_model

RUNS=V2/'outputs/runs'


def read(path):
    return json.loads(canonical(path).read_text())


def ancestors(run,root):
    seen=set()
    for _ in range(32):
        run=canonical(run)
        if run.parent!=root or run in seen:raise ValueError('INVALID_RUN_LINEAGE')
        seen.add(run);yield run
        parent=read(run/'manifest.json').get('parent_run_id')
        if not parent:return
        if not re.fullmatch(r'run_[A-Za-z0-9_-]+',parent):raise ValueError('INVALID_PARENT_RUN_ID')
        run=root/parent
    raise ValueError('RUN_LINEAGE_BUDGET')


def verified(path,root):
    path=canonical(path)
    parts=path.relative_to(root).parts
    if len(parts)<2:raise ValueError('NOT_A_RUN_ARTIFACT')
    owner=root/parts[0];manifest=read(owner/'manifest.json')
    expected=manifest.get('artifacts',{}).get(str(path.relative_to(owner)),{}).get('sha256')
    if not expected or digest(path)!=expected:raise ValueError('ARCHIVE_HASH_MISMATCH: '+str(path))
    return path


def load_auto(run,task,root=RUNS,csv_only=False):
    root=canonical(root);run=canonical(run)
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]*',task):raise ValueError('INVALID_TASK_ID')
    manifest=read(run/'manifest.json')
    if manifest.get('status')!='PASS':raise ValueError('RUN_NOT_PASS')
    vpath=verified(run/('validation_'+task+'.json'),root);val=read(vpath)
    collision=val.get('collision',{})
    if val.get('overall_status')!='PASS' or val.get('numerical_status')!='PASS' or collision.get('status')!='PASS' or collision.get('checked_states',0)<=0:
        raise ValueError('NUMERICAL_AND_DISCRETE_COLLISION_PASS_REQUIRED')
    lineage=list(ancestors(run,root));entry=None
    for holder in lineage:
        index=holder/'g_index.json'
        if index.is_file():
            entries=read(verified(index,root))
            entry=next((x for x in entries if x.get('task')==task and x.get('status','').startswith('G_NUMERICAL_PASS')),None)
            if entry:break
    if entry is None:raise ValueError('FROZEN_G_INDEX_MISSING')
    folder=canonical(entry['g_dir'])
    if folder.name!='g_'+task:raise ValueError('UNKNOWN_BACKEND_LAYOUT')
    resultpath=verified(folder/'result.json',root);csvpath=verified(folder/'trajectory.csv',root)
    arrays=None if csv_only else verified(folder/'phase_arrays.npz',root);source=verified(entry['source'],root)
    result=read(resultpath);capturepath=verified(source.parent/'capture_result.json',root);capture=read(capturepath)
    checks=[('g_result_sha256',resultpath),('csv_sha256',csvpath),('source_input_sha256',source)]
    if arrays is not None:checks.append(('phase_arrays_sha256',arrays))
    for key,path in checks:
        if val.get(key)!=digest(path):raise ValueError('VALIDATION_INPUT_HASH_MISMATCH: '+key)
    if entry.get('result_sha256')!=digest(resultpath) or entry.get('csv_sha256')!=digest(csvpath):raise ValueError('G_INDEX_HASH_MISMATCH')
    if result.get('source_input_sha256')!=digest(source) or result.get('numerical_status')!='PASS':raise ValueError('G_SOURCE_MISMATCH')
    schema=result['path']['schema']
    if tuple(schema['joint_names'])!=NAMES or schema['reference_point']!='car_link' or schema['frame_id']!='world':raise ValueError('G_SCHEMA_MISMATCH')
    if capture.get('parent_path_hash')!=result['path']['parent_path_hash']:raise ValueError('PARENT_PATH_MISMATCH')
    # Known full-scene snapshots only; never traverse arbitrary directory trees.
    candidates=[]
    for holder in lineage+list(ancestors(folder.parent,root)):
        if holder not in candidates:candidates.append(holder)
    models=None;scene_found=False
    for holder in candidates:
        artifacts=read(holder/'manifest.json').get('artifacts',{})
        for key in artifacts:
            if key.count('/')!=1:continue
            if key.endswith('/scene_snapshot.bin') and digest(verified(holder/key,root))==collision.get('scene_sha256'):
                scene_found=True
            if key.endswith('/robot_description.urdf') and models is None:
                urdf=verified(holder/key,root);srdf=verified(urdf.with_name('robot_description.srdf'),root)
                if collision.get('robot_description') and urdf.read_text().strip()!=collision['robot_description'].strip():continue
                if collision.get('robot_description_semantic') and srdf.read_text().strip()!=collision['robot_description_semantic'].strip():continue
                models=(urdf,srdf)
    if not scene_found or models is None:raise ValueError('MATCHING_SCENE_OR_MODEL_SNAPSHOT_MISSING')
    times,q,columns=read_csv(csvpath)
    if abs(times[-1]-result['duration_s'])>1e-7:raise ValueError('DURATION_MISMATCH')
    from xml.etree import ElementTree as ET
    meta=dict(name=run.name+'/'+task+'/G',model_id=ET.parse(models[0]).getroot().attrib['name'],frame_id=schema['frame_id'],base_joint='world_car',joint_order=list(NAMES),algorithm='G',original_run_id=run.name,task_id=task,duration_s=times[-1],created_utc=manifest.get('created_utc','UNKNOWN'),validation_status='DENSE_SOURCE_VERIFIED_DISPLAY_ONLY' if csv_only else 'ARCHIVED_NUMERICAL_AND_DISCRETE_COLLISION_PASS',csv=reference(csvpath,run),urdf=reference(models[0],run),srdf=reference(models[1],run),initial_state_source=reference(capturepath,run),position_columns=columns)
    state=state_and_model(meta,run)
    data=DisplayInput(run,task,times,q,(),(),schema['frame_id'],meta['model_id'],'',tuple((x['kind'],x['duration_s']) for x in result['phases']),result['path']['parent_path_hash'],state)
    return meta,data


def discover_runs(root=RUNS):
    root=Path(root)
    if not root.exists():return {},[]
    root=canonical(root);found={};errors=[]
    for run in sorted(root.iterdir()):
        if not run.name.startswith('run_') or not run.is_dir():continue
        try:
            if run.is_symlink():raise ValueError('SYMLINK_REFUSED')
            manifest=read(run/'manifest.json')
            if manifest.get('status')!='PASS':continue
            entries=read(verified(run/'validation_index.json',root))
            for entry in entries:
                task=entry.get('task','')
                try:
                    if entry.get('status')!='PASS':continue
                    meta,_=load_auto(run,task,root,csv_only=True)
                    found[meta['name']]=dict(run=run,task=task,root=root,summary=meta)
                except Exception as exc:errors.append((run.name+'/'+task,str(exc)))
        except Exception as exc:errors.append((run.name,str(exc)))
    return found,errors


def preview_status(result, collision):
    status=str(result.get('status','')).upper()
    if collision.get('status')=='FAIL':return 'COLLISION_FAIL'
    if 'FAIL' in status or result.get('numerical_status')=='FAIL':return 'NUMERICAL_FAIL'
    numerical=(result.get('numerical_status')=='PASS' or status in
               ('PASS','TASK_PASS','FULL_G_PASS','INDEPENDENT_NUMERICAL_PASS','NUMERICAL_PASS_COLLISION_PENDING','COLLISION_NOT_PASS'))
    if numerical and collision.get('status')=='PASS':
        return 'DENSE_SOURCE_VERIFIED_DISPLAY_ONLY' if result.get('display_sampling') else 'VERIFIED'
    if numerical and collision.get('status')=='FAIL':return 'COLLISION_FAIL'
    if numerical and collision.get('status')=='UNKNOWN' and collision.get('reason')!='deferred to final G1 trajectory':return 'COLLISION_UNKNOWN'
    if numerical and result.get('collision')=='NOT_ATTEMPTED' and not collision:
        return 'COLLISION_NOT_ATTEMPTED'
    if numerical:return 'COLLISION_PENDING'
    return 'UNVALIDATED'


def load_preview_csv(csv, summary_only=False):
    """Locate actual parent metadata; neither status nor hash is a display gate."""
    csv=canonical(csv); result_path=csv.with_name('result.json')
    result=read(result_path) if result_path.is_file() else {}
    stage=csv.parent; root=None; source=None
    if stage.name=='task' and result.get('source_parent'):
        root=stage.parent;source=(stage/Path(result['source_parent'])).parent
    elif stage.name in ('recommended','fast') and stage.parent.name=='P1' and (stage.parent.parent/'P0/parent_path.json').is_file():
        root=stage.parent.parent;source=root/'P0'
    elif stage.name=='task' and result.get('source_segment')=='segment_01':
        root=stage.parent;source=root/'segment_01'/'H'
    elif (stage.parent/'parent_path.json').is_file():
        root=stage.parent;source=root
        if root.name=='G0' and root.parent.name.startswith('V2-'):
            root=root.parent
    elif stage.parent.name=='g1' and (stage.parent.parent/'result.json').is_file():
        root=stage.parent.parent;pipeline=read(root/'result.json');source=Path(pipeline.get('h_dir',''))
        if not source.is_absolute():source=root/source
        summary=stage.parent/'summary.json'
        geometry=Path(read(summary).get('geometry','')) if summary.is_file() else None
        if geometry is not None and not geometry.is_absolute():geometry=summary.parent/geometry
        if geometry is None or canonical(source)!=canonical(geometry):
            raise ValueError('MISSING_MATCHING_G1_GEOMETRY_SOURCE')
    elif (stage.parent/'summary.json').is_file():
        root=stage.parent;source=Path(read(root/'summary.json').get('geometry',''))
        if not source.is_absolute():source=root/source
    else:raise ValueError('MISSING_PARENT_PATH_AND_INITIAL_ROBOT_STATE: provide matching V2 source metadata')
    source=canonical(source);parent=read(source/'parent_path.json');schema=parent['path']['schema']
    if tuple(schema['joint_names'])!=NAMES or schema['reference_point']!='car_link':raise ValueError('NINE_DOF_SCHEMA_MISMATCH')
    frame=schema['frame_id'];urdf=canonical(parent['sources']['urdf']['path']);srdf=canonical(parent['sources']['srdf']['path'])
    from xml.etree import ElementTree as ET
    from moveit_msgs.msg import RobotState
    model=ET.parse(urdf).getroot();semantic=ET.parse(srdf).getroot();virtual=semantic.find("virtual_joint[@name='world_car']")
    if virtual is None or virtual.get('type')!='planar' or virtual.get('parent_frame')!=frame or virtual.get('child_link')!='car_link':raise ValueError('PLANAR_MODEL_FRAME_MISMATCH')
    state=RobotState();state.deserialize(bytes.fromhex(parent['start_robot_state_hex']))
    names=list(state.joint_state.name);required={j.get('name') for j in model.findall('joint') if j.get('type')!='fixed' and j.find('mimic') is None}
    if len(set(names))!=len(names) or len(names)!=len(state.joint_state.position) or not required.issubset(names):
        raise ValueError('INCOMPLETE_INITIAL_ROBOT_STATE: '+str(sorted(required-set(names))))
    if not all(math.isfinite(v) for v in state.joint_state.position):raise ValueError('NONFINITE_INITIAL_ROBOT_STATE')
    md=state.multi_dof_joint_state
    if md.header.frame_id!=frame or list(md.joint_names)!=['world_car'] or len(md.transforms)!=1:raise ValueError('INITIAL_PLANAR_STATE_MISMATCH')
    transform=md.transforms[0]
    values=(transform.translation.x,transform.translation.y,transform.translation.z,
            transform.rotation.x,transform.rotation.y,transform.rotation.z,transform.rotation.w)
    if not all(math.isfinite(v) for v in values) or abs(sum(v*v for v in values[3:])-1)>1e-6:
        raise ValueError('INVALID_INITIAL_PLANAR_TRANSFORM')
    if summary_only:
        # Discovery only: inspect the first and final source row. Playback still
        # performs the original complete finite/time-order validation.
        with csv.open(newline='') as stream:
            reader=csv_module.DictReader(stream)
            header=reader.fieldnames or []
            first=next(reader)
        columns=(['q_'+n for n in NAMES] if 'q_world_car/x' in header
                 else ['x','y','theta']+list(NAMES[3:]))
        if len(set(header))!=len(header) or any(c not in header for c in ['time_s']+columns):
            raise ValueError('CSV_REQUIRES_TIME_AND_NINE_POSITIONS')
        with csv.open('rb') as stream:
            stream.seek(0,2); size=stream.tell(); stream.seek(max(0,size-65536))
            last=stream.read().decode('utf-8').splitlines()[-1]
        end=dict(zip(header,next(csv_module.reader([last]))))
        times=(float(first['time_s']),float(end['time_s']))
        q=tuple(tuple(float(row[c]) for c in columns) for row in (first,end))
        if not all(math.isfinite(x) for x in times) or times[0]<0 or times[-1]<=times[0] or not all(math.isfinite(x) for row in q for x in row):
            raise ValueError('INVALID_CSV_ENDPOINTS')
    else:
        times,q,columns=read_csv(csv)
    first=parent['path']['q'][0]
    if any(abs(q[0][i]-first[i])>1e-3 for i in (0,1,3,4,5,6,7,8)) or abs(math.remainder(q[0][2]-first[2],2*math.pi))>1e-3:
        raise ValueError('TRAJECTORY_START_DOES_NOT_MATCH_PARENT_ROBOT_STATE')
    collision_path=stage/'collision.json'
    if stage.name=='recommended' and stage.parent.name=='P1' and (stage.parent.parent/'collision.json').is_file():
        collision_path=stage.parent.parent/'collision.json'
    collision=read(collision_path) if collision_path.is_file() else result.get('collision',{})
    if not isinstance(collision,dict):collision={}
    if stage.name=='recommended' and stage.parent.name=='P1':
        prp_result=stage.parent.parent/'result.json'
        info=read(prp_result).get('csv',{}) if prp_result.is_file() else {}
        if isinstance(info,dict) and str(info.get('method','')).startswith('original_dense_rows'):
            result=dict(result,display_sampling=True)
    name=root.name+'/'+str(csv.relative_to(root).parent)
    meta=dict(name=name,task_id=parent['task_id'],algorithm=stage.name,original_run_id=root.name,
              created_beijing=parent.get('created_beijing','UNKNOWN'),validation_status=preview_status(result,collision),
              duration_s=times[-1],model_id=model.get('name'),frame_id=frame,base_joint='world_car',joint_order=list(NAMES),
              csv={'path':str(csv)},urdf={'path':str(urdf)},srdf={'path':str(srdf)},position_columns=columns)
    data=DisplayInput(root,parent['task_id'],times,q,(),(),frame,model.get('name'),'',(),
                      parent['path'].get('parent_path_hash',''),parent['start_robot_state_hex'])
    return meta,data


def load_minimal(folder, algorithm='G'):
    return load_preview_csv(Path(folder)/algorithm/'trajectory.csv')


def discover_minimal(root=V2/'outputs', summary_only=False):
    root=Path(root)
    if not root.exists():return {},[]
    found={};errors=[]
    folders=[]
    for folder in sorted(root.iterdir()):
        if not folder.is_dir():continue
        if folder.name.startswith(('V2-','PRP-')):folders.append(folder)
        elif folder.name.startswith('prp_yaml'):folders.extend(p for p in sorted(folder.iterdir()) if p.is_dir() and p.name.startswith(('V2-','PRP-')))
    for folder in folders:
        if folder.is_symlink():errors.append((folder.name,'SYMLINK_REFUSED'));continue
        for csv in sorted(folder.rglob('trajectory.csv')):
            if any(part.startswith('.') for part in csv.relative_to(folder).parts):continue
            variant=csv.parent
            try:
                meta,_=load_preview_csv(csv,summary_only=summary_only)
                name=folder.name+'/'+str(csv.relative_to(folder).parent)
                if name in found:name=str(folder.relative_to(root))+'/'+str(csv.relative_to(folder).parent)
                if name in found:raise ValueError('DUPLICATE_TRAJECTORY_NAME: '+name)
                found[name]=dict(format='preview',csv=csv,run=folder,summary=meta)
            except Exception as exc:errors.append((str(variant.relative_to(root)),str(exc)))
    return found,errors
