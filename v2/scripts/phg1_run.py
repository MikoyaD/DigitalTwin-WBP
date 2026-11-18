"""Run existing P -> H -> G0 -> G1 on one saved parent, without replanning."""
import argparse, json, os, shutil, subprocess, sys, time, traceback
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
from . import tm_feedback_hg, tm_feedback_piecewise_run, g1_general

ROOT = Path(__file__).resolve().parents[2]

def _summary_json(path):
    try:
        with Path(path).open() as stream:
            data = json.load(stream)
        return data if isinstance(data, dict) else {}
    except (OSError, ValueError, TypeError):
        return {}

def resolve_record_path(root, value):
    p = Path(value)
    return p if p.is_absolute() else root/p

def print_summary(out, record=None):
    """Report only persisted results; missing evidence never becomes PASS."""
    out = Path(out)
    record = record if record is not None else _summary_json(out/'result.json')
    print('\n========== PHG1 SUMMARY ==========', flush=True)
    print('P 目录：', record.get('parent', 'UNKNOWN'))
    print('PHG1 目录：', out.resolve())
    print('H 目录：', record.get('h_dir', 'UNKNOWN'))
    g0 = _summary_json(resolve_record_path(out,record['g0_dir'])/'result.json') if record.get('g0_dir') else {}
    def seconds(value):
        return ('%.6f s' % value) if isinstance(value, (int, float)) else 'UNKNOWN'
    def status(value):
        return value.get('status', 'UNKNOWN') if isinstance(value, dict) else (value or 'UNKNOWN')
    print('G0：时间 %s；数值 %s；碰撞 %s' % (
        seconds(g0.get('duration_s', record.get('g0_reference_s'))),
        g0.get('status', record.get('g0_status', 'UNKNOWN')),
        status(g0.get('collision'))))
    for name, label in (('fast','G1 fast'), ('recommended','G1 recommended')):
        folder = out/'g1'/name
        result = _summary_json(folder/'result.json')
        independent = _summary_json(folder/'independent_export.json')
        collision = _summary_json(folder/'collision.json')
        collision_status = status(collision) if collision else status(result.get('collision'))
        print('%s：时间 %s；数值 %s；独立复核 %s；碰撞 %s' % (
            label, seconds(result.get('duration_s')), result.get('status', 'UNKNOWN'),
            independent.get('status', 'UNKNOWN'), collision_status))
        if name == 'recommended' and record.get('status') == 'COLLISION_NOT_PASS':
            if not collision:
                print('  碰撞验收结果缺失：无法确定失败原因。')
            else:
                failures = [f for chunk in collision.get('chunks', [])
                            for f in chunk.get('failures', [])]
                contacts = [c for f in failures for c in f.get('contacts', [])]
                reason = collision.get('reason') or '未提供原因'
                if contacts:
                    pairs = ['%s ↔ %s (%s)' % (c.get('body1','?'), c.get('body2','?'), c.get('category','unknown')) for c in contacts]
                    print('  检测到碰撞：%s；接触：%s' % (reason, '；'.join(pairs)))
                    first_contact = next(f for f in failures if f.get('contacts'))
                    if first_contact.get('location') is not None:
                        print('  失败位置：', first_contact['location'])
                else:
                    print('  碰撞检查未通过或无法确定：%s；状态 %s（无接触证据）' % (reason, status(collision)))
    print('最终验收状态：', record.get('status', 'UNKNOWN'))
    if record.get('error'):
        print('运行错误：', record['error'])
    print('==================================', flush=True)

def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input', required=True, help='saved parent-path directory')
    p.add_argument('--out', required=True, help='new experiment directory')
    p.add_argument('--config', default='v2/configs/tm_rho_feedback_v1.yaml')
    p.add_argument('--until', choices=['h','g0','g1'], default='g1')
    p.add_argument('--existing-root', action='store_true', help=argparse.SUPPRESS)
    p.add_argument('--scene-snapshot', default=str(ROOT/'v2/outputs/runs/run_20261023T165050Z_548ef707/preflight_recovered/scene_snapshot.bin'))
    a = p.parse_args(argv); out = Path(a.out).resolve()
    if a.existing_root:
        if not (out/'P'/'parent_path.json').is_file() or (out/'result.json').exists():raise ValueError('INVALID_EXISTING_PHG1_ROOT')
    else:out.mkdir(parents=True, exist_ok=False)
    def rel(path):
        try:return str(Path(path).resolve().relative_to(out))
        except ValueError:return str(Path(path).resolve())
    start = time.perf_counter(); source=Path(a.input).resolve(strict=True)
    record = {'status':'STARTED', 'parent':'P', 'collision':'NOT_ATTEMPTED'}
    if source != out/'P':record['original_parent']=str(source)
    owned = False
    try:
        if source != out/'P':
            parent=out/'P';parent.mkdir(exist_ok=False)
            for name in ('parent_path.json','parent_path.csv'):shutil.copyfile(source/name,parent/name)
        else:parent=source
        def make_h_dir(config,task):
            dest=out/'H';dest.mkdir(parents=True,exist_ok=False);return dest
        t=time.perf_counter()
        with patch.object(tm_feedback_hg,'new_dir',make_h_dir):
            h=tm_feedback_hg.h(parent,a.config)
        record['h_wall_s']=time.perf_counter()-t
        if h is None: raise RuntimeError('H_FAILED; see newly created H directory')
        record['h_dir']=rel(h)
        if a.until=='h':
            record['status']='H_PASS'
            return 0
        t=time.perf_counter(); g0, gr=tm_feedback_piecewise_run.run(h,defer_collision=True,out=out/'G0'); record['g0_wall_s']=time.perf_counter()-t
        record.update(g0_dir=rel(g0/'G'),g0_status=gr['status'],g0_compute_s=gr.get('compute_s'),g0_reference_s=gr.get('duration_s'))
        if gr['status']!='NUMERICAL_PASS_COLLISION_PENDING': raise RuntimeError('G0_NUMERICAL_FAILURE')
        if a.until=='g0':
            record['status']='G0_NUMERICAL_PASS_COLLISION_PENDING'
            return 0
        args=SimpleNamespace(geometry=str(h),g0=str(g0/'G'),out=str(out/'g1'),allow_deferred_g0_collision=True)
        t=time.perf_counter(); result,summary=g1_general.stage_all(args);record['g1_stage_wall_s']=time.perf_counter()-t
        record['g1_search_s']=summary.get('search_total_s');record['g1_status']=summary['status'];record['g1_dir']=rel(result)
        summary_file=result/'summary.json'
        if summary_file.is_file():
            saved=json.loads(summary_file.read_text());saved['geometry']=os.path.relpath(h,result);saved['g0']=os.path.relpath(g0/'G',result);summary_file.write_text(json.dumps(saved,indent=2))
        record['generation_wall_s']=time.perf_counter()-start
        if summary['status']!='NUMERIC_ONLY_COLLISION_PENDING':raise RuntimeError('G1_NUMERICAL_FAILURE')
        status=g1_general.run_process([str(ROOT/'v2/scripts/status_v2.sh')]); state=json.loads(status['stdout'].strip().splitlines()[-1])['status']
        if state=='STOPPED':
            r=g1_general.run_process([str(ROOT/'v2/scripts/start_v2.sh'),'--config',str(ROOT/'v2/configs/v2_workspace_single_goal_9d_va_only.yaml')]);owned=True
            if r['returncode']:raise RuntimeError('ROS_START_FAILED')
        elif state!='READY':raise RuntimeError('ROS_RUNTIME_'+state)
        env=dict(os.environ,V2_MASTER_URI='http://127.0.0.1:11411');cmd=['bash','--noprofile','--norc','-c','source v2/scripts/env_noetic.sh >/dev/null; exec "$@"','_',sys.executable,'-B','-m','v2.scripts.g1_p01_collision','--input',str(result/'recommended'),'--geometry',str(h),'--scene-snapshot',str(Path(a.scene_snapshot).resolve())]
        t=time.perf_counter(); r=subprocess.run(cmd,cwd=ROOT,env=env,capture_output=True,text=True);record['collision_process_wall_s']=time.perf_counter()-t
        (out/'collision_command.log').write_text(r.stdout+'\n'+r.stderr)
        collision=json.loads((result/'recommended/collision.json').read_text()); record['collision']=collision['status'];record['collision_validation_s']=collision.get('collision_wall_s')
        record['status']='PASS' if r.returncode==0 and collision['status']=='PASS' else 'COLLISION_NOT_PASS'
        # Collision and independent numerical acceptance above consume dense CSVs.
        # Only after both pass do we publish unchanged selected display rows.
        if record['status']=='PASS':
            from . import g1_display_sampling
            staged=[]
            try:
                for variant in ('fast','recommended'):
                    folder=result/variant
                    temp,meta=g1_display_sampling.prepare(folder,h)
                    staged.append((folder,temp,meta))
                for folder,temp,meta in staged:
                    os.replace(temp,folder/'trajectory.csv')
                    result_file=folder/'result.json'
                    info=json.loads(result_file.read_text());info['display_sampling']=meta
                    result_file.write_text(json.dumps(info,indent=2))
                record['display_sampling']={folder.name:meta for folder,_,meta in staged}
            finally:
                for _,temp,_ in staged:temp.unlink(missing_ok=True)
    except Exception as e:
        record.update(status='FAIL',error=repr(e));(out/'error.log').write_text(traceback.format_exc())
    finally:
        record['acceptance_wall_s']=time.perf_counter()-start
        if owned:record['runtime_stop']=g1_general.run_process([str(ROOT/'v2/scripts/stop_v2.sh')])
        (out/'result.json').write_text(json.dumps(record,indent=2));print('PHG1_RESULT',out,record['status'],flush=True)
        if a.until=='g1':print_summary(out,record)
    return 0 if record['status']=='PASS' else 1

if __name__=='__main__':raise SystemExit(main())
