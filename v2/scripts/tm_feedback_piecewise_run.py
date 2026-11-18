"""Independent fixed-H piecewise validation; never replaces old run artifacts."""
import argparse,errno,json,os,shutil,time,traceback
from dataclasses import asdict
from pathlib import Path
import numpy as np
from ..scripts.tm_feedback_hg import inputs,check_workspace,metrics,ChartView
from ..adapters.planning_scene_checker import PlanningSceneChecker
from ..scripts.stage_v2 import context,clean_collision

def collision_chunked(parent,block,data):
    """Check every saved sample and midpoint, chunked without changing resolution."""
    scene=context(parent['config'],parent['scene'])
    import rospy
    if not rospy.core.is_initialized():rospy.init_node('v2_tm_piecewise_collision',anonymous=True,disable_signals=True)
    phases=[ChartView(x,block.schema,i) for i,x in enumerate(block.charts)]
    q=data['q']; results=[];i=0
    while i<len(q):
        end=min(len(q),i+20000)
        result=clean_collision(PlanningSceneChecker().check(phases,dict(q=q[i:end])))
        results.append(dict(start=i,end=end,**result))
        if result['status']!='PASS' or result.get('scene_sha256')!=scene['scene_sha256']:
            return dict(status=result['status'] if result['status']!='PASS' else 'UNKNOWN',reason=result.get('reason'),chunks=results)
        if end==len(q):break
        i=end-1  # preserve the boundary midpoint between adjacent chunks
    return dict(status='PASS',scene_sha256=scene['scene_sha256'],chunks=results,checked_states=sum(r['checked_states'] for r in results))
from ..scripts.stage_v2 import new_dir,write,digest,HEADER
from ..timing.tm_g import sample,seams
from ..timing.feasible_profile import ratio
from ..validation.numerical import validate
from ..validation.tm_feedback_piecewise import integrate


def dense_sample_count(T, n):
    count=max(n, max(1001,int(np.ceil(T/.02))+1)*16)
    if count>200001:raise ValueError('TM_TIME_BUDGET')
    return count


def nonintegration_failures(codes):
    return [x for x in codes if x not in ('INCONSISTENT_V_FROM_A','INCONSISTENT_A_FROM_J')]


def run(source, numeric_only=False, defer_collision=False, out=None):
    # Signed H uses the A→C physical time law; legacy H retains its original path.
    import json
    source_path=Path(source).resolve(strict=True)
    if json.loads((source_path/'geometry.json').read_text()).get('algorithm_variant')=='V2-SIGNED-H-v1':
        if out is None:
            parent,_=__import__('v2.scripts.stage_v2',fromlist=['load_parent']).load_parent(source_path)
            out=new_dir(parent['config'],dict(id=parent['task_id']+'_SIGNED_AC'))
            # new_dir already reserved this fresh output directory.
            from .g0_signed_ac import run as run_signed
            return run_signed(source_path,out,defer_collision=defer_collision,numeric_only=numeric_only,reserved=True)
        from .g0_signed_ac import run as run_signed
        return run_signed(source_path,out,defer_collision=defer_collision,numeric_only=numeric_only)
    source,parent,path,geo,lim,block=inputs(source)
    dest=new_dir(parent['config'],dict(id=parent['task_id']+'_TM_PIECEWISE')) if out is None else Path(out)
    if out is not None:dest.mkdir(parents=True,exist_ok=False)
    for name in ('parent_path.json','parent_path.csv','geometry.json'):
        if out is None:shutil.copyfile(source/name,dest/name)
        else:
            try:os.link(source/name,dest/name)
            except OSError as exc:
                if exc.errno != errno.EXDEV:raise
                shutil.copyfile(source/name,dest/name)
    if digest(source/'geometry.json')!=digest(dest/'geometry.json'):raise RuntimeError('GEOMETRY_COPY_MISMATCH')
    folder=dest/'G';folder.mkdir();log=dict(status='FAIL',source=str(source),geometry_sha256=digest(dest/'geometry.json'),collision='NOT_EVALUATED',validator_source_sha256=digest(Path(__file__).resolve().parents[1]/'validation/tm_feedback_piecewise.py'),runner_source_sha256=digest(Path(__file__).resolve()))
    try:
        last=None;start=time.perf_counter()
        for n in (1001,2001,4001,8001,16001):
            r,_=sample(block,path,1.,n,geo['construction']['progress']);peak=ratio(r,lim)
            if last is not None and abs(peak-last)<=1e-4*max(peak,1):break
            last=peak
        else:raise ValueError('TIME_GRID_UNRESOLVED')
        T=max(.001,1.01*peak)
        for i in range(5):
            n=max(1001,int(np.ceil(T/.02))+1)*2**i
            if n>200001:raise ValueError('TM_TIME_BUDGET')
            rec,u=sample(block,path,T,n,geo['construction']['progress']);factor=ratio(rec,lim)
            if factor>1:T*=factor*1.01;continue
            # Do not accept a coarse grid that misses a narrow limit peak.
            dense_n=dense_sample_count(T,n)
            rec,u=sample(block,path,T,dense_n,geo['construction']['progress']);n=dense_n
            factor=ratio(rec,lim)
            if factor>1:T*=factor*1.01;continue
            check=validate(rec,lim,block.start,block.end)
            other=nonintegration_failures(check.failure_codes)
            if other:raise ValueError('ORIGINAL_NUMERIC_FAILURE '+repr(other))
            pc=integrate(block,T,geo['construction']['progress'])
            if pc['status']!='NUMERIC_INTEGRATION_PASS':raise ValueError('PIECEWISE_'+repr(pc))
            sj=seams(block,T);check_workspace(parent,rec)
            break
        else:raise ValueError('TIME_REFINEMENT_EXHAUSTED')
        data=dict(time_s=rec.time_s,q=rec.q,velocity=rec.velocity,acceleration=rec.acceleration,jerk=rec.jerk)
        log.update(status='NUMERICAL_PASS_COLLISION_PENDING',duration_s=T,sample_count=n,
                   numerical_integration=pc,nine_dof_limits='PASS',c2='PASS',full_g='PENDING_COLLISION',
                   original_nonintegration_checks=asdict(check),seams=sj,metrics=metrics(block,data,T,lim),compute_s=time.perf_counter()-start)
        np.savetxt(folder/'trajectory.csv',np.column_stack([data[k] for k in ('time_s','q','velocity','acceleration','jerk')]),delimiter=',',fmt='%.17g',header=','.join(HEADER),comments='')
        write(folder/'numerical_result.json',log)
        if defer_collision:
            log['collision']={'status':'UNKNOWN','reason':'deferred to final G1 trajectory'}
        else:
          try:
            if numeric_only: raise RuntimeError('COLLISION_NOT_ATTEMPTED_PERMISSION_BLOCK')
            col=collision_chunked(parent,block,data);log['collision']=col
            if col['status']!='PASS':raise RuntimeError('COLLISION_'+col['status']+': '+str(col.get('reason')))
            log['status']='FULL_G_PASS';log['full_g']='PASS'
          except Exception as exc:
            state='FAIL' if "'status': 'FAIL'" in str(exc) else 'UNKNOWN'
            log['collision']=log['collision'] if isinstance(log['collision'],dict) else dict(status=state,error=repr(exc));log['status']='COLLISION_NOT_PASS';log['full_g']='NOT_PASS'
            (folder/'collision_error.log').write_text(traceback.format_exc())
    except Exception as exc:
        log['error']=repr(exc);(folder/'error.log').write_text(traceback.format_exc())
    write(folder/'result.json',log)
    print('PIECEWISE_RESULT',dest,log['status'],flush=True)
    return dest,log

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--input',required=True);p.add_argument('--numeric-only',action='store_true');a=p.parse_args();run(a.input,a.numeric_only)
