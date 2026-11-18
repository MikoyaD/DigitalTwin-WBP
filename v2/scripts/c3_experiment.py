"""Opt-in C3 numerical candidate experiment. Does not start ROS or execute robots."""
import argparse
import json
import os
import tempfile
import time
import traceback
from dataclasses import asdict
from datetime import datetime,timezone,timedelta
from hashlib import sha256
from pathlib import Path
import numpy as np
from ..geometry.c3_transition import make_candidate,reference_tube_extrema
from ..geometry.unified_chart import nine_dimensional_progress
from ..geometry.unified_block import thaw_v2
from ..timing.unified_g import parameterize_block,_inverse_quintic
from ..core.math_tools import quintic_time_jet,chain_rule,differential_kinematics
from ..scripts.stage_v2 import load_parent,limits_for
from ..core.json_io import compatible,write_exclusive

EVIDENCE=Path('/home/robot/digitaltwin/research_management/evidence')

def atomic(path,value):
    path=Path(path);data=json.dumps(compatible(value),indent=2,ensure_ascii=False,allow_nan=False)
    with tempfile.NamedTemporaryFile('w',encoding='utf-8',dir=str(path.parent),delete=False) as f:
        temp=f.name;f.write(data);f.flush();os.fsync(f.fileno())
    os.replace(temp,str(path))

def fingerprint(path):return sha256(Path(path).read_bytes()).hexdigest()

def quality(block, t, q, v, acc, jerk):
    T=float(t[-1]-t[0]); sq=lambda x:np.sqrt(np.trapz(x*x,t,axis=0)/T)
    peak=np.max(abs(jerk),axis=0);sides=[]
    for left,right in zip(block.charts[:-1],block.charts[1:]):
        ss=left.s1;tt=_inverse_quintic(float(ss))*T;_,d1,d2,d3=quintic_time_jet(np.array([tt]),T)
        a=chain_rule(left.evaluate_global([ss]),d1,d2,d3)[2][0];b=chain_rule(right.evaluate_global([ss]),d1,d2,d3)[2][0]
        peak=np.maximum(peak,np.maximum(abs(a),abs(b)));sides.append({'u':ss,'t_s':tt,'left':a.tolist(),'right':b.tolist()})
    kin=differential_kinematics(q,v,acc,.06,.3935)
    return dict(duration_s=T,xy_jerk_peak=float(max(np.max(np.linalg.norm(jerk[:,:2],axis=1)),*[np.linalg.norm(x[k][:2]) for x in sides for k in ('left','right')])),
                xy_jerk_rms=float(sq(np.linalg.norm(jerk[:,:2],axis=1))),angular_arm_peak=peak[2:].tolist(),angular_arm_rms=sq(jerk[:,2:]).tolist(),
                left_right_wheel_acceleration_peak=np.max(abs(kin['wheel_acceleration']),axis=0).tolist(),
                length_xy_m=float(np.trapz(np.linalg.norm(v[:,:2],axis=1),t)),angle_abs_rad=float(np.trapz(abs(v[:,2]),t)),seams=sides)

def from_output(block,out):
    d=out['output'];return quality(block,*[d[k] for k in ('time_s','q','velocity','acceleration','jerk')])

def from_csv(block,path):
    x=np.loadtxt(str(path),delimiter=',',skiprows=1);return quality(block,x[:,0],x[:,1:10],x[:,10:19],x[:,19:28],x[:,28:37])

def run(source,output,a_only=False):
    source=Path(source).resolve(strict=True);parent,path=load_parent(source);limits=limits_for(parent);frozen=json.loads((source/'geometry.json').read_text())
    if frozen['parent_path_hash']!=path.parent_path_hash:raise RuntimeError('PARENT_IDENTITY_MISMATCH')
    original=thaw_v2(frozen)['blocks'][0];q,s,_=nine_dimensional_progress(path.q);a0=original.charts[0].s1;b=original.charts[1].s1;L=b-a0
    choices=[]
    for fac in (1,1.5,2,3):
        goal=b-fac*L;valid=s[(s<=min(goal,a0))&(s>0)]
        a=a0 if fac==1 else float(valid[-1]) if len(valid) else None
        if a is not None and not any(abs(a-z)<1e-12 for z in choices):choices.append(a)
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    payload={'source':str(source),'parent_hash':path.parent_path_hash,'parent_geometry_hash':fingerprint(source/'geometry.json'),
             'source_version':fingerprint(Path(__file__).parents[1]/'geometry/c3_transition.py'),
             'limits':asdict(limits),'scene':parent['scene'],'windows':choices,'A':{},'B':[],'C':{'collision_status':'UNKNOWN','promotion':'NO_PROMOTION_ELIGIBLE'},'failures':[]}
    atomic(output/'summary.json',payload)
    if (source/'G/trajectory.csv').is_file():payload['baseline']=from_csv(original,source/'G/trajectory.csv')
    # A first, bounded windows; first success only. Each attempted failure remains recorded.
    A=None
    for i,a in enumerate(choices):
        t0=time.perf_counter()
        try:
            block,detail=make_candidate(path,limits,original,a,'A');detail['tube']=reference_tube_extrema(block,path)
            geo=block.freeze(path,dict(candidate_id='UHG-C3-A-w%d'%i,detail=detail),frozen['identities']);geo['construction']['algorithm_version']='unified_h_c3_A'
            restored=thaw_v2(geo)['blocks'][0];grid=np.unique(np.r_[np.linspace(0,1,1001),[c.s0 for c in block.charts],[c.s1 for c in block.charts]])
            for name in ('q','qs','qss','qsss'):
                if not np.array_equal(getattr(block.evaluate(grid),name),getattr(restored.evaluate(grid),name)):raise ValueError('FREEZE_RESTORE_MISMATCH '+name)
            write_exclusive(output/'geometry_A.json',geo);detail.update(status='NUMERICALLY_VERIFIED',elapsed_s=time.perf_counter()-t0,geometry_sha256=fingerprint(output/'geometry_A.json'))
            payload['A']=detail;A=block;break
        except Exception as e:payload['failures'].append(dict(stage='A',window=i,entry=a,error=repr(e),traceback=traceback.format_exc()))
        finally:atomic(output/'summary.json',payload)
    if A is None:
        payload['A']={'status':'FAILED','reason':'bounded candidate budget exhausted'};atomic(output/'summary.json',payload);return payload
    print('A NUMERICALLY_VERIFIED',payload['A'],flush=True)
    try:
        out=parameterize_block(A,path,limits);payload['A']['G']={'status':'NUMERICALLY_VERIFIED','metrics':from_output(A,out),'backend_s':out['backend_s']}
    except Exception as e:payload['A']['G']={'status':'G_FAIL','error':repr(e),'traceback':traceback.format_exc()}
    atomic(output/'summary.json',payload)
    if a_only:
        payload['B_status']='NOT_REQUESTED';atomic(output/'summary.json',payload);return payload
    # B: exactly the predeclared 4 x 2 or fewer combinations.
    for i,a in enumerate(choices):
        for weight in (.1,1.):
            cid='UHG-C3-B-w%d-p%s'%(i,str(weight).replace('.','_'));t0=time.perf_counter();audit={};item={'candidate_id':cid,'entry':a,'weight':weight,'collision':'UNKNOWN','qp_audit':audit}
            try:
                block,detail=make_candidate(path,limits,original,a,'B',weight,audit=audit);detail['tube']=reference_tube_extrema(block,path)
                geo=block.freeze(path,dict(candidate_id=cid,detail=detail),frozen['identities']);geo['construction']['algorithm_version']='unified_h_c3_B'
                restored=thaw_v2(geo)['blocks'][0];grid=np.unique(np.r_[np.linspace(0,1,301),[c.s0 for c in block.charts],[c.s1 for c in block.charts]])
                for key in ('q','qs','qss','qsss'):
                    if not np.array_equal(getattr(block.evaluate(grid),key),getattr(restored.evaluate(grid),key)):raise ValueError('FREEZE_RESTORE_'+key)
                file=output/(cid+'.json');write_exclusive(file,geo);item.update(status='GEOMETRY_PASS',geometry_path=str(file),geometry_sha256=fingerprint(file),geometry=detail,h_time_s=time.perf_counter()-t0)
                try:
                    g=parameterize_block(restored,path,limits);item.update(status='NUMERICALLY_VERIFIED',metrics=from_output(restored,g),g_time_s=g['backend_s'])
                except Exception as e:item.update(status='G_FAIL',error=repr(e),traceback=traceback.format_exc())
            except Exception as e:item.update(status='GEOMETRY_FAIL',error=repr(e),traceback=traceback.format_exc())
            payload['B'].append(item);atomic(output/'summary.json',payload);print(cid,item['status'],item.get('error',''),flush=True)
    payload['B_status']='COMPLETE';payload['C']['status']='NOT_COLLISION_TESTED';atomic(output/'summary.json',payload);return payload

def main():
    p=argparse.ArgumentParser();p.add_argument('--source',required=True);p.add_argument('--output',required=True);p.add_argument('--a-only',action='store_true');a=p.parse_args();run(a.source,a.output,a.a_only)
if __name__=='__main__':main()
