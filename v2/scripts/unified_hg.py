"""Fixed-parent contract-v2 H/G entry.  Never replans or executes a robot."""
import argparse
import json
import shutil
import sys
import tempfile
import time
from dataclasses import asdict
from pathlib import Path
import numpy as np
from ..core.contracts import Status
from ..geometry.unified_block import build_unified,thaw_v2
from ..timing.unified_g import parameterize_block,sample_block,validate_v2
from ..adapters.planning_scene_checker import PlanningSceneChecker
from .stage_v2 import (read,write,new_dir,load_parent,limits_for,assert_sources,
                       context,clean_collision,commit_g,error,digest,HEADER)


def h(source):
    source=Path(source).resolve(strict=True)
    parent,path=load_parent(source);assert_sources(parent)
    root=new_dir(parent['config'],dict(id=parent['task_id']+'_UHG'))
    try:
        # New directory, new artifacts; source P and history remain immutable.
        for name in ('parent_path.csv','parent_path.json'):
            shutil.copyfile(source/name,root/name)
        build_started=time.perf_counter()
        block,decision=build_unified(path,limits_for(parent))
        frozen=block.freeze(path,decision,dict(scene=parent['scene'],sources=parent['sources']))
        frozen['construction']['h_build_s']=float(time.perf_counter()-build_started)
        rebuilt=thaw_v2(frozen)['blocks'][0]
        points=np.unique(np.r_[np.linspace(0,1,1001),[c.s0 for c in block.charts],
                               [c.s1 for c in block.charts]])
        for key in ('q','qs','qss','qsss'):
            if not np.array_equal(getattr(block.evaluate(points),key),getattr(rebuilt.evaluate(points),key)):
                raise RuntimeError('H_FREEZE_ROUNDTRIP: '+key)
        write(root/'geometry.json',frozen)
        print('UNIFIED_H_PASS',root,'candidate='+decision['candidate_id'],flush=True)
        return root
    except Exception as exc:
        error(root,'UNIFIED_H',['unified_hg','--stage','h','--input',str(source)],exc);raise


def _inputs(root):
    root=Path(root).resolve(strict=True);parent,path=load_parent(root);assert_sources(parent)
    frozen=read(root/'geometry.json')
    if frozen.get('contract_version')!=2 or frozen['parent_path_hash']!=path.parent_path_hash:
        raise RuntimeError('H_CONTRACT_OR_PARENT_MISMATCH')
    block=thaw_v2(frozen)['blocks'][0]
    if np.max(np.abs(block.start-path.q[0]))>1e-6 or np.max(np.abs(block.end-path.q[-1]))>1e-6:
        raise RuntimeError('H_ENDPOINT_MISMATCH')
    return root,parent,path,block


def _scene(parent,block,output,existing_scene=False):
    if existing_scene:
        from .experiment import check_master
        from .ros_probe import probe
        import os
        if check_master(os.environ.get('ROS_MASTER_URI',''))!='REACHABLE':
            raise RuntimeError('EXISTING_MASTER_NOT_REACHABLE')
        with tempfile.TemporaryDirectory(prefix='v2_unified_scene_') as temp:
            scene=probe(Path(temp),float(parent['config'].get('service_timeout_s',45)))
        for key in ('scene_sha256','urdf_sha256','srdf_sha256'):
            if scene[key]!=parent['scene'][key]:
                raise RuntimeError('EXISTING_SCENE_OR_MODEL_CHANGED: '+key)
    else:
        scene=context(parent['config'],parent['scene'])
    import rospy
    if not rospy.core.is_initialized():
        rospy.init_node('v2_unified_scene_check',anonymous=True,disable_signals=True)
    collision=clean_collision(PlanningSceneChecker().check([block],output))
    if collision['status']!='PASS' or collision.get('scene_sha256')!=scene['scene_sha256']:
        raise RuntimeError('G_COLLISION_NOT_PASS: '+repr(dict(status=collision['status'],
                           reason=collision.get('reason'),checked_states=collision.get('checked_states'),
                           first_failure=(collision.get('failures') or [None])[0])))
    return collision


def g(root,existing_scene=False):
    root,parent,path,block=_inputs(root);folder=root/'G'
    if (folder/'result.json').exists():raise RuntimeError('G_ALREADY_SUCCESSFUL')
    try:
        limits=limits_for(parent);out=parameterize_block(block,path,limits)
        collision=_scene(parent,block,out['output'],existing_scene)
        check=out['validation'];record=out['record']
        item=dict(id=block.block_id,kind='UNIFIED_MOTION_BLOCK',start=block.start.tolist(),
                  end=block.end.tolist(),duration_s=out['duration_s'],samples=len(record.time_s),
                  uniform_n=out['uniform_n'],validation=asdict(check))
        result=dict(version=2,geometry_contract_version=2,algorithm='G',variant='unified_fast',
                    status='PASS',numerical_status='PASS',collision=collision,
                    parent_path_hash=path.parent_path_hash,geometry_sha256=digest(root/'geometry.json'),
                    duration_s=out['duration_s'],xy_length_m=out['xy_length_m'],
                    planning_time_s=parent['planning']['reported_planning_time_s'],
                    backend_s=out['backend_s'],planner_id=path.planner_id,
                    blocks=[item],limits=asdict(limits),
                    geometry_status='NUMERICAL_CHECKED_NOT_INTERVAL_CERTIFIED',
                    time_law='10u^3-15u^4+6u^5 once for complete MotionBlock; analytic chain derivatives',
                    post_totg_parent=True)
        matrix=np.column_stack([out['output'][key] for key in ('time_s','q','velocity','acceleration','jerk')])
        report=('# 统一九维 H/G：%s\n\n父路径：`%s`；候选：`%s`。\n\n'
                '参考时长 %.9f s；XY 路长 %.9f m；G 计算 %.9f s。\n\n'
                '数值验收 PASS；完整机器人离散碰撞 PASS（%d 状态）；连续几何仅工程数值检查。\n')%(
                    parent['task_id'],path.parent_path_hash,
                    read(root/'geometry.json')['construction']['candidate']['candidate_id'],
                    out['duration_s'],out['xy_length_m'],out['backend_s'],collision['checked_states'])
        commit_g(folder,result,matrix,report)
        print('UNIFIED_G_PASS',folder,flush=True)
        return folder
    except Exception as exc:
        folder.mkdir(exist_ok=True)
        error(folder,'UNIFIED_G',['unified_hg','--stage','g','--input',str(root)],exc)
        raise


def validate_saved(root,existing_scene=False):
    root,parent,path,block=_inputs(root);folder=root/'G';result=read(folder/'result.json')
    if result.get('geometry_contract_version')!=2 or result['geometry_sha256']!=digest(root/'geometry.json'):
        raise RuntimeError('G_GEOMETRY_CHANGED')
    info=result['blocks'][0];duration=float(info['duration_s'])
    record,u=sample_block(block,path,duration,int(info['uniform_n']))
    check=validate_v2(record,limits_for(parent),block,u)
    if check.numerical_status!=Status.PASS:raise RuntimeError('INDEPENDENT_NUMERICAL_FAILURE: '+repr(check.failure_codes))
    expected=np.column_stack([getattr(record,key) for key in ('time_s','q','velocity','acceleration','jerk')])
    saved=np.loadtxt(folder/'trajectory.csv',delimiter=',',skiprows=1)
    if saved.shape!=expected.shape or not np.allclose(saved,expected,atol=1e-10,rtol=0):
        raise RuntimeError('CSV_NOT_EQUAL_TO_FROZEN_GEOMETRY')
    collision=_scene(parent,block,dict(time_s=record.time_s,q=record.q,velocity=record.velocity,
                                      acceleration=record.acceleration,jerk=record.jerk),existing_scene)
    if collision['scene_sha256']!=result['collision']['scene_sha256']:
        raise RuntimeError('VALIDATION_SCENE_CHANGED')
    print('UNIFIED_VALIDATE_PASS',folder,'states='+str(collision['checked_states']),flush=True)


def main(argv=None):
    parser=argparse.ArgumentParser(description='Fixed-P unified H/G; never replans or executes')
    parser.add_argument('--stage',choices=['h','g','validate','all'],required=True)
    parser.add_argument('--input',required=True,help='existing P root for h/all, new v2 root for g/validate')
    parser.add_argument('--existing-scene',action='store_true',
                        help='read a live compatible Master only; do not launch or own it')
    args=parser.parse_args(argv)
    if args.stage in ('h','all'):
        root=h(args.input)
        if args.stage=='h':return 0
        g(root,args.existing_scene);validate_saved(root,args.existing_scene);return 0
    if args.stage=='g':g(args.input,args.existing_scene)
    else:validate_saved(args.input,args.existing_scene)
    return 0


if __name__=='__main__':
    try:raise SystemExit(main())
    except Exception as exc:
        print('FAILED',repr(exc),file=sys.stderr);raise SystemExit(2)
