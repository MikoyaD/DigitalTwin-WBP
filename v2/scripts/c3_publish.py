"""Publish an already checked C3 candidate to a new, independent V2 result."""
import argparse,json,shutil
from dataclasses import asdict
from pathlib import Path
import numpy as np
from ..geometry.unified_block import thaw_v2
from ..timing.unified_g import parameterize_block
from ..scripts.stage_v2 import load_parent,limits_for,new_dir,commit_g,digest,error
from ..scripts.c3_experiment import atomic,fingerprint


def main():
 p=argparse.ArgumentParser();p.add_argument('--evidence',required=True);args=p.parse_args();e=Path(args.evidence).resolve(strict=True);summary=json.loads((e/'summary.json').read_text());collision=json.loads((e/'collision_A.json').read_text())
 if summary['A'].get('published_result'):raise ValueError('ALREADY_PUBLISHED: '+summary['A']['published_result'])
 if collision['status']!='PASS' or summary['A']['G']['status']!='NUMERICALLY_VERIFIED':raise ValueError('NOT_VERIFIED_FOR_PUBLICATION')
 if summary['A']['geometry_sha256']!=fingerprint(e/'geometry_A.json'):raise ValueError('GEOMETRY_IDENTITY_CHANGED')
 source=Path(summary['source']);parent,path=load_parent(source);geo=json.loads((e/'geometry_A.json').read_text());block=thaw_v2(geo)['blocks'][0]
 if path.parent_path_hash!=geo['parent_path_hash'] or collision['scene_sha256']!=parent['scene']['scene_sha256']:raise ValueError('IDENTITY_MISMATCH')
 limits=limits_for(parent);g=parameterize_block(block,path,limits)
 if abs(g['duration_s']-summary['A']['G']['metrics']['duration_s'])>1e-9:raise ValueError('G_DURATION_CHANGED')
 root=new_dir(parent['config'],dict(id=parent['task_id']+'_UHG-C3-A'))
 try:
  for name in ('parent_path.csv','parent_path.json'):shutil.copyfile(str(source/name),str(root/name))
  shutil.copyfile(str(e/'geometry_A.json'),str(root/'geometry.json'))
  r=g['record'];v=g['validation'];item=dict(id=block.block_id,kind='UNIFIED_MOTION_BLOCK',start=block.start.tolist(),end=block.end.tolist(),duration_s=g['duration_s'],samples=len(r.time_s),uniform_n=g['uniform_n'],validation=asdict(v))
  result=dict(version=2,geometry_contract_version=2,algorithm='G',variant='UHG-C3-A',status='PASS',numerical_status='PASS',collision=collision,parent_path_hash=path.parent_path_hash,geometry_sha256=digest(root/'geometry.json'),duration_s=g['duration_s'],xy_length_m=g['xy_length_m'],planning_time_s=parent['planning']['reported_planning_time_s'],backend_s=g['backend_s'],planner_id=path.planner_id,blocks=[item],limits=asdict(limits),geometry_status='NUMERICAL_CHECKED_NOT_INTERVAL_CERTIFIED',time_law='10u^3-15u^4+6u^5 once for complete MotionBlock; analytic chain derivatives',post_totg_parent=True,promotion='VALID_BUT_NOT_PROMOTED',evidence=str(e))
  matrix=np.column_stack([g['output'][k] for k in ('time_s','q','velocity','acceleration','jerk')]);report='# UHG-C3-A：已验收但未推广\n\n固定父路径：`%s`。参考时间 %.9f s，XY 路长 %.9f m。数值与真实场景离散碰撞 PASS；连续碰撞未认证。\n\n候选证据：`%s`。\n'%(path.parent_path_hash,g['duration_s'],g['xy_length_m'],e)
  commit_g(root/'G',result,matrix,report);summary['A']['published_result']=str(root);atomic(e/'summary.json',summary);print('PUBLISHED',root,flush=True)
 except Exception as ex:error(root,'C3_PUBLISH',['c3_publish','--evidence',str(e)],ex);raise
if __name__=='__main__':main()
