"""Check an existing C3 A candidate against the original live scene; never execute."""
import argparse,json,sys,tempfile,traceback
from pathlib import Path
from ..geometry.unified_block import thaw_v2
from ..timing.unified_g import parameterize_block
from ..scripts.stage_v2 import load_parent,limits_for,clean_collision
from ..scripts.runtime_v2 import status
from ..scripts.c3_experiment import atomic,fingerprint
from ..core.json_io import write_exclusive
from ..scripts.ros_probe import probe
from ..adapters.planning_scene_checker import PlanningSceneChecker

def main():
 p=argparse.ArgumentParser();p.add_argument('--evidence',required=True);a=p.parse_args();root=Path(a.evidence);summary=json.loads((root/'summary.json').read_text());source=Path(summary['source']);parent,path=load_parent(source);geo=json.loads((root/'geometry_A.json').read_text());block=thaw_v2(geo)['blocks'][0]
 if (root/'collision_A.json').exists():raise RuntimeError('COLLISION_EVIDENCE_EXISTS: preserve prior attempt')
 result=dict(status='UNKNOWN',stage='scene_preflight',checked_states=0,reason='not attempted')
 try:
  st=status()
  if st['status']!='READY':raise RuntimeError('V2_RUNTIME_NOT_OWNED_OR_READY '+repr(st))
  with tempfile.TemporaryDirectory(prefix='v2_c3_scene_') as tmp: live=probe(Path(tmp),10)
  expected=parent['scene'];mismatch={k:[expected.get(k),live.get(k)] for k in ('scene_sha256','urdf_sha256','srdf_sha256') if expected.get(k)!=live.get(k)}
  if mismatch:raise RuntimeError('SCENE_OR_MODEL_MISMATCH '+repr(mismatch))
  if geo['parent_path_hash']!=path.parent_path_hash:raise RuntimeError('PARENT_HASH_MISMATCH')
  g=parameterize_block(block,path,limits_for(parent))
  if abs(g['duration_s']-summary['A']['G']['metrics']['duration_s'])>1e-9:raise RuntimeError('FROZEN_G_NOT_REPRODUCIBLE')
  result=clean_collision(PlanningSceneChecker().check([block],g['output']));result['stage']='collision';result['actual_scene']=live['scene_sha256']
  if result.get('scene_sha256')!=live['scene_sha256']:
   result.update(status='UNKNOWN',reason='LIVE_SCENE_IDENTITY_CHANGED_OR_MISSING')
 except Exception as e:result.update(status='UNKNOWN',reason=repr(e),traceback=traceback.format_exc())
 write_exclusive(root/'collision_A.json',result);summary['C']['status']='COLLISION_'+result['status'];summary['C']['collision_status']=result['status'];summary['C']['collision_file']=str(root/'collision_A.json');summary['C']['promotion']='NO_PROMOTION_ELIGIBLE'
 if result['status']=='PASS':summary['C']['candidate']='VALID_BUT_NOT_PROMOTED'
 atomic(root/'summary.json',summary);print(json.dumps(result,ensure_ascii=False),flush=True)
if __name__=='__main__':main()
