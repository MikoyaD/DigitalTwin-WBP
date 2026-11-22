"""Construct signed R from a saved parent, then use the V2-PRP P0/P1 runner."""
import argparse,json,shutil,time,traceback
from pathlib import Path
import numpy as np
from ..core import array_store as store
from .stage_v2 import load_parent,limits_for
from ..geometry.signed_reconstruction import solve,verify
from ..geometry.signed_frozen import Geometry
from ..geometry.tm_rho_feedback_frozen import thaw
from ..geometry.tm_frozen import verify_geometry
from . import prp_from_r


def main(argv=None):
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--parent',required=True);p.add_argument('--out',required=True);p.add_argument('--until',choices=('r','g0','g1'),default='g1');p.add_argument('--scene-snapshot');p.add_argument('--scene-map');p.add_argument('--compact-output',action='store_true');p.add_argument('--rho-controls',type=int,default=22,help='number of rho spline controls (default: 22)');p.add_argument('--rho-fallback',type=int,choices=[36],help='retry exact infeasibility with 36 controls and the same profile');p.add_argument('--signed-recovery',action='store_true',help='bounded theta/rho recovery after exact fixed-profile infeasibility');a=p.parse_args(argv)
 src=Path(a.parent).resolve(strict=True);src=src.parent if src.is_file() and src.name=='parent_path.json' else src
 out=Path(a.out).resolve();out.mkdir(parents=True,exist_ok=False);res={'status':'R_STARTED','input_P':str(src),'collision':'NOT_ATTEMPTED'};t=time.perf_counter()
 def save():res['wall_s']=time.perf_counter()-t;(out/'result.json').write_text(json.dumps(res,indent=2))
 try:
  parent,path=load_parent(src);lim=limits_for(parent);folder=out/'R';folder.mkdir();shutil.copyfile(src/'parent_path.json',folder/'parent_path.json')
  if a.scene_map:
   from .runtime_v2 import PACKAGE
   if Path(a.scene_map).resolve(strict=True)!=(PACKAGE/parent['config']['static_octomap']).resolve(strict=True):raise ValueError('SAVED_PARENT_SCENE_MAP_MISMATCH')
  start=time.perf_counter()
  try:data,info,(q,u)=solve(path,lim,rho_controls=a.rho_controls,rho_fallback=a.rho_fallback)
  except RuntimeError as exc:
   if not a.signed_recovery or str(exc)!='RHO_QP_infeasible':raise
   res['fixed_profile_attempts']=getattr(exc,'rho_attempts',[]);save()
   from ..geometry.signed_recovery import solve as recover
   data,info,(q,u)=recover(path,lim)
  res['R_solver']=info;store.save(folder/'candidate.arrays',**data);check=verify(data,q,u,lim);(folder/'verification.json').write_text(json.dumps(check,indent=2));res['R_verification']=check;res['R_wall_s']=time.perf_counter()-start
  if check['status']!='PASS':raise ValueError('SIGNED_R_GEOMETRY_NOT_PASS')
  geom=Geometry(data);modes=[dict(u0=float(c.a),u1=float(c.b),kind='MOVE' if c.sign else 'ADJUST',direction=int(c.sign),anchor=c.anchor.tolist()) for c in geom.cs]
  rec={'contract_version':4,'algorithm_variant':'V2-SIGNED-H-v1','signed_arrays':str((folder/'candidate.arrays').resolve()),'schema':parent['path']['schema'],'start':q[0].tolist(),'end':q[-1].tolist(),'construction':{'progress':u.tolist(),'modes':modes,'h_build_s':res['R_wall_s']}}
  (folder/'geometry.json').write_text(json.dumps(rec,indent=2));block=thaw(rec);res['R_formal_reference_tube']=verify_geometry(block,path,rec['construction']);res['status']='SIGNED_R_NUMERICAL_PASS';save();print('R_RESULT PASS',res['R_wall_s'],flush=True)
  if a.until=='r':return 0
  cmd=['--signed-r',str(folder),'--out',str(out/'PRP'),'--until',a.until]
  if a.scene_snapshot:cmd+=['--scene-snapshot',a.scene_snapshot]
  if a.scene_map:cmd+=['--scene-map',a.scene_map]
  if a.compact_output:cmd+=['--compact-output']
  code=prp_from_r.main(cmd);pr=json.loads((out/'PRP/result.json').read_text());res['PRP']=pr;res['collision']=pr.get('collision','NOT_ATTEMPTED');res['status']=pr['status'];save();return code
 except Exception as e:
  res['status']='R_FAIL' if res['status']=='R_STARTED' else 'DOWNSTREAM_FAIL';res['error']=repr(e);res['rho_attempts']=getattr(e,'rho_attempts',[]);(out/'error.log').write_text(traceback.format_exc());save();print('PRP_RESULT',res['status'],repr(e),flush=True);return 1
if __name__=='__main__':raise SystemExit(main())
