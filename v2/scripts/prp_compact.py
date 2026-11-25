"""Finalize one owned YAML run without retaining redundant dense intermediates."""
import json,os,shutil
from pathlib import Path
import numpy as np
from ..core import array_store as store

BUDGET=30*1024*1024

def _copy(src,dst):
 if not src.is_file():return False
 dst.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(src,dst);return True

def _measure(root,exclude_execution=False):
 logical=allocated=0;seen=set()
 for p in root.rglob('*'):
  if exclude_execution and 'Execution' in p.relative_to(root).parts:continue
  if p.is_symlink():raise ValueError('SYMLINK_IN_OUTPUT: '+str(p))
  if p.is_file() and 'plots' not in p.relative_to(root).parts:
   st=p.stat();logical+=st.st_size;key=(st.st_dev,st.st_ino)
   if key not in seen:allocated+=st.st_blocks*512;seen.add(key)
 return logical,allocated

def finalize(root,work,stage,source_r=None,retain_dense=False):
 """Copy unique deliverables first; remove only this invocation's work tree last."""
 root=Path(root).resolve();work=Path(work).resolve()
 if work.parent!=root or work.name!='.work' or not work.is_dir():raise ValueError('INVALID_OWNED_WORK_DIRECTORY')
 out=root;(out/'Execution').mkdir(exist_ok=True);result=work/'result';pr=result/'PRP' if stage in ('P','new') else result
 # New input P is produced in work/P; saved P is contained in R's parent JSON.
 parent=work/'P/parent_path.json' if stage=='new' else result/'R/parent_path.json' if stage=='P' else work/'R_input/parent_path.json'
 _copy(parent,out/'ParentPath/parent_path.json')
 rdir=result/'R' if stage in ('P','new') else work/'R_input'
 geo=rdir/'geometry.json';record=None
 if geo.is_file():
  record=json.loads(geo.read_text());candidate=Path(record.get('signed_arrays',record.get('signed_npz')))
  with store.load(candidate) as source:store.save(out/'Reconstruction/candidate.arrays',**{k:source[k] for k in source.files})
  record.pop('signed_npz',None);record['signed_arrays']=str((out/'Reconstruction/candidate.arrays').resolve());(out/'Reconstruction/geometry.json').write_text(json.dumps(record,indent=2))
  _copy(rdir/'parent_path.json',out/'Reconstruction/parent_path.json');_copy(rdir/'verification.json',out/'Reconstruction/verification.json')
 else:_copy(rdir/'verification.json',out/'Reconstruction/verification.json')
 for f in ('result.json','error.log'):_copy(result/f,out/f)
 for f in ('result.json','error.log','collision.json','geometry_collision.json','scene_snapshot.bin'):_copy(pr/f,out/'Parameterization'/f)
 for f in ('result.json','numerical_result.json','trajectory.csv'):_copy(pr/'P0/G'/f,out/'Parameterization/P0/G'/f)
 _copy(pr/'P0/cases/seg01/C/verification.json',out/'Parameterization/P0/cases/seg01/C/verification.json')
 for f in ('A/fast_solver.json','A/smooth_solver.json','C/solver.json'):
  _copy(pr/'P0/cases/seg01'/f,out/'Parameterization/P0/cases/seg01'/f)
 if record:
  p=out/'Parameterization/P0/geometry.json';p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(record,indent=2))
  _copy(out/'Reconstruction/parent_path.json',out/'Parameterization/P0/parent_path.json')
 full=pr/'P0/G/trajectory.arrays';thin=out/'Parameterization/P0/G/time_law.arrays'
 if full.is_dir():
  with store.load(full) as z:
   keys=('g1_density_knots','density_x','active_intervals','density_beta','normalized_nodes','g0_density_knots','g0_density_values','g1_density_values')
   thin.parent.mkdir(parents=True,exist_ok=True);store.save(thin,**{k:z[k] for k in keys if k in z.files})
  p=out/'Parameterization/P0/G/result.json'
  if p.is_file():
   data=json.loads(p.read_text());data.pop('time_law_npz',None);data['time_law_arrays']=str(thin);data['numeric_verification']=str(out/'Parameterization/P0/cases/seg01/C/verification.json');p.write_text(json.dumps(data,indent=2))
 for f in ('summary.json','recommended_law.json','fast_law.json','error.log'):_copy(pr/'P1'/f,out/'Parameterization/P1'/f)
 for f in ('trajectory.csv','result.json','events.json'):_copy(pr/'P1/recommended'/f,out/'Parameterization/P1/recommended'/f)
 if retain_dense and (pr/'P1/recommended/trajectory.arrays').is_dir():
  (root/'.dense').mkdir();shutil.copytree(pr/'P1/recommended/trajectory.arrays',root/'.dense/recommended.arrays')
 for f in ('trajectory.csv','result.json','events.json'):_copy(pr/'P1/fast'/f,out/'Parameterization/P1/fast'/f)
 for f in ('IK/result.json','IK/target.json','IK/p_endpoint_fk.json','IK/final_fk.json','P/error.log','error.log'):_copy(work/f,out/('ParentPath/error.log' if f=='P/error.log' else f))
 # Rewrite retained descriptor paths to the final location. Keep status and
 # numerical evidence, but never leave a time-law pointer into deleted work.
 def rewrite(x):
  if isinstance(x,dict):return {k:rewrite(v) for k,v in x.items()}
  if isinstance(x,list):return [rewrite(v) for v in x]
  if isinstance(x,str):
   return x.replace(str(pr/'P0/G/trajectory.arrays'),str(thin)).replace(str(pr),str(out/'Parameterization')).replace(str(rdir),str(out/'Reconstruction')).replace(str(work/'P'),str(out/'ParentPath')).replace(str(work/'IK'),str(out/'IK')).replace(str(result/'IK'),str(out/'IK')).replace(str(result/'result.json'),str(out/'result.json'))
  return x
 for p in out.rglob('*.json'):
  if work in p.parents:continue
  try:data=json.loads(p.read_text());p.write_text(json.dumps(rewrite(data),ensure_ascii=False,indent=2))
  except (ValueError,UnicodeError):pass
 # A required full P1 or geometry file is never dropped to satisfy the budget.
 # CSV is optional only if it is reproducible from the retained laws/geometry.
 storage=root/'storage.json';storage.write_text(json.dumps({'logical_bytes':'00000000000000000000','allocated_bytes':'00000000000000000000','budget_bytes':BUDGET,'csv_omitted':False,'status':'PENDING'},indent=2))
 # Do not count the transient work tree, which remains recoverable until after
 # all deliverables have been copied and validated.
 logical=allocated=0;seen=set()
 for p in root.rglob('*'):
  if work in p.parents or p==work or not p.is_file() or 'plots' in p.relative_to(root).parts or 'Execution' in p.relative_to(root).parts:continue
  if p.is_symlink():raise ValueError('SYMLINK_IN_OUTPUT: '+str(p))
  st=p.stat();logical+=st.st_size;key=(st.st_dev,st.st_ino)
  if key not in seen:allocated+=st.st_blocks*512;seen.add(key)
 # The final CSV is now a required deliverable, not a disposable cache.
 # Report an exceeded budget instead of removing the only display trajectory.
 omitted=False
 # Work contains only data produced by this invocation and is removed only
 # after all final deliverables have been successfully copied.
 shutil.rmtree(work)
 logical,allocated=_measure(root,exclude_execution=True);status='PASS' if max(logical,allocated)<=BUDGET else 'OUTPUT_BUDGET_EXCEEDED'
 data={'logical_bytes':f'{logical:020d}','allocated_bytes':f'{allocated:020d}','budget_bytes':BUDGET,'csv_omitted':omitted,'status':status,'scope':'planning directory excluding plots and Execution; allocated counts each inode once'}
 # fixed-width accounting ensures rewriting does not change file length.
 storage.write_text(json.dumps(data,indent=2));logical2,allocated2=_measure(root,exclude_execution=True)
 data['logical_bytes']=f'{logical2:020d}';data['allocated_bytes']=f'{allocated2:020d}';data['status']='PASS' if max(logical2,allocated2)<=BUDGET else 'OUTPUT_BUDGET_EXCEEDED';storage.write_text(json.dumps(data,indent=2))
 return data
