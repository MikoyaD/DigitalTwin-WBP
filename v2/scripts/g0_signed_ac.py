"""Signed-H G0: uniform A then nested C, without quality fallback."""
import json,os,shutil,subprocess,sys,time
from pathlib import Path
import numpy as np
from ..core import array_store as store
from .stage_v2 import load_parent,limits_for,HEADER
from ..geometry.tm_rho_feedback_frozen import thaw
from ..geometry.tm_frozen import verify_geometry

def write_trajectory_csv(folder,block,geo,output_path=None):
 """Save original C rows at event-aware display density; never change C."""
 from .g1_display_sampling import select_storage_rows
 from ..timing.g0_ac_geometry import chart_events
 with store.load((Path(folder)/'trajectory.arrays') if (Path(folder)/'trajectory.arrays').is_dir() else (Path(folder)/'trajectory.npz'),allow_pickle=False) as z:
  values=np.column_stack([z[k] for k in ('t','q','velocity','acceleration','jerk')])
  t=z['t'];u=z['u'];chart=z['chart'];direction=z['direction'];base=z['base_s'];indices={0,len(t)-1}
  change=np.flatnonzero((chart[1:]!=chart[:-1])|(direction[1:]!=direction[:-1]))+1
  for i in change:indices.update((int(i-1),int(i)))
  for value in z['normalized_nodes']:
   i=int(np.argmin(abs(base-value)))
   if abs(base[i]-value)>1e-10:raise ValueError('P0_DENSITY_EVENT_MISSING')
   indices.add(i)
  for value in np.unique(np.concatenate([np.asarray(geo['construction']['progress'])]+[chart_events(c) for c in block.charts])):
   i=int(np.argmin(abs(u-value)))
   if abs(u[i]-value)>1e-9:raise ValueError('P0_GEOMETRY_EVENT_MISSING')
   indices.add(i)
  selector=np.column_stack((values,u,chart,np.where(direction==0,0,1),direction));ix=select_storage_rows(selector,t[sorted(indices)])
  np.savetxt(Path(output_path) if output_path else Path(folder)/'trajectory.csv',values[ix],delimiter=',',fmt='%.17g',header=','.join(HEADER),comments='')
  return {'source_rows':len(t),'rows':len(ix),'events':len(indices),'sampling':'original rows; time-law and geometry events, extrema, 50 Hz target and position-error refinement'}

def run(source,out,defer_collision=False,numeric_only=False,reserved=False,compact_output=False):
 source=Path(source).resolve(); out=Path(out).resolve();start=time.perf_counter()
 if not (defer_collision or numeric_only):raise ValueError('SIGNED_G0_REQUIRES_DEFERRED_COLLISION')
 parent,path=load_parent(source);geo=json.loads((source/'geometry.json').read_text());limits=limits_for(parent)
 if limits.metadata['constraint_profile']['identity']!='V2_NINE_DOF_VA_ONLY':raise ValueError('G0_LIMIT_CONTRACT')
 block=thaw(geo);verify_geometry(block,path,geo['construction'])
 if reserved:
  if not out.is_dir() or any(out.iterdir()):raise ValueError('SIGNED_G0_DEST_NOT_EMPTY')
 else:out.mkdir(parents=True,exist_ok=False)
 (out/'solver').mkdir();
 for name in ('parent_path.json','geometry.json'):shutil.copyfile(source/name,out/name)
 if geo.get('algorithm_variant')=='V2-SIGNED-H-v1':
  h_path=Path(geo.get('signed_arrays',geo.get('signed_npz'))).resolve()
 else:
  # Read the original frozen evaluator directly; no refitting or sampled approximation.
  h_path=out/'legacy_geometry_adapter.arrays';store.save(h_path,legacy_geometry_file=str((source/'geometry.json').resolve()),global_u=np.array([block.charts[0].s0]+[c.s1 for c in block.charts]),directions=np.array([0 if getattr(c,'stationary',False) else 1 for c in block.charts]))
 case={'H':str(h_path),'parent':str(source),'geometry_evidence':'prevalidated frozen H'};(out/'solver/cases.json').write_text(json.dumps({'1':case},indent=2))
 env=dict(os.environ,V2_G0_AC_OUTPUT=str(out),OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1'); commands=[('A',['1','A']),('C',['1','C']),('verify',['1','C'])];modules=['g0_ac_a','g0_ac_c','g0_ac_verify'];runs=[]
 for (name,args),module in zip(commands,modules):
  st=time.perf_counter();p=subprocess.run([sys.executable,'-B','-m','v2.timing.'+module,*args],env=env,capture_output=True,text=True);(out/(name+'.log')).write_text(p.stdout+p.stderr);runs.append({'stage':name,'wall_s':time.perf_counter()-st,'returncode':p.returncode})
  if p.returncode:raise RuntimeError('SIGNED_G0_'+name+'_FAILED: '+p.stderr[-1500:])
 cand=out/'cases/seg01/C/candidate.arrays';verify=json.loads((out/'cases/seg01/C/verification.json').read_text())
 if verify['status']!='PASS':raise ValueError('SIGNED_G0_C_NUMERICAL_FAILURE')
 folder=out/'G';folder.mkdir()
 if compact_output:
  # Both names belong to this new output tree; retain the usual G entry point
  # without writing a second dense file. They must be treated as immutable.
  shutil.copytree(cand,folder/'trajectory.arrays')
 else:
  shutil.copytree(cand,folder/'trajectory.arrays')
 csv=write_trajectory_csv(folder,block,geo)
 r={'status':'NUMERICAL_PASS_COLLISION_PENDING','full_g':'PENDING_COLLISION','collision':{'status':'NOT_ATTEMPTED'},'duration_s':verify['duration_s'],'time_law_type':'NESTED_C_V1','time_law_arrays':str((folder/'trajectory.arrays').resolve()),'algorithm':'signed_A_then_nested_C','compute_s':time.perf_counter()-start,'stages':runs,'numeric_verification':str((out/'cases/seg01/C/verification.json').resolve()),'compact_output':compact_output,'csv':csv};(folder/'result.json').write_text(json.dumps(r,indent=2));(folder/'numerical_result.json').write_text(json.dumps(dict(r,numeric_details=verify),indent=2));return out,r
