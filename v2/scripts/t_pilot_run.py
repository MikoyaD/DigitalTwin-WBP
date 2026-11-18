"""Read-only frozen-H pilot solve. Results are diagnostic until validated."""
import argparse,json,time,traceback,hashlib
from pathlib import Path
from ..scripts.tm_feedback_hg import inputs
from ..timing.t_pilot import solve

def main():
 p=argparse.ArgumentParser();p.add_argument('--input',required=True);p.add_argument('--intervals',type=int,default=80);p.add_argument('--solver',default='CLARABEL');p.add_argument('--output',required=True);a=p.parse_args()
 out=Path(a.output);out.mkdir(parents=True,exist_ok=False)
 data=dict(status='FAIL',input=a.input,collision='NOT_ATTEMPTED',numerical_validation='NOT_ATTEMPTED')
 try:
  root,parent,path,geo,lim,block=inputs(a.input)
  data.update(geometry_sha256=hashlib.sha256((root/'geometry.json').read_bytes()).hexdigest(),parent_hash=path.parent_path_hash,limits=lim.metadata['constraint_profile'],source_sha256=hashlib.sha256((Path(__file__).resolve().parents[1]/'timing/t_pilot.py').read_bytes()).hexdigest(),scene=parent.get('scene'))
  print('PILOT_SOLVING',a.intervals,flush=True); data['solve']=solve(block,a.intervals,a.solver); data['status']='SOLVER_ONLY_NOT_VALIDATED';print('PILOT_RESULT',data['solve']['status'],data['solve'].get('objective_s'),flush=True)
 except Exception as e:data['error']=repr(e);(out/'error.log').write_text(traceback.format_exc());print('PILOT_ERROR',repr(e),flush=True)
 (out/'result.json').write_text(json.dumps(data,indent=2,default=str));return 0 if data['status']=='SOLVER_ONLY_NOT_VALIDATED' else 1
if __name__=='__main__':raise SystemExit(main())
