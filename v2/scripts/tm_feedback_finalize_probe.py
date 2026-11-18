"""Validate P01 projected feedback candidate as a whole; never run G."""
import json,traceback
from pathlib import Path
import numpy as np,yaml
from ..scripts.stage_v2 import load_parent,limits_for
from ..geometry.tm_rho_feedback_prepare import prepare_candidate,make_problem
from ..geometry.tm_frozen import verify_geometry
from ..geometry.unified_chart import ExplicitChart
from ..geometry.unified_block import MotionBlock
from ..core.math_tools import QuinticSpline


def main():
 root=Path(__file__).resolve().parents[1]
 evidence=Path('/home/robot/digitaltwin/research_management/evidence/R5_V2_TM_RHO_FEEDBACK_V1')
 saved=json.loads((evidence/'p01_seed1_integration.json').read_text())
 out=evidence/'p01_seed1_geometry_validation.json'
 with out.open('x') as f:
  try:
   assert saved['status']=='ALL_PROJECTED_NOT_H'
   parent,path=load_parent(root/'outputs/V2-20261028-224435-single_goal_05-RRTstar')
   parent['experiment_limit_profile']=yaml.safe_load((root/'configs/v2_workspace_single_goal_9d_va_only.yaml').read_text())['constraint_profile']
   prep=prepare_candidate(path,limits_for(parent),1/16,1);problem=make_problem(prep)
   beta=saved['center']['beta']; projected={m['segment']:p for m,p in zip(problem['moves'],saved['projections'])}
   charts=[]; probe=[]; idx={x:i for i,x in enumerate(problem['profile_ids'])}
   for i,seg in enumerate(prep['segments']):
    if seg['kind']=='REGULAR':charts.append(seg['chart']);continue
    controls=prep['profiles'][i]; theta=QuinticSpline(np.asarray(beta[idx[i]])[:,None]);arm=QuinticSpline(controls[:,1:]);a,b=seg['a'],seg['b']
    if seg['kind']=='ADJUST':
     chart=ExplicitChart(a,b,np.asarray(prep['modes'][seg['mode']]['anchor']),QuinticSpline(np.zeros((6,1))),theta,arm,True)
    else:
     m=next(v for v in problem['moves'] if v['segment']==i); c=np.asarray(projected[i]['controls']);rho=QuinticSpline(c[:,None]); grid=np.unique(np.r_[rho.knots,(rho.knots[:-1]+rho.knots[1:])/2]);grid=grid[(grid>0)&(grid<1)]
     if np.min(rho.evaluate(grid)[:,0])<=0: raise ValueError('MOVE_INTERIOR_STATIONARY segment=%s'%i)
     chart=ExplicitChart(a,b,m['p0'],rho,theta,arm,False)
     points=np.unique(np.r_[np.linspace(0,1,5),rho.knots,theta.knots]); err=np.max(np.abs(chart.positions(points)-np.vstack([chart._xy(float(x)) for x in points])))
     if err>1e-8:raise ValueError('INTEGRATION_UNCERTAIN %s'%err)
     probe.append(dict(segment=i,quadrature_error=float(err),rho_min=float(np.min(rho.evaluate(grid)[:,0]))))
    charts.append(chart)
   block=MotionBlock('tm_feedback',tuple(charts),path.schema)
   decision=dict(modes=prep['modes'],progress=prep['u'].tolist(),source_indices=prep['source'].tolist())
   worst=verify_geometry(block,path,decision)
   # Check configured joint limits independently of sampled reference constraints.
   lim=limits_for(parent)
   for c in charts:
    s=np.unique(np.r_[np.linspace(c.s0,c.s1,1001),prep['u'][(prep['u']>=c.s0)&(prep['u']<=c.s1)]])
    q=c.evaluate_global(s).q
    if np.any(q[:,3:]<lim.arm_lower.value-1e-6) or np.any(q[:,3:]>lim.arm_upper.value+1e-6):raise ValueError('JOINT_LIMIT')
   result=dict(status='GEOMETRY_PASS_NOT_FROZEN',worst_reference=worst,quadrature=probe)
  except Exception as e:result=dict(status='GEOMETRY_FAIL',error=repr(e),traceback=traceback.format_exc())
  json.dump(result,f,ensure_ascii=False,indent=2,allow_nan=False)
 print(result)
if __name__=='__main__':main()
