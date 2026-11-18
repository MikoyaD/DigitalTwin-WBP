"""Fixed P01 feedback integration probe. This does not declare an H or run G."""
import json, traceback
from pathlib import Path
import numpy as np
import yaml
from ..scripts.stage_v2 import load_parent, limits_for
from ..geometry.tm_rho_feedback_prepare import prepare_candidate,make_problem
from ..geometry.tm_rho_feedback_context import initialize,callbacks
from ..geometry.tm_rho_feedback_loop import iterate_center


def main():
    root=Path(__file__).resolve().parents[1]
    parent,path=load_parent(root/'outputs/V2-20261028-224435-single_goal_05-RRTstar')
    parent['experiment_limit_profile']=yaml.safe_load((root/'configs/v2_workspace_single_goal_9d_va_only.yaml').read_text())['constraint_profile']
    output=Path('/home/robot/digitaltwin/research_management/evidence/R5_V2_TM_RHO_FEEDBACK_V1')
    output.mkdir(exist_ok=True)
    file=output/'p01_seed1_integration.json'
    with file.open('x') as f:
        try:
            prepared=prepare_candidate(path,limits_for(parent),1/16,1)
            problem=make_problem(prepared)
            center=initialize(problem)
            if center['status']=='DIAGNOSTIC_INIT':
                propose,evaluate,project=callbacks(problem)
                result=iterate_center(center,propose,evaluate,project)
            else:
                result=dict(status='INITIALIZATION_FAILED',detail=center)
            result['mode_sequence']=[(m['kind'],m['source_start'],m['source_end']) for m in prepared['modes']]
        except Exception as e:
            result=dict(status='ENGINEERING_ERROR',error=repr(e),traceback=traceback.format_exc())
        json.dump(result,f,indent=2,ensure_ascii=False,allow_nan=False,default=lambda x: x.tolist() if isinstance(x,np.ndarray) else float(x) if isinstance(x,np.generic) else str(x))
    print('status:',result['status'],'evidence:',file)
    if result['status']=='ENGINEERING_ERROR': print(result['traceback'])
    return 0 if result['status']!='ENGINEERING_ERROR' else 1

if __name__=='__main__': raise SystemExit(main())
