"""Dense fixed-H velocity/acceleration check, not full acceptance."""
import argparse,json
from pathlib import Path
import numpy as np
from ..scripts.tm_feedback_hg import inputs
from ..timing.t_pilot import matrices

def check(result,subdivisions):
 d=json.loads(Path(result).read_text());s=d['solve'];_,_,_,_,_,block=inputs(d['input'])
 if 'coefficients' not in s:return dict(status='UNKNOWN',reason='NO_SOLVER_COEFFICIENTS')
 g=np.asarray(s['grid']);u=np.unique(np.r_[np.concatenate([a+(b-a)*np.linspace(.0001,.9999,subdivisions) for a,b in zip(g[:-1],g[1:])]),g[1:-1]])
 A,D=matrices(u,np.asarray(s['knots']));c=np.asarray(s['coefficients']);b=.001*(A@c);bp=.001*(D@c)
 if np.any(b<=0):return dict(status='FAIL',reason='NONPOSITIVE_INTERNAL_B',min_b=float(np.min(b)))
 j=block.evaluate(u);v=np.abs(j.qs)*np.sqrt(b)[:,None];acc=np.abs(j.qss*b[:,None]+.5*j.qs*bp[:,None]);iv=np.unravel_index(np.argmax(v),v.shape);ia=np.unravel_index(np.argmax(acc),acc.shape)
 return dict(status='PASS_SAMPLED_ONLY' if v[iv]<=.5 and acc[ia]<=1 else 'FAIL',points=len(u),max_velocity=float(v[iv]),velocity_channel=int(iv[1]),velocity_u=float(u[iv[0]]),max_acceleration=float(acc[ia]),acceleration_channel=int(ia[1]),acceleration_u=float(u[ia[0]]),b_min=float(b.min()),b_max=float(b.max()),note='Not independent full numerical validation or collision acceptance')
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('result');p.add_argument('--subdivisions',type=int,default=25);p.add_argument('--output',required=True);a=p.parse_args();data=check(a.result,a.subdivisions);
 with Path(a.output).open('x') as f: json.dump(data,f,indent=2)
 print(json.dumps(data))
