"""Bounded independent whole-body candidate builder; does not run ROS or G."""
import time
import numpy as np
from .tm_rho_feedback_prepare import prepare_candidate,make_problem
from .tm_rho_feedback_context import initialize,callbacks
from .tm_rho_feedback_loop import iterate_center
from .tm_frozen import verify_geometry
from .unified_chart import ExplicitChart
from .unified_block import MotionBlock
from ..core.math_tools import QuinticSpline

CANDIDATES=((1/16,1),(1/16,0),(1/8,0),(1/8,1),(1/4,0),(1/4,1),(1/2,0),(1/2,1))

def finalize(path,limits,prep,problem,center,projections):
    if not projections or any(x['status']!='PROJECTED' for x in projections):
        raise ValueError('FULL_PROJECTION_REQUIRED')
    proj={m['segment']:x for m,x in zip(problem['moves'],projections)}
    if len(proj)!=len(problem['moves']):raise ValueError('PROJECTION_DIMENSION')
    charts=[];audit=[];index={v:i for i,v in enumerate(problem['profile_ids'])}
    for i,seg in enumerate(prep['segments']):
        if seg['kind']=='REGULAR':charts.append(seg['chart']);continue
        a,b=seg['a'],seg['b'];theta=QuinticSpline(np.asarray(center['beta'][index[i]])[:,None]);arm=QuinticSpline(prep['profiles'][i][:,1:])
        if seg['kind']=='ADJUST':
            chart=ExplicitChart(a,b,np.asarray(prep['modes'][seg['mode']]['anchor']),QuinticSpline(np.zeros((6,1))),theta,arm,True)
        else:
            m=next(x for x in problem['moves'] if x['segment']==i);rho=QuinticSpline(np.asarray(proj[i]['controls'])[:,None]);kn=rho.knots;grid=np.unique(np.r_[kn,(kn[:-1]+kn[1:])/2]);grid=grid[(grid>0)&(grid<1)]
            if np.min(rho.evaluate(grid)[:,0])<=0:raise ValueError('MOVE_INTERIOR_STATIONARY')
            for a0,b0 in zip(np.unique(kn)[:-1],np.unique(kn)[1:]):
                if b0>a0 and not np.any((rho.evaluate([(a0+b0)/2])>0)):raise ValueError('MOVE_NONPOSITIVE_SPAN')
            chart=ExplicitChart(a,b,m['p0'],rho,theta,arm,False)
            points=np.unique(np.r_[np.linspace(0,1,5),rho.knots,theta.knots]);error=float(np.max(np.abs(chart.positions(points)-np.vstack([chart._xy(float(x)) for x in points]))))
            if error>1e-8:raise ValueError('INTEGRATION_UNCERTAIN')
            audit.append(dict(segment=i,quadrature_error=error,rho_sample_min=float(np.min(rho.evaluate(grid)))))
        charts.append(chart)
    block=MotionBlock('tm_feedback',tuple(charts),path.schema)
    decision=dict(modes=prep['modes'],progress=prep['u'].tolist(),source_indices=prep['source'].tolist(),epsilon_xy=prep['eps'],candidate='local_%g_seed%d'%(prep['fraction'],prep['seed']))
    decision['reference_tube_max']=verify_geometry(block,path,decision);decision['quadrature']=audit
    for c in charts:
        s=np.unique(np.r_[np.linspace(c.s0,c.s1,1001),prep['u'][(prep['u']>=c.s0)&(prep['u']<=c.s1)]])
        q=c.evaluate_global(s).q
        if np.any(q[:,3:]<limits.arm_lower.value-1e-6) or np.any(q[:,3:]>limits.arm_upper.value+1e-6):raise ValueError('JOINT_POSITION_LIMIT')
    return block,decision

def build(path,limits):
    start=time.perf_counter(); logs=[]
    for fraction,seed in CANDIDATES:
        entry=dict(candidate='local_%g_seed%d'%(fraction,seed),status='FAIL');logs.append(entry)
        try:
            prep=prepare_candidate(path,limits,fraction,seed);problem=make_problem(prep);center=initialize(problem)
            entry['initial']=center
            if center['status']!='DIAGNOSTIC_INIT':entry['reason']='INITIALIZATION_FAILED';continue
            fn=callbacks(problem); result=iterate_center(center,*fn);entry['feedback']=result
            if result['status']!='ALL_PROJECTED_NOT_H':entry['reason']=result['status'];continue
            block,decision=finalize(path,limits,prep,problem,result['center'],result['projections'])
            entry['status']='H_GEOMETRY_PASS';decision['attempts']=logs;decision['h_build_s']=time.perf_counter()-start
            return block,decision
        except Exception as exc:entry['reason']=repr(exc)
    raise RuntimeError('FEEDBACK_CANDIDATES_EXHAUSTED',logs)
