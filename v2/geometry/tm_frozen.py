"""Independent contract 3; legacy geometry contracts remain untouched."""
import numpy as np
from .unified_block import MotionBlock
from .unified_chart import ExplicitChart,RegularChart
from ..core.contracts import StateSchema,require
from dataclasses import asdict
from pathlib import Path
from hashlib import sha256

def freeze(block,path,decision,parent,limits):
    root=Path(__file__).resolve().parents[1]
    paths=('geometry/tm_modes.py','geometry/tm_builder.py','geometry/tm_local.py',
           'geometry/tm_frozen.py','geometry/unified_chart.py','geometry/unified_block.py',
           'timing/tm_g.py','core/math_tools.py','validation/numerical.py')
    source_identity={name:sha256((root/name).read_bytes()).hexdigest() for name in paths}
    return dict(source_identity=source_identity,contract_version=3,algorithm_variant='V2-TM-HG',parent_path_hash=path.parent_path_hash,
                schema=asdict(path.schema),charts=[c.freeze() for c in block.charts],
                start=block.start.tolist(),end=block.end.tolist(),construction=decision,
                scene=parent['scene'],sources=parent['sources'],
                effective_limit_identity=limits.metadata['constraint_profile']['effective_limit_identity'])

def thaw(record):
    require(record.get('contract_version')==3 and record.get('algorithm_variant')=='V2-TM-HG',
            'TM_CONTRACT','v3 TM required')
    schema=StateSchema(**record['schema']);charts=[]
    for c in record['charts']:
        if c['type']=='REGULAR_XY_QUINTIC': charts.append(RegularChart.thaw(c))
        elif c['type']=='EXPLICIT_HEADING_INTEGRAL': charts.append(ExplicitChart.thaw(c))
        else: raise ValueError('TM_UNKNOWN_CHART '+str(c.get('type')))
    block=MotionBlock('tm_000',tuple(charts),schema)
    require(np.max(abs(block.start-np.asarray(record['start'])))<1e-6 and
            np.max(abs(block.end-np.asarray(record['end'])))<1e-6,'TM_FROZEN_ENDPOINT','mismatch')
    return block

def verify_geometry(block,path,decision):
    """Independently recheck mode semantics, reference tube and nonholonomy."""
    from .unified_chart import nine_dimensional_progress
    from .unified_block import _reference_at
    q,u,_=nine_dimensional_progress(path.q);modes=decision['modes'];worst=np.zeros(3)
    require(np.allclose(u,decision['progress'],atol=1e-14,rtol=0),'TM_PROGRESS','parent mismatch')
    for chart in block.charts:
        x=np.unique(np.r_[np.linspace(chart.s0,chart.s1,1001),u[(u>=chart.s0)&(u<=chart.s1)]])
        jet=chart.evaluate_global(x);ref=_reference_at(u,q,x);deviation=np.max(abs(jet.q-ref),axis=0)
        worst=np.maximum(worst,[max(deviation[:2]),deviation[2],max(deviation[3:])])
        mid=(chart.s0+chart.s1)/2;matched=[m for m in modes if m['u0']<=mid<=m['u1']]
        require(len(matched)==1,'TM_MODE_MAPPING','chart does not map to exactly one mode')
        mode=matched[0]
        require(chart.s0>=mode['u0']-1e-12 and chart.s1<=mode['u1']+1e-12,'TM_MODE_MAPPING','chart crosses mode')
        if mode['kind']=='ADJUST':
            require(getattr(chart,'stationary',False) and np.max(abs(jet.q[:,:2]-mode['anchor']))<1e-9 and
                    np.max(abs(jet.qs[:,:2]))<1e-10 and np.max(abs(jet.qss[:,:2]))<1e-10,
                    'TM_ADJUST_XY','not stationary')
            require(np.max(np.ptp(jet.q[:,2:],axis=0))>1e-10,'TM_ADJUST_NO_MOTION','no adjustment')
        else:
            # Open MOVE interval: positive forward speed; a true ADJUST boundary may be zero.
            interior=jet.qs[1:-1,:2];theta=jet.q[1:-1,2]
            forward=np.cos(theta)*interior[:,0]+np.sin(theta)*interior[:,1]
            lateral=-np.sin(theta)*interior[:,0]+np.cos(theta)*interior[:,1]
            direction=getattr(chart,'direction',1)
            require(direction in (-1,1) and np.all(direction*forward>0) and np.max(abs(lateral))<1e-8,'TM_MOVE_KINEMATICS','wrong direction/slip')
    require(worst[0]<=.1 and worst[1]<=np.pi/2 and worst[2]<=.1,'TM_REFERENCE_TUBE',str(worst))
    require(np.max(abs(block.start-q[0]))<1e-6 and np.max(abs(block.end-q[-1]))<1e-6,'TM_PARENT_ENDPOINT','mismatch')
    return worst.tolist()
