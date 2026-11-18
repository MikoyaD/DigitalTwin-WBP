"""Frozen contract-v2 nine-state MotionBlock, not a legacy PhaseType."""
from dataclasses import dataclass
import numpy as np
from ..core.contracts import PathJet, StateSchema, require
from .unified_chart import ExplicitChart, RegularChart


def _reference_at(grid, q, points):
    return np.column_stack([np.interp(points,grid,q[:,k]) for k in range(9)])


def build_unified(path, limits):
    """Bounded fixed-P prototype: regular motion and exact stationary tail.

    The tail mode is located from source XY at serialization round-off only;
    nonzero millimetre motion is never silently converted to a stop.
    """
    from .unified_chart import (nine_dimensional_progress,fit_regular,
                                fit_tail_profile,fit_tail_rho)
    from ..core.math_tools import QuinticSpline
    q,s,source_indices=nine_dimensional_progress(path.q)
    roundoff=4*np.spacing(max(1.,float(np.max(np.abs(q[:,:2])))))
    last=len(q)-1
    while last>0 and np.max(np.abs(q[last-1,:2]-q[-1,:2]))<=roundoff:
        last-=1
    require(2<last<len(q)-2,'NO_STATIONARY_TAIL','fast prototype needs a true stationary mixed tail')
    require(np.max(np.abs(q[-1]-path.requested_goal))<=1e-6 and
            np.max(np.abs(q[0]-path.requested_start))<=1e-6,
            'INPUT_ENDPOINT_MISMATCH','actual nine-state parent endpoints')
    failures=[]
    for width,dip in ((14,.1),(12,.1),(16,.08),(20,.08),(14,.15),(16,.1),(24,.08),(32,.08)):
        cut=last-width
        if cut<3:continue
        try:
            regular=fit_regular(q[:cut+1],s[:cut+1],limits)
            theta,arm=fit_tail_profile(q[cut:],s[cut:],regular,limits,
                                       controls=18,heading_dip=dip,
                                       stationary_index=last-cut,smoothing=1e-8)
            rho=fit_tail_rho(q[cut:],s[cut:],regular,theta,last-cut,controls=14)
            moving=ExplicitChart(float(s[cut]),float(s[last]),q[cut,:2],rho,theta,arm,
                                 False,(float(s[cut]),1.))
            last_xy=moving.evaluate_global([s[last]]).q[0,:2]
            stationary=ExplicitChart(float(s[last]),1.,last_xy,
                                     QuinticSpline(np.zeros((8,1))),theta,arm,True,
                                     (float(s[cut]),1.))
            block=MotionBlock('block_000',(regular,moving,stationary),path.schema)
            # Independent dense engineering checks, not interval certification.
            worst=np.zeros(3)
            for chart in block.charts:
                samples=np.unique(np.r_[np.linspace(chart.s0,chart.s1,1001),
                                         s[(s>=chart.s0)&(s<=chart.s1)]])
                jet=chart.evaluate_global(samples);ref=_reference_at(s,q,samples)
                deviation=np.max(np.abs(jet.q-ref),axis=0)
                worst=np.maximum(worst,[np.max(deviation[:2]),deviation[2],np.max(deviation[3:])])
                lateral=-np.sin(jet.q[:,2])*jet.qs[:,0]+np.cos(jet.q[:,2])*jet.qs[:,1]
                forward=np.cos(jet.q[:,2])*jet.qs[:,0]+np.sin(jet.q[:,2])*jet.qs[:,1]
                require(np.max(np.abs(lateral))<=1e-8 and np.min(forward)>=-1e-9,
                        'NONHOLONOMIC_NUMERICAL','dense geometry samples')
            require(worst[0]<=.1 and worst[1]<=np.pi/2 and worst[2]<=.1,
                    'REFERENCE_TUBE','dense max XY/theta/arm='+repr(worst.tolist()))
            require(np.max(np.abs(block.start-q[0]))<=1e-6 and
                    np.max(np.abs(block.end-q[-1]))<=1e-6,
                    'BLOCK_ENDPOINT','full nine-state endpoints')
            # Independent QUAD comparison at nontrivial interior positions.
            probe=np.linspace(0,1,11)
            integrated=moving.positions(probe)
            reference=np.vstack([moving._xy(float(v)) for v in probe])
            quadrature_error=float(np.max(np.abs(integrated-reference)))
            require(quadrature_error<=1e-8,'INTEGRATION_UNCERTAIN',str(quadrature_error))
            decision=dict(candidate_id='tail_%d_dip_%g'%(width,dip),stationary_source_index=int(source_indices[last]),
                          regular_cut_source_index=int(source_indices[cut]),
                          reference_tube_max=dict(xy_m=float(worst[0]),theta_rad=float(worst[1]),arm_rad=float(worst[2])),
                          quadrature_crosscheck_m=quadrature_error,
                          derivative_seams='C2_NUMERICAL_CHECKED',
                          source_theta_winding='raw_continuous_steps_lt_pi',
                          failures_before_selection=failures)
            return block,decision
        except Exception as exc:
            failures.append(dict(width=width,dip=dip,error=str(exc)))
    raise ValueError('UNIFIED_H_CANDIDATES_EXHAUSTED: '+repr(failures))


@dataclass(frozen=True)
class MotionBlock:
    block_id: str
    charts: tuple
    schema: StateSchema

    def __post_init__(self):
        require(bool(self.charts),'EMPTY_BLOCK','at least one continuous chart')
        require(abs(self.charts[0].s0)<1e-12 and abs(self.charts[-1].s1-1)<1e-12,
                'BLOCK_COVERAGE','one global [0,1] interval')
        for left,right in zip(self.charts[:-1],self.charts[1:]):
            require(abs(left.s1-right.s0)<1e-12,'CHART_GAP','charts must meet')
            a=left.evaluate_global([left.s1]);b=right.evaluate_global([right.s0])
            for name,tol in (('q',1e-6),('qs',1e-6),('qss',1e-6)):
                require(np.max(np.abs(getattr(a,name)-getattr(b,name)))<=tol,
                        'CHART_C2_SEAM',name+' mismatch')

    @property
    def phase_id(self):return self.block_id

    @property
    def start(self):return self.evaluate([0.]).q[0]

    @property
    def end(self):return self.evaluate([1.]).q[0]

    def evaluate(self,u):
        u=np.atleast_1d(np.asarray(u,float))
        require(np.isfinite(u).all() and np.all((u>=0)&(u<=1)),
                'BLOCK_PARAMETER','[0,1]')
        arrays=[np.empty((len(u),9)) for _ in range(4)]
        for k,chart in enumerate(self.charts):
            # Right chart owns the boundary; both one-sided jets checked in init.
            mask=(u>=chart.s0) & (u<=chart.s1 if k==len(self.charts)-1 else u<chart.s1)
            if not np.any(mask):continue
            jet=chart.evaluate_global(u[mask])
            for output,name in zip(arrays,('q','qs','qss','qsss')):
                output[mask]=getattr(jet,name)
        return PathJet(*arrays)

    def freeze(self,path,decision,model_scene):
        return dict(contract_version=2,parent_path_hash=path.parent_path_hash,
                    schema=dict(joint_names=list(path.schema.joint_names),position_units=list(path.schema.position_units),
                                velocity_units=list(path.schema.velocity_units),acceleration_units=list(path.schema.acceleration_units),
                                jerk_units=list(path.schema.jerk_units),frame_id=path.schema.frame_id,
                                reference_point=path.schema.reference_point,time_unit=path.schema.time_unit),
                    progress=dict(kind='normalized_nine_dimensional_chord',scales=[1.]*9),
                    forward_only=True,blocks=[dict(id=self.block_id,start=self.start.tolist(),end=self.end.tolist(),
                                                   rest_to_rest=True,charts=[c.freeze() for c in self.charts])],
                    identities=model_scene,construction=dict(algorithm_version='unified_hg_fast_v1',
                    candidate=decision,geometry_status='NUMERICAL_CHECKED',collision_status='NOT_CHECKED_AT_H'))


def thaw_v2(record):
    require(record.get('contract_version')==2,'GEOMETRY_VERSION','v2 contract')
    schema=StateSchema(**record['schema']);blocks=[]
    for item in record['blocks']:
        charts=[]
        for c in item['charts']:
            charts.append(RegularChart.thaw(c) if c['type']=='REGULAR_XY_QUINTIC' else ExplicitChart.thaw(c))
        block=MotionBlock(item['id'],tuple(charts),schema)
        require(np.max(np.abs(block.start-np.asarray(item['start'])))<1e-6 and
                np.max(np.abs(block.end-np.asarray(item['end'])))<1e-6,
                'BLOCK_ENDPOINT','frozen endpoints')
        blocks.append(block)
    require(bool(blocks),'EMPTY_GEOMETRY','at least one block')
    return dict(blocks=blocks,construction=record['construction'])
