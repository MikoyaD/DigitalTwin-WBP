"""Pure numerical checks for the separately named C3 experiment."""
import json
from pathlib import Path
import numpy as np
from v2.geometry.c3_transition import restrict_prefix, make_candidate,reference_tube_extrema
from v2.geometry.unified_block import thaw_v2
from v2.scripts.stage_v2 import load_parent, limits_for

ROOT=Path(__file__).resolve().parents[1]/'outputs/V2-20261026-214202-single_goal_05_UHG-RRTstar'

def source():
    parent,path=load_parent(ROOT)
    block=thaw_v2(json.loads((ROOT/'geometry.json').read_text()))['blocks'][0]
    return parent,path,block

def test_prefix_exact():
    _,_,block=source();old=block.charts[0];end=old.s1*.96;new=restrict_prefix(old,end);grid=np.linspace(0,end,101)
    for name in ('q','qs','qss','qsss'):
        assert np.max(abs(getattr(old.evaluate_global(grid),name)-getattr(new.evaluate_global(grid),name)))<1e-6

def test_original_entry_c3_candidate():
    parent,path,old=source();block,info=make_candidate(path,limits_for(parent),old,old.charts[0].s1)
    assert max(max(v.values()) for v in info['seams'].values())<1e-6
    assert np.max(abs(block.start-path.q[0]))<1e-6 and np.max(abs(block.end-path.q[-1]))<1e-6
    assert reference_tube_extrema(block,path)['status']=='NUMERICAL_CHECKED_NOT_CERTIFIED'
    s=np.linspace(block.charts[-1].s0,1,51);jet=block.evaluate(s)
    assert np.max(abs(jet.qs[:,:2]))==0 and np.max(abs(jet.qs[:,2]))>0 and np.max(abs(jet.qs[:,3:]))>0
    restored=thaw_v2(block.freeze(path,dict(candidate_id='C3_TEST'),parent['scene']))['blocks'][0]
    for key in ('q','qs','qss','qsss'):
        assert np.array_equal(getattr(block.evaluate(s),key),getattr(restored.evaluate(s),key))

def test_analytic_geometric_derivatives_interior():
    parent,path,old=source();block,_=make_candidate(path,limits_for(parent),old,old.charts[0].s1)
    for chart in block.charts:
        u=chart.s0+(chart.s1-chart.s0)*.43;h=1e-7*(chart.s1-chart.s0)
        minus=chart.evaluate_global([u-h]);plus=chart.evaluate_global([u+h]);center=chart.evaluate_global([u])
        for low,high in [('q','qs'),('qs','qss'),('qss','qsss')]:
            fd=(getattr(plus,low)-getattr(minus,low))/(2*h);analytic=getattr(center,high)
            assert np.allclose(fd,analytic,atol=1e-3,rtol=2e-5),(low,high,np.max(abs(fd-analytic)))

def test_c3_rolling_map_matches_original():
    from v2.core.math_tools import quintic_time_jet,chain_rule,differential_kinematics
    parent,path,old=source();block,_=make_candidate(path,limits_for(parent),old,old.charts[0].s1)
    T=73.27969652156213;t=np.linspace(T*.74,T*.80,101);u,v,a,j=quintic_time_jet(t,T);jet=block.evaluate(u);vel,acc,_=chain_rule(jet,v,a,j)
    kin=differential_kinematics(jet.q,vel,acc,.06,.3935)
    theta=jet.q[:,2];rho=np.cos(theta)*jet.qs[:,0]+np.sin(theta)*jet.qs[:,1]
    rho_u=np.cos(theta)*jet.qss[:,0]+np.sin(theta)*jet.qss[:,1]
    expected=np.column_stack([(rho_u*v*v+rho*a+sign*.3935/2*(jet.qss[:,2]*v*v+jet.qs[:,2]*a))/.06 for sign in (-1,1)])
    assert np.allclose(expected,kin['wheel_acceleration'],atol=1e-9,rtol=1e-9)

def test_b_physical_operator_matrices_match_geometry_and_wheels():
    from scipy.optimize import brentq
    from v2.geometry.c3_transition import physical_rho_operators
    from v2.core.math_tools import quintic_time_jet,chain_rule,differential_kinematics
    _,_,old=source();c=old.charts[1];u=np.linspace(c.s0+.00001,c.s1-.00001,83);H=1-c.s0;T=73.27969652156213
    F,Axy,Jxy,alpha=physical_rho_operators(u,c.s0,c.s1,c.theta,H);rc=c.rho.controls[:,0]
    times=np.array([brentq(lambda x:x**3*(10-15*x+6*x*x)-z,0,1)*T for z in u]);progress,ds,dds,ddds=quintic_time_jet(times,T)
    jet=c.evaluate_global(progress);vel,acc,jerk=chain_rule(jet,ds,dds,ddds);kin=differential_kinematics(jet.q,vel,acc,.06,.3935)
    assert np.allclose(np.column_stack([m@rc for m in Axy])/T**2,acc[:,:2],atol=1e-8,rtol=1e-8)
    assert np.allclose(np.column_stack([m@rc for m in Jxy])/T**3,jerk[:,:2],atol=1e-7,rtol=1e-7)
    wheel=np.column_stack([(F@rc+sign*.3935/2*alpha)/(.06*T**2) for sign in (-1,1)])
    assert np.allclose(wheel,kin['wheel_acceleration'],atol=1e-8,rtol=1e-8)
