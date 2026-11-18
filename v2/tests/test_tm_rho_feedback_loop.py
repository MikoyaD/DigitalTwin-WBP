from v2.geometry.tm_rho_feedback_loop import iterate_center

def test_budget_and_all_projections_on_every_valid_step():
    calls=[]
    def propose(center,trust):
        calls.append(('proposal',trust)); return {'status':'DIAGNOSTIC_PROPOSAL'}
    def evaluate(center,proposal,fraction):
        calls.append(('evaluate',fraction))
        return dict(previously_feasible=[0]),dict(predicted_decrease=1.,actual_decrease=0.),True
    def project_all(candidate):
        calls.append(('project_all',))
        return [{'status':'PROJECTED'},{'status':'NUMERICALLY_INFEASIBLE'}]
    result=iterate_center(dict(previously_feasible=[0]),propose,evaluate,project_all)
    assert result['status']=='FIXED_JET_FEEDBACK_EXHAUSTED'
    assert [x for x in calls if x[0]=='proposal']==[('proposal',1.),('proposal',.5)]
    assert len([x for x in calls if x[0]=='project_all'])==6
    assert all(len(r['steps'])==3 for r in result['rounds'])

def test_projected_segments_are_not_h_success():
    r=iterate_center({},lambda c,t:{'status':'DIAGNOSTIC_PROPOSAL'},
        lambda c,p,f:({},dict(predicted_decrease=1.,actual_decrease=1.),True),
        lambda c:[{'status':'PROJECTED'}])
    assert r['status']=='ALL_PROJECTED_NOT_H'
