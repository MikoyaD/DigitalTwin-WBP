import numpy as np
from v2.geometry.tm_rho_feedback_context import callbacks

def test_projection_checks_every_move_and_global_jet(monkeypatch):
    import v2.geometry.tm_rho_feedback_context as ctx
    called=[]
    def projected(A,ref,delta,J,d):
        called.append(delta.copy())
        return dict(status='PROJECTED',controls=np.zeros(14))
    monkeypatch.setattr(ctx,'exact_rho_projection',projected)
    profile=dict(beta=np.zeros(18),initial=np.zeros(18),basis=np.zeros((1,18)),reference=np.zeros(1))
    moves=[dict(profile=0,segment=i,x=np.array([0.,1.]),h=.1,p0=np.zeros(2),p1=np.array([i,0.]),ref=np.zeros((2,2)),J=np.zeros((0,14)),d=np.zeros(0)) for i in range(3)]
    problem=dict(profiles=[profile],moves=moves,theta_eq=np.zeros((0,18)),theta_rhs=np.zeros(0))
    _,_,project=callbacks(problem)
    result=project(dict(beta=[np.zeros(18)]))
    assert len(result)==len(called)==3
    assert all(x['status']=='PROJECTED' for x in result)
    assert [x['segment'] for x in result]==[0,1,2]
