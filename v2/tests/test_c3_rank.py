import copy
from v2.scripts.c3_rank import promotion,pareto

def sample():
 m={'duration_s':1.,'xy_jerk_peak':1.,'xy_jerk_rms':1.,'angular_arm_peak':[1.]*7,'angular_arm_rms':[1.]*7,'left_right_wheel_acceleration_peak':[1.,1.],'length_xy_m':1.,'angle_abs_rad':1.}
 return dict(id='a',metrics=m,h_time_s=1.,g_time_s=1.,reference_peak=[1.]*9,c3_error=0.,collision='PASS',numerical='PASS',c3=True,restore=True,sampling_comparable=True)

def test_promotion_requires_collision_and_each_unit_group():
 a=sample();baseline=copy.deepcopy(a['metrics']);assert promotion(a,baseline)
 a['metrics']['angular_arm_rms'][4]=1.001;assert not promotion(a,baseline)
 a=sample();a['collision']='UNKNOWN';assert not promotion(a,baseline)

def test_pareto_excludes_unknown_and_keeps_tradeoffs():
 a=sample();b=copy.deepcopy(a);b['id']='b';b['metrics']['duration_s']=.9;b['metrics']['xy_jerk_peak']=1.1
 c=copy.deepcopy(a);c['id']='c';c['metrics']['duration_s']=.5;c['collision']='UNKNOWN'
 assert set(pareto([a,b,c]))=={'a','b'}
