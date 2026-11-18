import csv
import numpy as np
import pytest
from v2.scripts.plan_v2_multi import combine,read_csv,NAMES

def make(path,start,end,broken=False):
 head=['time_s']+[p+n for p in ('q_','v_','a_','jerk_') for n in NAMES]+['u','g0_time_s','chart_index','mode_index']
 with path.open('w',newline='') as f:
  w=csv.DictWriter(f,fieldnames=head);w.writeheader()
  for t,x in [(0,start),(1,end)]:
   row={k:'0' for k in head};row['time_s']=str(t);row['q_world_car/x']=str(x)
   if broken and t==0:row['v_joint1']='0.1'
   w.writerow(row)

def test_combine_retains_motion_rows_and_dwell(tmp_path):
 a,b,o=[tmp_path/x for x in ('a.csv','b.csv','out.csv')];make(a,0,1);make(b,1,2);r=combine(a,b,o);h,rows=read_csv(o);_,a0=read_csv(a);_,b0=read_csv(b)
 assert r['dwell_s']==1 and r['duration_s']==3 and len(rows)==5
 assert [float(x['time_s']) for x in rows]==[0,1,1.5,2,3]
 for old,new in zip(a0+b0,rows[:2]+rows[3:]):
  assert all(old[k]==new[k] for k in h if k!='time_s')
 assert all(float(rows[2]['q_'+n])==float(rows[1]['q_'+n]) for n in NAMES)

def test_rejects_moving_seam(tmp_path):
 a,b,o=[tmp_path/x for x in ('a.csv','b.csv','out.csv')];make(a,0,1);make(b,1,2,True)
 with pytest.raises(ValueError,match='STOP_SEAM_NOT_PASS'):combine(a,b,o)
 assert not o.exists()

def test_five_segments_have_five_holds_and_preserve_rows(tmp_path):
 from v2.scripts.plan_v2_multi import combine_all
 paths=[]
 for i in range(5):
  p=tmp_path/('s%d.csv'%i);make(p,i,i+1);paths.append(p)
 dest=tmp_path/'combined.csv';result=combine_all(paths,dest);head,rows=read_csv(dest)
 assert abs(result['duration_s']-10)<1e-7 and len(result['seams'])==4 and len(rows)==15
 assert all(x['dwell_end_s']-x['dwell_start_s']==1 for x in result['intervals'])
 for i in range(5):
  assert abs(float(rows[3*i+2]['time_s'])-float(rows[3*i+1]['time_s'])-1)<1e-12
  assert all(rows[3*i+2]['q_'+n]==rows[3*i+1]['q_'+n] for n in NAMES)
  assert all(float(rows[3*i+2][prefix+n])==0 for prefix in ('v_','a_') for n in NAMES)
 for i,p in enumerate(paths):
  _,original=read_csv(p)
  for j,row in enumerate(original):assert all(rows[3*i+j][k]==row[k] for k in head if k!='time_s')
 assert abs(float(rows[-1]['time_s'])-10)<1e-7

def test_five_segments_refuse_bad_parking_boundary(tmp_path):
 from v2.scripts.plan_v2_multi import combine_all
 paths=[]
 for i in range(5):
  p=tmp_path/('bad%d.csv'%i);make(p,i,i+1,broken=(i==2));paths.append(p)
 with pytest.raises(ValueError,match='STOP_BOUNDARY_NOT_PASS'):combine_all(paths,tmp_path/'badout.csv')
 assert not (tmp_path/'badout.csv').exists()

def test_hold_accepts_float_seam_drift(tmp_path):
 from v2.scripts.plan_v2_multi import combine_all
 paths=[]
 for i in range(5):
  p=tmp_path/('f%d.csv'%i);make(p,i,i+1);paths.append(p)
 h,rows=read_csv(paths[1]);rows[0]['q_world_car/x']='1.0000000000000002'
 with paths[1].open('w',newline='') as f:
  w=csv.DictWriter(f,fieldnames=h);w.writeheader();w.writerows(rows)
 result=combine_all(paths,tmp_path/'tolerated.csv');assert result['seams'][0]['q_max']>0 and result['seams'][0]['q_max']<1e-4

def test_hold_rejects_position_outside_tolerance(tmp_path):
 from v2.scripts.plan_v2_multi import combine_all
 paths=[]
 for i in range(5):
  p=tmp_path/('o%d.csv'%i);make(p,i,i+1);paths.append(p)
 h,rows=read_csv(paths[1]);rows[0]['q_world_car/x']='1.000101'
 with paths[1].open('w',newline='') as f:
  w=csv.DictWriter(f,fieldnames=h);w.writeheader();w.writerows(rows)
 with pytest.raises(ValueError,match='POSITION_SEAM_NOT_PASS'):combine_all(paths,tmp_path/'outofrange.csv')
