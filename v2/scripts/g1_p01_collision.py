"""Fresh full-robot scene query for saved G1 P01 trajectory; no robot execution."""
import argparse,json,time,traceback
from pathlib import Path
import numpy as np
from ..geometry.tm_rho_feedback_frozen import thaw
from ..scripts.tm_feedback_hg import ChartView
from ..adapters.planning_scene_checker import PlanningSceneChecker
from .g1_dense_recovery import read_numeric_rows

ROOT=Path(__file__).resolve().parents[2]
H=ROOT/'v2/outputs/V2-20261030-000922-single_goal_05_TM_RHO_FEEDBACK-RRTstar'
SCENE=ROOT/'v2/outputs/runs/run_20261023T165050Z_548ef707/preflight_recovered/scene_snapshot.bin'

def main():
 p=argparse.ArgumentParser();p.add_argument('--input',required=True);p.add_argument('--geometry',default=str(H));p.add_argument('--scene-snapshot',default=str(SCENE));p.add_argument('--geometry-once',dest='geometry_once',action='store_true',default=True,help='check analytic geometry in first chunk only (default)');p.add_argument('--repeat-geometry',dest='geometry_once',action='store_false',help='legacy A/B method: check analytic geometry in every chunk');a=p.parse_args();dest=Path(a.input);out=dest/'collision.json'
 if out.exists():raise RuntimeError('COLLISION_RESULT_ALREADY_EXISTS')
 start=time.perf_counter();result=dict(status='UNKNOWN',chunks=[],expected_snapshot=str(Path(a.scene_snapshot)),checked_states=0)
 try:
  import rospy
  rospy.init_node('g1_p01_collision',anonymous=True,disable_signals=True)
  block=thaw(json.loads((Path(a.geometry)/'geometry.json').read_text()));phases=[ChartView(c,block.schema,i) for i,c in enumerate(block.charts)];q=read_numeric_rows(dest,a.geometry)[:,1:10];expected=Path(a.scene_snapshot).read_bytes()
  i=0
  while i<len(q):
   end=min(i+20000,len(q));r=PlanningSceneChecker().check(phases,dict(q=q[i:end]),include_geometry=(not a.geometry_once or i==0));same=bool(r.get('scene_snapshot_hex') and bytes.fromhex(r['scene_snapshot_hex'])==expected)
   model=(r.get('robot_description')==(ROOT/'config/gazebo_combined_robot.urdf').read_text() and r.get('robot_description_semantic')==(ROOT/'config/combined_robot.srdf').read_text())
   chunk=dict(include_geometry=(not a.geometry_once or i==0),start=i,end=end,status=r['status'],reason=r['reason'],checked_states=r['checked_states'],failures=r['failures'],scene_raw_equal=same,model_text_equal=model,resolution=r['resolution']);result['chunks'].append(chunk);result['checked_states']+=r['checked_states'];print('COLLISION_CHUNK',i,end,r['status'],'scene_raw',same,'model_raw',model,flush=True)
   if r['status']!='PASS' or not same or not model:
    result.update(status=r['status'] if r['status']!='PASS' else 'UNKNOWN',reason=r['reason'] if r['status']!='PASS' else 'SCENE_OR_MODEL_NOT_IDENTICAL');break
   if end==len(q):result.update(status='PASS',reason='all new trajectory samples, midpoints and analytic geometry checked in byte-identical scene');break
   i=end-1
 except Exception as e:result.update(status='UNKNOWN',reason=repr(e));(dest/'collision_error.log').write_text(traceback.format_exc())
 result['collision_wall_s']=time.perf_counter()-start;out.write_text(json.dumps(result,indent=2));print('COLLISION_RESULT',result['status'],'states',result['checked_states'],'wall_s',result['collision_wall_s'],flush=True)
 return 0 if result['status']=='PASS' else 1
if __name__=='__main__':raise SystemExit(main())
