"""V2-PRP: run a previously frozen signed R through P0 and P1.

An existing matching ROS scene is reused; otherwise the V2 runtime is started.
No planning, H fitting, robot execution, or implicit historical scene snapshot.
"""
import argparse,json,os,time,traceback,shutil
from hashlib import sha256
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from ..core import array_store as store
from . import g0_signed_ac,g1_general,runtime_v2
from .stage_v2 import HEADER,load_parent
from .tm_feedback_hg import ChartView
from ..geometry.tm_rho_feedback_frozen import thaw
from ..adapters.planning_scene_checker import PlanningSceneChecker,wire

def finalize_direct(p0,p1=None):
 """Keep analytic C parameters, discard only this invocation's dense bundles."""
 g=Path(p0)/'G';source=g/'trajectory.arrays';thin=g/'time_law.arrays'
 if source.is_dir():
  with store.load(source) as z:
   keys=('g1_density_knots','density_x','active_intervals','density_beta','normalized_nodes','g0_density_knots','g0_density_values','g1_density_values')
   store.save(thin,**{k:z[k] for k in keys if k in z.files})
  for name in ('result.json','numerical_result.json'):
   f=g/name
   if f.is_file():
    data=json.loads(f.read_text());data['time_law_arrays']=str(thin.resolve());f.write_text(json.dumps(data,indent=2))
  shutil.rmtree(source)
 for folder in (Path(p0)/'cases',Path(p1) if p1 else None):
  if folder and folder.exists():
   for bundle in list(folder.rglob('*.arrays')):
    if bundle.is_dir():shutil.rmtree(bundle)


def ensure_master(uri, config):
 from .experiment import check_master
 master=check_master(uri)
 if master=='NOT_RUNNING':
  if uri.rstrip('/')!=runtime_v2.URI.rstrip('/'):raise RuntimeError('ROS_MASTER_NOT_RUNNING: '+uri)
  runtime_v2.start(config);master=check_master(uri)
 if master!='REACHABLE':raise RuntimeError('ROS_MASTER_UNAVAILABLE: '+str(master))


def display_csv(dest, source, p0, output_path=None):
 """Select unchanged source rows, retaining the same events as signed export."""
 from scipy.optimize import brentq
 from ..timing.nested_time import NestedBase
 from ..timing.unified_g import _inverse_quintic
 from ..timing.g0_ac_geometry import chart_events
 from .g1_p01_finalize import law
 from .g1_display_sampling import select_storage_rows
 geo=json.loads((source/'geometry.json').read_text());block=thaw(geo);base_file=p0/'G/trajectory.arrays'
 if not base_file.exists():
  candidates=(p0/'G/time_law.arrays',p0/'G/time_law.npz',p0/'G/trajectory.npz')
  base_file=next((p for p in candidates if p.exists()),candidates[0])
 base=NestedBase(base_file)
 spec=json.loads((dest.parent/(dest.name+'_law.json')).read_text());k=np.asarray(spec['nodes']);x=np.asarray(spec['lambda_values'])
 def inverse(t):
  return 0. if t<=0 else base.end if t>=base.T else brentq(lambda r:base.density.integrate(np.array([r]))[0]-t,0.,base.end,xtol=2e-12)
 rknots=np.array([inverse(t) for t in k]);cuts=[]
 for c in block.charts:
  u=np.unique(np.clip(np.r_[geo['construction']['progress'],chart_events(c),c.s0,c.s1],c.s0,c.s1))
  r=np.array([_inverse_quintic(float(t))*base.end for t in u])
  cuts.extend(np.unique(np.r_[r,base.density.k[(base.density.k>r[0])&(base.density.k<r[-1])],rknots[(rknots>r[0])&(rknots<r[-1])]]))
 events=law(base.density.integrate(np.unique(cuts)),k,x)[0]
 data_file=dest/'trajectory.arrays' if (dest/'trajectory.arrays').is_dir() else dest/'trajectory.npz'
 with store.load(data_file,allow_pickle=False) as z:
  mode=np.where(z['direction']==0,0,1)
  a=np.column_stack([z[t] for t in ('t','q','velocity','acceleration','jerk','u','chart','direction')]);a=np.column_stack((a[:,:-1],mode,a[:,-1]))
  ix=select_storage_rows(a,events);csv=Path(output_path) if output_path else dest/'trajectory.csv'
  if csv.exists():raise FileExistsError('CSV_OUTPUT_EXISTS: '+str(csv))
  if output_path is None:(dest/'events.json').write_text(json.dumps({'time_s':events.tolist()}))
  np.savetxt(csv,a[ix],delimiter=',',fmt='%.17g',header=','.join(HEADER+['u','chart_index','mode_index','direction']),comments='')
  gaps=np.diff(a[:,0]);return {'path':str(csv),'dense_rows':len(a),'display_rows':len(ix),'bytes':csv.stat().st_size,'events':len(events),'max_gap_s':float(np.max(np.diff(a[ix,0]))),'source_gaps_over_0_02_s':int(np.count_nonzero(gaps>.02)),'source_max_gap_s':float(max(gaps)),'method':'original_dense_rows_50hz_event_extrema_position_refined','scope':'display only; dense source used for acceptance'}


def main(argv=None):
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('--signed-r',required=True,help='directory with geometry.json and parent_path.json')
 p.add_argument('--out',required=True,help='new result directory')
 p.add_argument('--scene-snapshot',help='optional explicitly supplied snapshot; otherwise capture current scene')
 p.add_argument('--scene-map',help='selected static map; must match the saved parent')
 p.add_argument('--compact-output',action='store_true',help='omit the redundant dense P0 CSV')
 p.add_argument('--until',choices=('g0','g1'),default='g1')
 a=p.parse_args(argv); source=Path(a.signed_r).resolve(strict=True); out=Path(a.out).resolve();out.mkdir(parents=True,exist_ok=False)
 result={'status':'FAIL','scope':'fixed saved Signed R; offline numerical and full-robot discrete collision only','signed_r':str(source),'collision':'NOT_ATTEMPTED'}; started=time.perf_counter()
 def save():
  result['wall_s']=time.perf_counter()-started;(out/'result.json').write_text(json.dumps(result,indent=2))
 try:
  geo=json.loads((source/'geometry.json').read_text());parent,path=load_parent(source)
  if geo.get('algorithm_variant')!='V2-SIGNED-H-v1':raise ValueError('SIGNED_R_REQUIRED')
  parent_map=(runtime_v2.PACKAGE/parent['config']['static_octomap']).resolve(strict=True)
  if a.scene_map and Path(a.scene_map).resolve(strict=True)!=parent_map:raise ValueError('SAVED_PARENT_SCENE_MAP_MISMATCH')
  if a.until=='g1':
   # Only start a runtime if the configured master is actually absent, never if
   # access is denied or an existing master is partial/incompatible.
   uri=os.environ.get('ROS_MASTER_URI',runtime_v2.URI);ensure_master(uri,parent['config'])
   runtime=runtime_v2.status() if uri.rstrip('/')==runtime_v2.URI.rstrip('/') else {'status':'EXTERNAL'}
   expected_map=(runtime_v2.PACKAGE/parent['config']['static_octomap']).resolve(strict=True)
   if runtime.get('status')=='READY' and Path(runtime['map']).resolve()!=expected_map:raise RuntimeError('WAREHOUSE_RUNTIME_MAP_MISMATCH '+repr(runtime))
   import rospy
   from moveit_msgs.srv import GetPlanningScene,GetPlanningSceneRequest
   if not rospy.core.is_initialized():rospy.init_node('v2_prp_from_r',anonymous=True,disable_signals=True)
   rospy.wait_for_service('/get_planning_scene',timeout=5.);rospy.wait_for_service('/check_state_validity',timeout=5.);rospy.wait_for_service('/plan_kinematic_path',timeout=5.)
   get=rospy.ServiceProxy('/get_planning_scene',GetPlanningScene);req=GetPlanningSceneRequest();req.components.components=1023;scene=get(req).scene
   if scene.is_diff or not scene.world.octomap.octomap.id or not scene.world.octomap.octomap.data:raise RuntimeError('WAREHOUSE_OCTOMAP_MISSING')
   urdf=rospy.get_param('/robot_description');srdf=rospy.get_param('/robot_description_semantic')
   if urdf!=(runtime_v2.PACKAGE/'config/gazebo_combined_robot.urdf').read_text() or srdf!=(runtime_v2.PACKAGE/'config/combined_robot.srdf').read_text():raise RuntimeError('ROBOT_MODEL_MISMATCH')
   expected=wire(scene)
   if a.scene_snapshot:
    if Path(a.scene_snapshot).resolve(strict=True).read_bytes()!=expected:raise RuntimeError('EXPLICIT_SCENE_SNAPSHOT_MISMATCH')
   elif runtime.get('status')!='READY':
    # External servers lack V2's map provenance: use the saved R's full scene
    # identity, not an assumed map based on a nonempty OctoMap.
    if sha256(expected).hexdigest()!=parent['scene']['scene_sha256']:raise RuntimeError('EXTERNAL_SCENE_IDENTITY_MISMATCH; supply matching scene snapshot')
   (out/'scene_snapshot.bin').write_bytes(expected)
   result['scene']={'mode':'explicit' if a.scene_snapshot else 'current','runtime':runtime,'master_uri':uri,'map':str(expected_map),'octomap_id':scene.world.octomap.octomap.id,'octomap_bytes':len(scene.world.octomap.octomap.data),'snapshot':str(out/'scene_snapshot.bin'),'model_matches_package':True}
   save()
   block=thaw(geo)
   from ..geometry.tm_frozen import verify_geometry
   verify_geometry(block,path,geo['construction'])
   phases=[ChartView(c,block.schema,i) for i,c in enumerate(block.charts)]
   print('R_GEOMETRY_COLLISION_START',flush=True);gs=time.perf_counter();gr=PlanningSceneChecker().check(phases,include_geometry=True)
   same=bool(gr.get('scene_snapshot_hex')) and bytes.fromhex(gr['scene_snapshot_hex'])==expected
   model=gr.get('robot_description')==urdf and gr.get('robot_description_semantic')==srdf
   geom={'status':gr['status'] if same and model else 'UNKNOWN','reason':gr['reason'],'checked_states':gr['checked_states'],'failures':gr['failures'],'wall_s':time.perf_counter()-gs,'scene_equal':same,'model_equal':model}
   result['geometry_collision']=geom;print('R_GEOMETRY_COLLISION',json.dumps(geom),flush=True)
   (out/'geometry_collision.json').write_text(json.dumps(geom,indent=2));save()
   if geom['status']!='PASS':
    result['collision']=geom;result['status']='COLLISION_NOT_PASS';(out/'collision.json').write_text(json.dumps(geom,indent=2));raise RuntimeError('R_GEOMETRY_COLLISION_NOT_PASS '+repr(geom))
  save();print('P0_START',flush=True)
  st=time.perf_counter();p0,p0r=g0_signed_ac.run(source,out/'P0',defer_collision=True,compact_output=a.compact_output);result['p0_wall_s']=time.perf_counter()-st;result['p0']=p0r
  if p0r['status']!='NUMERICAL_PASS_COLLISION_PENDING' or p0r.get('time_law_type')!='NESTED_C_V1':raise RuntimeError('P0_NOT_PASS')
  if a.until=='g0':
   if not a.compact_output:finalize_direct(p0);result['p0']=json.loads((p0/'G/result.json').read_text())
   result['status']='P0_NUMERICAL_PASS_COLLISION_PENDING';save();print('PRP_RESULT',result['status'],flush=True);return 0
  save();print('P1_START',flush=True);st=time.perf_counter()
  p1,summary=g1_general.stage_all(SimpleNamespace(geometry=str(source),g0=str(p0/'G'),out=str(out/'P1'),allow_deferred_g0_collision=True));result['p1_wall_s']=time.perf_counter()-st;result['p1_summary']=summary
  if summary['status']!='NUMERIC_ONLY_COLLISION_PENDING' or summary['stages']['recommended_numeric']!='PASS':raise RuntimeError('P1_NUMERIC_NOT_PASS')
  dest=p1/'recommended';z=store.load(dest/'trajectory.arrays',allow_pickle=False)
  result['csv']=display_csv(dest,source,p0)
  if (p1/'fast/trajectory.arrays').is_dir() and (p1/'fast_law.json').is_file():
   result['fast_csv']=display_csv(p1/'fast',source,p0)
  from .plot_trajectory import read_csv as plot_read
  from ..adapters.trajectory_viewer import read_csv as viewer_read
  pdata=plot_read(dest/'trajectory.csv');vt,vq,_=viewer_read(dest/'trajectory.csv')
  if len(vt)!=result['csv']['display_rows'] or len(pdata['time_s'])!=len(vt):raise RuntimeError('CSV_READER_MISMATCH')
  result['status']='P1_NUMERICAL_PASS_COLLISION_PENDING'
  save()
  print('COLLISION_START',len(z['q']),flush=True);st=time.perf_counter();chunks=[];count=geom['checked_states'];i=0;q=z['q'];chunk_size=8000;resource_retry=False
  while i<len(q):
   end=min(i+chunk_size,len(q));chunk_start=time.perf_counter();r=PlanningSceneChecker().check(phases,dict(q=q[i:end]),include_geometry=False);count+=r['checked_states']
   same=r.get('scene_snapshot_hex') and bytes.fromhex(r['scene_snapshot_hex'])==expected
   model=r.get('robot_description')==urdf and r.get('robot_description_semantic')==srdf
   chunk={'start':i,'end':end,'include_geometry':False,'status':r['status'],'reason':r['reason'],'checked_states':r['checked_states'],'wall_s':time.perf_counter()-chunk_start,'failures':r['failures'],'scene_equal':bool(same),'model_equal':bool(model)};chunks.append(chunk)
   print('COLLISION_CHUNK',i,end,r['status'],r['checked_states'],flush=True)
   if r['status']=='UNKNOWN' and 'COLLISION_TIME_BUDGET' in r['reason'] and same and model and not resource_retry:
    # Retry the same candidate, not the whole planning chain, at most once.
    resource_retry=True;chunk_size=4000;print('COLLISION_RETRY_SAME_CANDIDATE',i,chunk_size,flush=True);continue
   if r['status']!='PASS' or not same or not model:
    result['collision']={'status':r['status'] if r['status']!='PASS' else 'UNKNOWN','checked_states':count,'chunks':chunks,'wall_s':time.perf_counter()-st}
    result['status']='COLLISION_NOT_PASS'
    (out/'collision.json').write_text(json.dumps(result['collision'],indent=2));raise RuntimeError('COLLISION_NOT_PASS '+repr(chunk))
   if end==len(q):break
   i=end-1
  result['collision']={'status':'PASS','checked_states':count,'chunks':chunks,'wall_s':time.perf_counter()-st,'geometry_precheck':geom,'coverage':'same frozen adaptive signed geometry prechecked, every exported state and every adjacent arithmetic midpoint, discrete only'}
  (out/'collision.json').write_text(json.dumps(result['collision'],indent=2));result['status']='FIXED_SIGNED_R_SINGLE_PASS'
  if not a.compact_output:finalize_direct(p0,p1);result['p0']=json.loads((p0/'G/result.json').read_text())
  save();print('PRP_RESULT',result['status'],flush=True);return 0
 except Exception as e:
  result['error']=repr(e);(out/'error.log').write_text(traceback.format_exc());save();print('PRP_RESULT FAIL',repr(e),flush=True);return 1
if __name__=='__main__':raise SystemExit(main())
