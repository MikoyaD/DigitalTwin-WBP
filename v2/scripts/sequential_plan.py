#!/usr/bin/env python3
"""Run a single-goal sequential baseline using an existing PRP parent and scene."""
import argparse,hashlib,json,math,subprocess,sys,traceback
from pathlib import Path
from moveit_msgs.msg import RobotTrajectory
P=Path(__file__).resolve().parents[2]
def main():
 p=argparse.ArgumentParser();p.add_argument('--parent',required=True,help='existing PRP ParentPath/parent_path.json');p.add_argument('--scene-snapshot',required=True);p.add_argument('--out',required=True,help='new output directory');p.add_argument('--base-plan',help='reuse an explicitly saved native base plan with matching endpoints');a=p.parse_args();out=Path(a.out).resolve();out.mkdir(parents=True,exist_ok=False);result={'status':'FAIL','parent':str(Path(a.parent).resolve()),'scene_snapshot':str(Path(a.scene_snapshot).resolve())}
 def run(name,args):
  z=subprocess.run([sys.executable,'-B','-m','v2.scripts.'+name,*map(str,args)],cwd=str(P),stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True);(out/(name+'.log')).write_text(z.stdout);print('STEP',name,'exit',z.returncode,flush=True);return z.returncode
 try:
  original=json.loads(Path(a.parent).read_text())
  if a.base_plan:
   raw_dir=Path(a.base_plan).resolve(strict=True);code=None;result['base_plan_reused']=str(raw_dir)
  else:
   raw_dir=out/'base_plan';code=run('sequential_baseline',['--parent',a.parent,'--out',raw_dir])
  raw_file=raw_dir/'base_trajectory.bin';status=json.loads((raw_dir/'result.json').read_text());result['base_native_status']=status
  import numpy as np
  if not np.allclose(status['start'],original['path']['q'][0],atol=1e-9,rtol=0) or not np.allclose(status['goal'],original['path']['q'][-1],atol=1e-9,rtol=0):raise ValueError('REUSED_BASE_ENDPOINT_MISMATCH')
  if not raw_file.exists() or status.get('error')!="ValueError('BASE_VELOCITY_OR_ACCELERATION_MISSING')":raise RuntimeError('BASE_PLAN_FAILED_OR_UNEXPECTED_STATUS_'+str(code))
  # The base path is accepted only as geometric input; its missing derivative
  # fields and nonholonomic error are repaired and checked by the formal R/P1 chain.
  raw=raw_file.read_bytes();traj=RobotTrajectory();traj.deserialize(raw);q0=original['path']['q'][0];qq=[];tt=[]
  for point in traj.multi_dof_joint_trajectory.points:
   x=point.transforms[0];qq.append([x.translation.x,x.translation.y,2*math.atan2(x.rotation.z,x.rotation.w)]+q0[3:]);tt.append(point.time_from_start.to_sec())
  parent=original['path'];parent.update(trial_id='sequential_base',q=qq,actual_time_s=tt,requested_start=qq[0],requested_goal=qq[-1],parent_path_hash=hashlib.sha256(raw).hexdigest());parent['metadata']['baseline']='MoveIt base group; six arm joints fixed';original['planning']['source']='base group RobotTrajectory';base_parent=out/'base_parent';base_parent.mkdir();(base_parent/'parent_path.json').write_text(json.dumps(original,indent=2))
  rec=out/'base_reconstructed';code=run('prp_from_p',['--parent',base_parent,'--out',rec,'--until','g1','--rho-controls','36','--signed-recovery','--compact-output','--scene-snapshot',a.scene_snapshot]);result['base_reconstruction_code']=code
  if code:raise RuntimeError('BASE_RECONSTRUCTION_FAILED')
  finish=out/'final';code=run('sequential_finish',['--parent',a.parent,'--base',rec/'PRP/P1/recommended/trajectory.csv','--out',finish,'--smooth-arm']);result['final_code']=code
  if code:raise RuntimeError('ARM_OR_FINAL_VALIDATION_FAILED')
  result.update(status='PASS',trajectory=str(finish/'trajectory.csv'),details=str(finish/'result.json'))
 except Exception as exc:result.update(error=repr(exc),traceback=traceback.format_exc());print('FAIL',repr(exc),flush=True)
 (out/'result.json').write_text(json.dumps(result,indent=2)+'\n');print('SEQUENTIAL_RESULT',json.dumps(result),flush=True);return 0 if result['status']=='PASS' else 1
if __name__=='__main__':raise SystemExit(main())
