"""Evaluate Isaac physical states against one P1 CSV; no simulator imports."""
import csv,json,math,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from v2.core.kdl_fk import LocalFK
N=['world_car/x','world_car/y','world_car/theta']+['joint%d'%i for i in range(1,7)]
def read(path):
 with Path(path).open() as f:r=list(csv.DictReader(f))
 return {k:np.array([float(x[k]) for x in r]) for k in r[0]}
def rm(x):return float(np.sqrt(np.mean(np.asarray(x)**2)))
def dev(x,lim,t):
 a=np.abs(x);over=np.maximum(a-lim,0);dt=np.diff(t);mask=over[:-1]>0;edges=np.diff(np.r_[False,mask,False].astype(int));starts=np.flatnonzero(edges==1);ends=np.flatnonzero(edges==-1);return {'peak_ratio':float(np.max(a)/lim),'max_excess':float(np.max(over)),'time_fraction':float(np.sum(dt*mask)/sum(dt)),'longest_s':float(max((np.sum(dt[i:j]) for i,j in zip(starts,ends)),default=0)),'excess_integral':float(np.sum(over[:-1]*dt))}
def graph(path,t,arrays,labels,title,ylabel):
 fig,ax=plt.subplots(figsize=(11,5));
 for x,label in zip(arrays,labels):ax.plot(t,x,label=label,linewidth=.8)
 ax.set_title(title);ax.set_xlabel('Time (s)');ax.set_ylabel(ylabel);ax.grid(alpha=.25);ax.legend(ncol=3,fontsize=7);fig.tight_layout();fig.savefig(path,dpi=200);plt.close(fig)
def evaluate(task,out,make_plots=False,save_derived=False,replace_metrics=False):
 task=Path(task);out=Path(out);metadata=json.loads((out/'execution_metadata.json').read_text());trajectory=task/metadata['reference_trajectory'];s=read(out/'states.csv');r=read(trajectory);sim=s['sim_time_s'];t=s['trajectory_time_s'];rt=r['time_s'];n=len(t)
 if n<3 or np.any(np.diff(sim)<=0) or np.any(np.diff(t)<0):raise ValueError('INVALID_EXECUTION_TIMESTAMPS')
 qref=np.column_stack([np.interp(t,rt,np.unwrap(r['q_'+name]) if name=='world_car/theta' else r['q_'+name]) for name in N]);vref=np.column_stack([np.interp(t,rt,r['v_'+name]) for name in N]);aref=np.column_stack([np.interp(t,rt,r['a_'+name]) for name in N]);jref=np.column_stack([np.interp(t,rt,r['jerk_'+name]) for name in N]);qact=np.column_stack([s['x_actual'],s['y_actual'],np.unwrap(s['yaw_actual'])]+[s['joint%d_actual'%j] for j in range(1,7)]);vact=np.column_stack([s['vx_world_actual'],s['vy_world_actual'],s['yaw_velocity_actual']]+[s['joint%d_vactual'%j] for j in range(1,7)]);acc=np.gradient(vact,sim,axis=0,edge_order=1);jerk=np.gradient(acc,sim,axis=0,edge_order=1);qe=qact-qref;qe[:,2]=np.arctan2(np.sin(qe[:,2]),np.cos(qe[:,2]));ve=vact-vref;ae=acc-aref;lateral=-np.sin(qact[:,2])*vact[:,0]+np.cos(qact[:,2])*vact[:,1];wheel=np.column_stack([s['wheel_left_actual']-s['wheel_left_cmd'],s['wheel_right_actual']-s['wheel_right_cmd']]);
 fk=LocalFK((Path(__file__).resolve().parents[2]/'config/gazebo_combined_robot.urdf').read_text());positions=[];quats=[]
 for q in qref:
  pos,rot=fk.pose(q);pos[2]+=.034224;positions.append(pos);quats.append(rot.as_quat())
 pr=np.asarray(positions);rr=Rotation.from_quat(np.asarray(quats));pa=np.column_stack([s['link6_x'],s['link6_y'],s['link6_z']]);qa=np.column_stack([s['link6_qx'],s['link6_qy'],s['link6_qz'],s['link6_qw']]);ra=Rotation.from_quat(qa);ep=pa-pr;eo=(rr.inv()*ra).as_rotvec();poserr=np.linalg.norm(ep,axis=1);angerr=np.linalg.norm(eo,axis=1);xy=np.linalg.norm(qe[:,:2],axis=1)
 effective_path=task/'effective_config.json'
 if effective_path.is_file():effective=json.loads(effective_path.read_text())
 else:
  import yaml
  effective=yaml.safe_load((task/'effective_config.yaml').read_text())
 lim=effective['limits'];vlim=lim['velocity'];alim=lim['acceleration'];metrics={'tracking':{'base':{'xy_rmse_m':rm(xy),'xy_max_m':float(max(xy)),'xy_final_m':float(xy[-1]),'yaw_rmse_rad':rm(qe[:,2]),'yaw_max_rad':float(max(abs(qe[:,2]))),'yaw_final_rad':float(qe[-1,2])},'arm':{f'joint{j}':{'rmse_rad':rm(qe[:,j+2]),'max_rad':float(max(abs(qe[:,j+2]))),'final_rad':float(qe[-1,j+2])} for j in range(1,7)},'link6':{'position_rmse_m':rm(poserr),'position_max_m':float(max(poserr)),'position_final_m':float(poserr[-1]),'orientation_rmse_rad':rm(angerr),'orientation_max_rad':float(max(angerr)),'orientation_final_rad':float(angerr[-1])}},'motion':{},'constraint_deviation':{},'nonholonomic':{'lateral_max_m_s':float(max(abs(lateral))),'lateral_rms_m_s':rm(lateral),'lateral_abs_integral_m':float(np.trapz(abs(lateral),sim)),'wheels':{side:{'rmse_rad_s':rm(wheel[:,i]),'max_rad_s':float(max(abs(wheel[:,i])))} for i,side in enumerate(['left','right'])}}}
 metrics['tracking']['arm_overall_max_rad']=float(np.max(np.abs(qe[:,3:])))
 # Compare actual Link6 with the reference at recorded segment end events;
 # report sampling offset rather than inventing an exact physical state.
 task_result=task/'result.json'
 if task_result.is_file():
  result=json.loads(task_result.read_text());events=result.get('task',{}).get('events',[])
  if events:
   measurements=[]
   for event in events:
    if 'motion_end_s' not in event:continue
    event_time=float(event['motion_end_s']);index=int(np.argmin(np.abs(t-event_time)))
    measurements.append({'goal':event.get('segment'),'reference_event_time_s':event_time,'sample_trajectory_time_s':float(t[index]),'sample_time_offset_s':float(t[index]-event_time),'link6_position_tracking_error_m':float(poserr[index]),'link6_orientation_tracking_error_rad':float(angerr[index])})
   metrics['tracking']['goal_events']=measurements

 for i,name in enumerate(N):
  metrics['motion'][name]={'velocity_ref_peak':float(max(abs(vref[:,i]))),'velocity_actual_peak':float(max(abs(vact[:,i]))),'velocity_error_rmse':rm(ve[:,i]),'acceleration_ref_peak':float(max(abs(aref[:,i]))),'acceleration_actual_peak':float(max(abs(acc[:,i]))),'acceleration_ref_rms':rm(aref[:,i]),'acceleration_actual_rms':rm(acc[:,i]),'acceleration_error_rmse':rm(ae[:,i]),'jerk_ref_peak':float(max(abs(jref[:,i]))),'jerk_actual_peak':float(max(abs(jerk[:,i]))),'jerk_ref_rms':rm(jref[:,i]),'jerk_actual_rms':rm(jerk[:,i])};metrics['constraint_deviation'][name]={'velocity':dev(vact[:,i],vlim[i],sim),'acceleration':dev(acc[:,i],alim[i],sim)}
 cols={};
 for i,name in enumerate(N):
  tag=name.replace('/','_');cols['position_error_'+tag]=qe[:,i];cols['velocity_error_'+tag]=ve[:,i];cols['actual_acceleration_'+tag]=acc[:,i];cols['acceleration_error_'+tag]=ae[:,i];cols['actual_jerk_'+tag]=jerk[:,i]
 cols.update(lateral_velocity=lateral,wheel_left_error=wheel[:,0],wheel_right_error=wheel[:,1],link6_position_error_x=ep[:,0],link6_position_error_y=ep[:,1],link6_position_error_z=ep[:,2],link6_position_error_norm=poserr,link6_orientation_error_rad=angerr)
 if save_derived:
  target=out/'derived.csv'
  if target.exists():raise FileExistsError('DERIVED_CSV_EXISTS: '+str(target))
  with target.open('x',newline='') as f:
   w=csv.writer(f);w.writerow(['sim_time_s','trajectory_time_s']+list(cols));
   for i in range(n):w.writerow([format(sim[i],'.12g'),format(t[i],'.12g')]+[format(v[i],'.9g') for v in cols.values()])
 metrics_path=out/'metrics.json'
 if metrics_path.exists() and not replace_metrics:
  if json.loads(metrics_path.read_text())!=metrics:raise ValueError('EXISTING_METRICS_MISMATCH; original preserved')
 else:metrics_path.write_text(json.dumps(metrics,indent=2)+'\n')
 if not make_plots:return metrics
 fig=out/'figures';fig.mkdir(exist_ok=True)
 fig1,ax=plt.subplots(figsize=(8,6));ax.plot(qref[:,0],qref[:,1],label='Reference');ax.plot(qact[:,0],qact[:,1],label='Actual');ax.set_aspect('equal');ax.set_xlabel('World x (m)');ax.set_ylabel('World y (m)');ax.legend();ax.grid();fig1.tight_layout();fig1.savefig(fig/'base_xy.png',dpi=200);plt.close(fig1)
 graph(fig/'base_error.png',t,[xy,qe[:,2]],['XY (m)','yaw (rad)'],'Base tracking errors','Error (m / rad)')
 graph(fig/'arm_position.png',t,[x for j in range(6) for x in (qref[:,j+3],qact[:,j+3])],[z for j in range(6) for z in (f'J{j+1} ref',f'J{j+1} actual')],'Joint positions','Position (rad)')
 graph(fig/'arm_error.png',t,[qe[:,j+3] for j in range(6)],[f'J{j+1}' for j in range(6)],'Joint position errors','Error (rad)')
 for title,actual,reference in [('velocity',vact,vref),('acceleration',acc,aref),('jerk',jerk,jref)]:
  fig2,axs=plt.subplots(3,3,figsize=(13,10),sharex=True)
  for j,ax in enumerate(axs.ravel()):ax.plot(t,reference[:,j],label='ref',linewidth=.7);ax.plot(t,actual[:,j],label='actual',linewidth=.7);ax.set_title(N[j],fontsize=8);ax.grid(alpha=.2);ax.tick_params(labelsize=6)
  axs[0,0].legend(fontsize=6);fig2.suptitle(title+'; SI units, translational m and rotational rad');fig2.tight_layout();fig2.savefig(fig/(title+'.png'),dpi=180);plt.close(fig2)
 graph(fig/'lateral_velocity.png',t,[lateral],['lateral'],'Base nonholonomic deviation','Velocity (m/s)')
 fig3=plt.figure(figsize=(11,5));ax=fig3.add_subplot(121,projection='3d');ax.plot(pr[:,0],pr[:,1],pr[:,2],label='ref');ax.plot(pa[:,0],pa[:,1],pa[:,2],label='actual');ax.legend();ax.set_title('Link6 world path');ax=fig3.add_subplot(122);ax.plot(t,poserr,label='position m');ax.plot(t,angerr,label='orientation rad');ax.legend();ax.grid();fig3.tight_layout();fig3.savefig(fig/'link6.png',dpi=200);plt.close(fig3)
 return metrics
if __name__=='__main__':
 import argparse
 parser=argparse.ArgumentParser();parser.add_argument('task');parser.add_argument('output');parser.add_argument('--figures',action='store_true');parser.add_argument('--save-derived',action='store_true');args=parser.parse_args();evaluate(args.task,args.output,make_plots=args.figures,save_derived=args.save_derived)
