"""Independent TM fixed-parent H/G; no planning or robot execution."""
import argparse, json, shutil, tempfile, time, traceback
from dataclasses import asdict
from pathlib import Path
import numpy as np
import yaml
from ..core.contracts import Status,require
from ..core.math_tools import quintic_time_jet,chain_rule,differential_kinematics
from ..geometry.tm_rho_feedback_build import build as build_with_tail_fallback
from ..geometry.tm_rho_feedback_frozen import freeze,thaw
from ..geometry.tm_frozen import verify_geometry
from ..timing.tm_g import parameterize,sample,seams
from ..timing.unified_g import _inverse_quintic
from ..adapters.planning_scene_checker import PlanningSceneChecker
from .stage_v2 import (PACKAGE,load_parent,limits_for,assert_sources,new_dir,write,read,
                       context,clean_collision,digest,HEADER)

class ChartView:
    def __init__(self,chart,schema,index): self.chart=chart;self.schema=schema;self.phase_id='chart_%d'%index
    def evaluate(self,x):
        x=np.asarray(x);return self.chart.evaluate_global(self.chart.s0+x*(self.chart.s1-self.chart.s0))

def metrics(block,output,T,limits):
    integral=np.zeros(9);peak=np.zeros(9);rms=np.zeros(9);spin_peak=0.
    for chart in block.charts:
        ta=_inverse_quintic(chart.s0)*T;tb=_inverse_quintic(chart.s1)*T
        t=np.linspace(ta,tb,max(2,int(np.ceil((tb-ta)/.01))+1))
        u,v,a,j=quintic_time_jet(t,T);jet=chart.evaluate_global(np.clip(u,chart.s0,chart.s1));jerk=chain_rule(jet,v,a,j)[2]
        integral+=np.trapz(abs(jerk),t,axis=0);rms+=np.trapz(jerk**2,t,axis=0);peak=np.maximum(peak,np.max(abs(jerk),axis=0))
        if getattr(chart,'stationary',False):spin_peak=max(spin_peak,float(np.max(abs(jerk[:,2]))))
    kin=differential_kinematics(output['q'],output['velocity'],output['acceleration'],float(limits.wheel_radius.value),float(limits.wheel_separation.value))
    return dict(spin_jerk_peak_rad_s3=spin_peak, xy_speed_peak=float(np.max(np.linalg.norm(output['velocity'][:,:2],axis=1))),
                xy_acceleration_peak=float(np.max(np.linalg.norm(output['acceleration'][:,:2],axis=1))),
                wheel_velocity_peak=np.max(abs(kin['wheel_velocity']),axis=0).tolist(),
                wheel_acceleration_peak=np.max(abs(kin['wheel_acceleration']),axis=0).tolist(),
                lateral_residual_peak=float(np.max(abs(kin['lateral']))),
                jerk_mean_absolute=(integral/T).tolist(),jerk_peak=peak.tolist(),jerk_rms=np.sqrt(rms/T).tolist(),
                units=['m/s^3']*2+['rad/s^3']*7,sampling='<=0.01s per chart; both one-sided endpoint jets')

def profile(parent,config):
    cfg=yaml.safe_load(Path(config).read_text())
    if cfg.get('algorithm')=='V2-TM-RHO-FEEDBACK-v1':
        require(cfg.get('max_feedback_rounds')==2 and cfg.get('step_lengths')==[1.0,0.5,0.25], 'TM_POLICY','unsupported budget')
        parent['tm_policy']=dict(config=str(Path(config).resolve()),settings=cfg)
        cfg=yaml.safe_load((PACKAGE/cfg['profile_config']).read_text())
    p=cfg['constraint_profile']
    require(p['identity']=='V2_NINE_DOF_VA_ONLY' and p['velocity']==[.5]*9 and p['acceleration']==[1.]*9
            and all(not p['enabled'][k] for k in p['enabled']),'TM_LIMIT_CONTRACT','nine DOF va-only required')
    parent['experiment_limit_profile']=p
    return parent

def h(source,config):
    source=Path(source).resolve(strict=True);parent,path=load_parent(source);assert_sources(parent);profile(parent,config)
    root=new_dir(parent['config'],dict(id=parent['task_id']+'_TM_RHO_FEEDBACK'))
    try:
        shutil.copyfile(source/'parent_path.csv',root/'parent_path.csv');write(root/'parent_path.json',parent)
        lim=limits_for(parent);b,decision=build_with_tail_fallback(path,lim)
        verify_geometry(b,path,decision)
        geometry=freeze(b,path,decision,parent,lim);rebuilt=thaw(geometry)
        points=np.unique(np.r_[np.linspace(0,1,1001),[c.s0 for c in b.charts],[c.s1 for c in b.charts]])
        for field in ('q','qs','qss','qsss'):
            require(np.array_equal(getattr(b.evaluate(points),field),getattr(rebuilt.evaluate(points),field)),
                    'TM_ROUNDTRIP',field)
        write(root/'geometry.json',geometry)
        print('TM_H_PASS',root,decision['candidate'],flush=True);return root
    except Exception as exc:
        (root/'error.log').write_text('stage=H\ninput=%s\nerror=%r\n%s'%(source,exc,traceback.format_exc()))
        print('TM_H_FAIL',root,repr(exc),flush=True);return None

def inputs(root):
    root=Path(root).resolve(strict=True);parent,path=load_parent(root);assert_sources(parent)
    geo=read(root/'geometry.json');lim=limits_for(parent)
    for name,expected in geo.get('source_identity',{}).items():
        require(digest(PACKAGE/'v2'/name)==expected,'TM_SOURCE_CHANGED',name)
    require(geo['parent_path_hash']==path.parent_path_hash,'TM_PARENT','mismatch')
    require(geo['effective_limit_identity']==lim.metadata['constraint_profile']['effective_limit_identity'],
            'TM_LIMITS','mismatch');block=thaw(geo);verify_geometry(block,path,geo['construction'])
    return root,parent,path,geo,lim,block

def collision(parent,block,out):
    scene=context(parent['config'],parent['scene'])
    import rospy
    if not rospy.core.is_initialized():rospy.init_node('v2_tm_scene_check',anonymous=True,disable_signals=True)
    c=clean_collision(PlanningSceneChecker().check([ChartView(x,block.schema,i) for i,x in enumerate(block.charts)],out))
    require(c['status']=='PASS' and c.get('scene_sha256')==scene['scene_sha256'],
            'TM_COLLISION_NOT_PASS',repr(dict(status=c['status'],reason=c.get('reason'),first=(c.get('failures') or [None])[0])))
    return c

def check_workspace(parent,record):
    workspace=parent['config'].get('workspace')
    require(isinstance(workspace,dict) and workspace.get('frame_id')==record.schema.frame_id,
            'TM_WORKSPACE_UNKNOWN','workspace frame or limits missing')
    lo=np.asarray(workspace['min_corner'][:2],float);hi=np.asarray(workspace['max_corner'][:2],float)
    require(np.all(record.q[:,:2]>=lo) and np.all(record.q[:,:2]<=hi),
            'TM_WORKSPACE_BOUNDS','sampled XY outside planning workspace')


def g(root):
    root,parent,path,geo,lim,block=inputs(root);folder=root/'G';require(not folder.exists(),'TM_G_EXISTS','never overwrite')
    try:
        out=parameterize(block,path,lim,geo['construction']['progress']);r=out['record']
        data=dict(time_s=r.time_s,q=r.q,velocity=r.velocity,acceleration=r.acceleration,jerk=r.jerk)
        check_workspace(parent,r)
        col=collision(parent,block,data);met=metrics(block,data,out['duration_s'],lim)
        mode_boundaries={x['u1'] for x in geo['construction']['modes'][:-1]}
        stops=sum(x['whole_body_speed']<=1e-8 for x in out['seams'] if any(abs(x['u']-v)<1e-12 for v in mode_boundaries))
        result=dict(algorithm='V2-TM-RHO-FEEDBACK-v1',geometry_contract_version=4,status='PASS',numerical_status='PASS',
                    collision=col,parent_path_hash=path.parent_path_hash,geometry_sha256=digest(root/'geometry.json'),
                    duration_s=out['duration_s'],h_build_s=geo['construction']['h_build_s'],g_build_s=out['backend_s'],
                    uniform_n=out['uniform_n'],seams=out['seams'],mode_boundary_whole_body_stops=stops,
                    limits=asdict(lim),validation=asdict(out['validation']),metrics=met)
        matrix=np.column_stack([data[k] for k in ('time_s','q','velocity','acceleration','jerk')])
        with tempfile.TemporaryDirectory(prefix='.tm_g_',dir=str(root)) as tmp:
            d=Path(tmp)/'G';d.mkdir();np.savetxt(d/'trajectory.csv',matrix,delimiter=',',fmt='%.17g',header=','.join(HEADER),comments='')
            write(d/'result.json',result);write(d/'diagnostics.json',met)
            (d/'report.md').write_text('# V2-TM-RHO-FEEDBACK-v1 H/G\n\n参考时长 %.9f s；数值PASS；真实离散碰撞PASS（%s状态）。\n'%(out['duration_s'],col['checked_states']))
            d.rename(folder)
        print('TM_G_PASS',folder,'time=',out['duration_s'],'collision_states=',col['checked_states'],flush=True);return True
    except Exception as exc:
        folder.mkdir();(folder/'error.log').write_text('stage=G\nerror=%r\n%s'%(exc,traceback.format_exc()))
        print('TM_G_FAIL',folder,repr(exc),flush=True);return False

def independent(root):
    root,parent,path,geo,lim,block=inputs(root);result=read(root/'G/result.json')
    require(result['geometry_sha256']==digest(root/'geometry.json'),'TM_H_CHANGED','geometry mismatch')
    record,_=sample(block,path,result['duration_s'],result['uniform_n'],geo['construction']['progress'])
    check=__import__('v2.validation.numerical',fromlist=['validate']).validate(record,lim,block.start,block.end)
    require(check.numerical_status==Status.PASS,'TM_VALIDATION',repr(check.failure_codes));seams(block,result['duration_s']);check_workspace(parent,record)
    expected=np.column_stack([getattr(record,k) for k in ('time_s','q','velocity','acceleration','jerk')]);saved=np.loadtxt(root/'G/trajectory.csv',delimiter=',',skiprows=1)
    require(saved.shape==expected.shape and np.allclose(saved,expected,atol=1e-10,rtol=0),'TM_CSV','mismatch')
    # The same frozen geometry/scene was queried by G; caller can explicitly rerun collision if desired.
    print('TM_INDEPENDENT_NUMERICAL_PASS',root,'collision_reused_from_G',flush=True)

def main():
    p=argparse.ArgumentParser();p.add_argument('--stage',choices=['h','g','validate','all'],required=True)
    p.add_argument('--input',required=True);p.add_argument('--config',default='v2/configs/tm_rho_feedback_v1.yaml')
    a=p.parse_args()
    if a.stage in ('h','all'):
        root=h(a.input,a.config)
        if root is None:return 2
        if a.stage=='h':return 0
        if not g(root):return 3
        independent(root);return 0
    if a.stage=='g':return 0 if g(a.input) else 3
    independent(a.input);return 0

if __name__=='__main__':raise SystemExit(main())
