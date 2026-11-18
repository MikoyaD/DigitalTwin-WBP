"""Nine-coordinate reference metrics; phase/chart one-sided jerk is not interpolated across seams."""
import argparse,json
from pathlib import Path
import numpy as np
from ..core.contracts import NAMES
from ..core.math_tools import differential_kinematics,quintic_time_jet,chain_rule
from ..geometry.frozen import thaw
from ..geometry.unified_block import thaw_v2
from ..timing.feasible_profile import sample
from ..timing.unified_g import sample_block,spin_scope
from ..core.json_io import write_exclusive
from .stage_v2 import load_parent,limits_for,read


def metrics(root):
    root=Path(root).resolve(strict=True);parent,path=load_parent(root);limits=limits_for(parent)
    geometry=read(root/'geometry.json');result=read(root/'G/result.json')
    records=[];segments=[];seams=[];spin_peak=0.
    if geometry.get('contract_version')==2:
        block=thaw_v2(geometry)['blocks'][0];info=result['blocks'][0]
        n=max(int(info['uniform_n']),int(np.ceil(float(info['duration_s'])/.01))+1)
        record,u=sample_block(block,path,float(info['duration_s']),n)
        records=[record];mask=spin_scope(block,u)
        if np.any(mask):spin_peak=float(np.max(np.abs(record.jerk[mask,2])))
        endpoint_jets=[]
        for chart in block.charts:
            lo=int(np.argmin(np.abs(u-chart.s0)));hi=int(np.argmin(np.abs(u-chart.s1)))
            if hi<=lo:raise ValueError('CHART_SAMPLES_MISSING')
            tt=record.time_s[lo:hi+1].copy();jj=record.jerk[lo:hi+1].copy()
            ends=[]
            for k,coord,ti in [(0,chart.s0,tt[0]),(-1,chart.s1,tt[-1])]:
                jet=chart.evaluate_global(np.array([coord]));_,v,a,j=quintic_time_jet(np.array([ti]),float(info['duration_s']))
                deriv=chain_rule(jet,v,a,j);jj[k]=deriv[2][0]
                ends.append((jet.q[0],deriv[0][0],deriv[1][0],deriv[2][0]))
            segments.append((tt,jj));endpoint_jets.append(ends)
            if getattr(chart,'stationary',False):spin_peak=max(spin_peak,float(np.max(np.abs(jj[:,2]))))
        for left,right in zip(endpoint_jets[:-1],endpoint_jets[1:]):
            seams.append({name:float(np.max(np.abs(left[1][i]-right[0][i])))
                          for i,name in enumerate(('q','velocity','acceleration','jerk'))})
    else:
        phases=thaw(geometry)['phases'];offset=0.
        for phase,info in zip(phases,result['phases']):
            n=max(int(info['samples']),int(np.ceil(float(info['duration_s'])/.01))+1)
            r=sample(phase,path,float(info['duration_s']),n,offset)
            records.append(r);segments.append((r.time_s,r.jerk));offset=r.time_s[-1]
            if phase.phase_type.value=='SPIN':spin_peak=max(spin_peak,float(np.max(np.abs(r.jerk[:,2]))))
        for left,right in zip(records[:-1],records[1:]):
            seams.append({name:float(np.max(np.abs(getattr(left,name)[-1]-getattr(right,name)[0])))
                          for name in ('q','velocity','acceleration','jerk')})
    duration=float(result['duration_s']);integral=np.zeros(9);square=np.zeros(9);peak=np.zeros(9)
    for t,j in segments:
        integral+=np.trapz(np.abs(j),t,axis=0);square+=np.trapz(j*j,t,axis=0)
        peak=np.maximum(peak,np.max(np.abs(j),axis=0))
    q=np.vstack([r.q for r in records]);v=np.vstack([r.velocity for r in records]);a=np.vstack([r.acceleration for r in records])
    kin=differential_kinematics(q,v,a,float(limits.wheel_radius.value),float(limits.wheel_separation.value))
    maxv=np.max(np.abs(v),axis=0);maxa=np.max(np.abs(a),axis=0)
    vl=np.r_[limits.xy_component_velocity.value,limits.angular_velocity.value,limits.arm_velocity.value]
    al=np.r_[limits.xy_component_acceleration.value,limits.angular_acceleration.value,limits.arm_acceleration.value]
    return dict(parent_path_hash=path.parent_path_hash,geometry_sha256=result['geometry_sha256'],
        algorithm=result.get('variant'),profile=limits.metadata.get('constraint_profile'),
        reference_duration_s=duration,numerical_status=result['numerical_status'],collision=result['collision'],
        joint_order=list(NAMES),start=records[0].q[0].tolist(),end=records[-1].q[-1].tolist(),
        velocity_peak=maxv.tolist(),velocity_utilization=(maxv/vl).tolist(),
        velocity_units=['m/s']*2+['rad/s']*7,
        acceleration_peak=maxa.tolist(),acceleration_utilization=(maxa/al).tolist(),
        acceleration_units=['m/s^2']*2+['rad/s^2']*7,
        jerk_mean_absolute=(integral/duration).tolist(),jerk_peak=peak.tolist(),jerk_rms=np.sqrt(square/duration).tolist(),
        jerk_units=['m/s^3']*2+['rad/s^3']*7,
        jerk_method='at most 0.01 s common physical-time spacing; phase/chart integration separately; analytic left and right endpoint jerk; time-weighted trapezoids',
        diagnostic_xy_speed_m_s=float(np.max(np.linalg.norm(v[:,:2],axis=1))),
        diagnostic_xy_acceleration_m_s2=float(np.max(np.linalg.norm(a[:,:2],axis=1))),
        diagnostic_wheel_velocity_rad_s=np.max(np.abs(kin['wheel_velocity']),axis=0).tolist(),
        diagnostic_wheel_acceleration_rad_s2=np.max(np.abs(kin['wheel_acceleration']),axis=0).tolist(),
        diagnostic_spin_jerk_rad_s3=spin_peak,lateral_residual_peak_m_s=float(np.max(np.abs(kin['lateral']))),
        seam_max_errors=seams,h_build_s=geometry.get('construction',{}).get('h_build_s'),
        g_build_s=result['backend_s'])


def main():
    p=argparse.ArgumentParser();p.add_argument('--input',required=True);args=p.parse_args()
    out=Path(args.input)/'G'/'paired_metrics_10ms.json';value=metrics(args.input);write_exclusive(out,value)
    print(out);print(json.dumps(dict(duration_s=value['reference_duration_s'],numerical=value['numerical_status'],
                                    collision=value['collision']['status']),ensure_ascii=False))

if __name__=='__main__':main()
