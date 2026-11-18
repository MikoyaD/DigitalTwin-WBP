"""Rank verified C3 candidates; failed or unchecked candidates never enter Pareto."""
import argparse,json
from pathlib import Path
import numpy as np
from ..core.json_io import write_exclusive


def promotion(a,baseline,tol=1e-9):
    if a.get('collision')!='PASS' or a.get('numerical')!='PASS' or not a.get('c3') or not a.get('restore'):
        return False
    x=a['metrics'];y=baseline
    return bool(x['duration_s']<=y['duration_s']+tol and x['xy_jerk_peak']<=y['xy_jerk_peak']+tol and x['xy_jerk_rms']<=y['xy_jerk_rms']+tol and np.all(np.asarray(x['angular_arm_peak'])<=np.asarray(y['angular_arm_peak'])+tol) and np.all(np.asarray(x['angular_arm_rms'])<=np.asarray(y['angular_arm_rms'])+tol))


def vector(c):
    m=c['metrics'];return np.r_[m['duration_s'],m['xy_jerk_peak'],m['xy_jerk_rms'],m['angular_arm_peak'],m['angular_arm_rms'],m['left_right_wheel_acceleration_peak'],c['h_time_s'],c['g_time_s'],m['length_xy_m'],m['angle_abs_rad'],c['reference_peak'],c['c3_error']]


def pareto(candidates,tol=1e-12):
    valid=[x for x in candidates if x.get('collision')=='PASS' and x.get('numerical')=='PASS' and x.get('c3') and x.get('restore') and x.get('sampling_comparable')]
    return [a['id'] for a in valid if not any(b is not a and np.all(vector(b)<=vector(a)+tol) and np.any(vector(b)<vector(a)-tol) for b in valid)]


def rank(summary,b_summary=None,common=None):
    metrics=common['candidate'] if common else summary['A']['G']['metrics'];detail=summary['A'];c=dict(id='UHG-C3-A',metrics=metrics,h_time_s=detail['elapsed_s'],g_time_s=detail['G']['backend_s'],reference_peak=detail['tube']['peak'],c3_error=max(v['qsss'] for v in detail['seams'].values()),collision=summary['C']['collision_status'],numerical='PASS' if detail['G']['status']=='NUMERICALLY_VERIFIED' else 'FAIL',c3=max(v['qsss'] for v in detail['seams'].values())<=1e-6,restore=True)
    baseline=common['baseline'] if common else summary['baseline']; comparable=bool(common and common.get('identical_normalized_grid')); c['sampling_comparable']=comparable; candidates=[c];records=(b_summary or summary).get('B',[])
    for item in records:
        if item.get('status')!='NUMERICALLY_VERIFIED':continue
        detail=item['geometry'];compare=(common or {}).get('b_candidates',{}).get(item['candidate_id'],{});candidate=dict(id=item['candidate_id'],metrics=compare.get('metrics',item['metrics']),sampling_comparable=bool(compare.get('identical_normalized_grid')),h_time_s=item['h_time_s'],g_time_s=item['g_time_s'],reference_peak=detail['tube']['peak'],c3_error=max(v['qsss'] for v in detail['seams'].values()),collision=item.get('collision','UNKNOWN'),numerical='PASS',c3=max(v['qsss'] for v in detail['seams'].values())<=1e-6,restore=True)
        candidates.append(candidate)
    for candidate in candidates:candidate['promotion_eligible']=bool(comparable and candidate.get('sampling_comparable') and promotion(candidate,baseline))
    front=pareto(candidates);eligible=[x['id'] for x in candidates if x['promotion_eligible']]
    return dict(parent_hash=summary['parent_hash'],baseline=baseline,candidates=candidates,pareto_nondominated=front,promotion_eligible=eligible,decision='PROMOTION_ELIGIBLE' if eligible else 'NO_PROMOTION_ELIGIBLE',b_candidates=records,common_sampling_verified=comparable,ordering='componentwise Pareto; heterogeneous units not summed; non-verified excluded',comparison_scope='same frozen P, model, scene, limits and jerk sampling procedure')


def main():
    p=argparse.ArgumentParser();p.add_argument('--a',required=True);p.add_argument('--b');p.add_argument('--common');p.add_argument('--output',required=True);args=p.parse_args();a=json.loads(Path(args.a).read_text());b=json.loads(Path(args.b).read_text()) if args.b else None;common=json.loads(Path(args.common).read_text()) if args.common else None
    if b and (b['parent_hash']!=a['parent_hash'] or b['scene']!=a['scene'] or b['limits']!=a['limits']):raise ValueError('PARENT_SCENE_OR_LIMITS_MISMATCH')
    result=rank(a,b,common);write_exclusive(args.output,result);print(json.dumps({'decision':result['decision'],'pareto':result['pareto_nondominated'],'promotion':result['promotion_eligible']},ensure_ascii=False))
if __name__=='__main__':main()
