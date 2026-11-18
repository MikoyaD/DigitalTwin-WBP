"""Post-save diagnostics; reads frozen G output only, never changes timing."""
import argparse,json
from pathlib import Path
from ..core.json_io import write_exclusive

def diagnose(folder):
    data=json.loads((folder/'result.json').read_text());phases=[];shares={}
    for p in data.get('blocks',data.get('phases',[])):
        v=p['validation'];m=v['metrics'];duration=p['duration_s'];checks={};candidates={}
        for name,check in v['checks'].items():
            detail=check.get('detail')
            if check.get('status') != 'DISABLED' and isinstance(detail,dict) and 'max_ratio' in detail:
                checks[name]=detail
                order=2 if 'acceleration' in name else (3 if 'jerk' in name else 1)
                candidates[name]=duration*float(detail['max_ratio'])**(1/order)
        shares[p['kind']]=shares.get(p['kind'],0)+duration
        delta=[b-a for a,b in zip(p['start'][3:],p['end'][3:])]
        phases.append(dict(id=p['id'],kind=p['kind'],duration_s=duration,xy_length_m=m['xy_length_m'],spin_angle_rad=m['spin_angle_rad'],arm_net_change_rad=delta,limits=checks,sampled_unit_duration_candidates_s=candidates,sampled_candidate_leader=max(candidates,key=candidates.get) if candidates else 'UNKNOWN',actual_solver_scaling_factor='UNKNOWN: G output does not record solver iterations',evidence='saved independent validation peaks; candidate duration reconstructed under uniform quintic scaling, not proof of exact solver active constraint'))
    return dict(duration_s=data['duration_s'],phases=phases,time_fraction={k:v/data['duration_s'] for k,v in shares.items()},effective_limits=data['limits'],slow_factors=['Rest-to-rest on every phase; phase durations and utilization above are direct evidence.','Motion limits can be model/YAML scaled caps stricter than XY norm research limit.','Exact solver dominant constraint UNKNOWN; do not infer from average speed.'])

def main():
    p=argparse.ArgumentParser();p.add_argument('g_dir');a=p.parse_args();folder=Path(a.g_dir);out=diagnose(folder)
    write_exclusive(folder/'diagnostics.json',out)
    with (folder/'diagnostics.md').open('x') as f:
        f.write('# G post-run diagnostics\n\nTotal %.6f s\n\n'%out['duration_s'])
        for phase in out['phases']:f.write('- %s %s: %.6f s, XY %.6f m, sampled candidate %s; actual scaling UNKNOWN\n'%(phase['id'],phase['kind'],phase['duration_s'],phase['xy_length_m'],phase['sampled_candidate_leader']))
if __name__=='__main__':main()
