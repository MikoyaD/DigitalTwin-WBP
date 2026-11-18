"""Explicit synthetic smoke examples only. Not robot experiments."""
import json
import uuid
from pathlib import Path
from hashlib import sha256
from dataclasses import asdict
import numpy as np
from .tests.support import limits,SCHEMA
from .core.contracts import PathRecord
from .geometry.phase_segmenter import build_g
from .adapters.run_g import plain


def main():
    root=Path(__file__).resolve().parent;folder=root/'outputs'/('synthetic_'+uuid.uuid4().hex);folder.mkdir(parents=True,exist_ok=False)
    cases=[];line=np.zeros((6,9));line[:,0]=np.linspace(0,1,6);line[:,3]=line[:,0]*.3
    angle=np.linspace(0,1,12);arc=np.zeros((12,9));arc[:,0]=np.sin(angle);arc[:,1]=1-np.cos(angle);arc[:,2]=angle;arc[:,4]=angle*.2
    for name,q in [('synthetic_line_arm',line),('synthetic_arc_arm',arc)]:
        path=PathRecord(name,'synthetic',sha256(q.tobytes()).hexdigest(),'synthetic',SCHEMA,q,None,q[0],q[-1],{'not_robot_experiment':True})
        out=build_g(path,limits());row=dict(name=name,not_robot_experiment=True,duration_s=out['duration_s'],xy_length_m=out['xy_length_m'],backend_s=out['backend_s'],
                                        collision='UNKNOWN',phase_metrics=[v.metrics for v in out['validation']],decisions=out['decisions'])
        # Keep one-sided phase endpoint jerk in addition to unique-time export.
        payload=dict(summary=row,path=asdict(path),limits=asdict(limits()),output=out['output'],records=[asdict(r) for r in out['records']])
        with (folder/(name+'.json')).open('x') as f:json.dump(payload,f,indent=2,default=plain)
        cases.append(row)
    with (folder/'summary.json').open('x') as f:json.dump(cases,f,indent=2,default=plain)
    print(folder);print(json.dumps(cases,default=plain))

if __name__=='__main__':main()
