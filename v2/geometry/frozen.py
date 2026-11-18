"""Small, deterministic geometry contract shared by G and future time laws."""
from dataclasses import asdict
import numpy as np
from ..core.contracts import PhaseType, StateSchema, require
from ..scripts.validate_g import reconstruct_phase


def freeze(path, geometry):
    phases = []
    for phase in geometry['phases']:
        spline = getattr(phase.evaluator, 'geometry_spline', None)
        item = dict(id=phase.phase_id, kind=phase.phase_type.value,
                    start=phase.start.tolist(), end=phase.end.tolist(),
                    provenance=phase.provenance)
        if spline is not None:
            item.update(controls=spline.controls.tolist(), knots=spline.knots.tolist())
        phases.append(item)
    return dict(version=1, parent_path_hash=path.parent_path_hash,
                schema=asdict(path.schema), phases=phases, decisions=geometry['decisions'])


def thaw(record):
    if record.get('contract_version') == 2:
        from .unified_block import thaw_v2
        return thaw_v2(record)
    require(record.get('version') == 1, 'GEOMETRY_VERSION', 'expected frozen geometry v1')
    schema = StateSchema(**record['schema'])
    phases = [reconstruct_phase(info, schema) for info in record['phases']]
    require(bool(phases), 'EMPTY_GEOMETRY', 'at least one motion phase required')
    for phase in phases:
        phase.evaluate(np.array([0., .125, .5, .875, 1.]))
    for a, b in zip(phases, phases[1:]):
        require(np.max(np.abs(a.end-b.start)) < 1e-8,
                'GEOMETRY_SEAM', '%s to %s' % (a.phase_id, b.phase_id))
    return dict(phases=phases, decisions=record['decisions'])
