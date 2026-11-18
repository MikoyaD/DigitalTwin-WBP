"""Bounded diagnostic-center control flow. Never constructs or publishes H."""
from copy import deepcopy


def iterate_center(initial, propose, evaluate, project_all, *, max_rounds=2):
    """Try at most two proposals and three fractions; return diagnostic evidence.

    Callbacks are deliberately separated: evaluate must use true nonlinear
    integrals and project_all must independently solve every original rho QP.
    A positive projection is not an H; geometry verification is a later gate.
    """
    if max_rounds not in (1, 2):
        raise ValueError('FEEDBACK_BUDGET')
    center = deepcopy(initial)
    trust = 1.
    rounds = []
    for index in range(max_rounds):
        proposal = propose(center, trust)
        event = dict(round=index+1, trust_factor=trust, proposal_status=proposal.get('status'), steps=[])
        rounds.append(event)
        if proposal.get('status') != 'DIAGNOSTIC_PROPOSAL':
            event['next'] = 'STOP_PROPOSAL_FAILURE'
            break
        accepted = False
        for fraction in (1., .5, .25):
            candidate, metric, hard_ok = evaluate(center, proposal, fraction)
            record = dict(fraction=fraction, metric=metric, hard_ok=bool(hard_ok))
            event['steps'].append(record)
            if not hard_ok:
                record['status'] = 'HARD_JET_OR_PROFILE_FAILURE'
                continue
            projections = project_all(candidate)
            record['projections'] = projections
            if not projections:
                record['status'] = 'NO_MOVE_PROJECTION'
                continue
            if all(x.get('status') == 'PROJECTED' for x in projections):
                record['status'] = 'ALL_PROJECTED_NOT_H'
                return dict(status='ALL_PROJECTED_NOT_H', center=candidate,
                            projections=projections, rounds=rounds)
            previous = center.get('previously_feasible', [])
            preserved = all(i < len(projections) and projections[i].get('status') == 'PROJECTED' for i in previous)
            predicted = metric['predicted_decrease']; actual = metric['actual_decrease']
            if preserved and predicted > 1e-8 and actual >= .1*predicted:
                center = candidate
                center['previously_feasible'] = [i for i,x in enumerate(projections) if x.get('status') == 'PROJECTED']
                record['status'] = 'DIAGNOSTIC_CENTER_ACCEPTED'
                event['next'] = 'NEXT_ROUND' if index+1 < max_rounds else 'BUDGET_EXHAUSTED'
                accepted = True
                break
            record['status'] = 'NO_TRUE_DESCENT_OR_FEASIBLE_LOST'
        if not accepted:
            trust *= .5
            event['next'] = 'SHRINK_TRUST' if index+1 < max_rounds else 'BUDGET_EXHAUSTED'
    return dict(status='FIXED_JET_FEEDBACK_EXHAUSTED', center=center, rounds=rounds)
