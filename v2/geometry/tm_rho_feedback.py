"""V2-TM-RHO-FEEDBACK-v1 numerical primitives; diagnostic values are not trajectories.

This module is independent of the v3 builder.  A complete candidate builder is
required before these functions may be used to freeze or publish a trajectory.
"""
import numpy as np
import cvxpy as cp
from .tm_builder import _basis, _rho_matrix
from ..core.math_tools import QuinticSpline

TUBE = 0.0999
SOLVER = dict(solver='CLARABEL', max_iter=200, tol_feas=1e-10,
              tol_gap_abs=1e-10, tol_gap_rel=1e-10)


def integral_jacobians(x, theta, rho, length):
    """Return true displacements and dP/d(rho controls), dP/d(theta controls).

    All integrals are in local xi, with both knot sets included in each panel.
    The rho spline stores derivative with respect to global u, hence length.
    """
    x = np.asarray(x, float)
    if x.ndim != 1 or not np.isfinite(x).all() or np.any(x < 0) or np.any(x > 1) or not np.isfinite(length) or length <= 0:
        raise ValueError('INVALID_INTEGRATION_INPUT')
    n, m = len(rho.controls), len(theta.controls)
    if rho.controls.shape != (n, 1) or theta.controls.shape != (m, 1):
        raise ValueError('INVALID_SPLINE_DIMENSION')
    cuts0 = np.unique(np.r_[rho.knots, theta.knots])
    nodes, weights = np.polynomial.legendre.leggauss(24)
    A = np.zeros((len(x), 2, n)); K = np.zeros((len(x), 2, m))
    for i, end in enumerate(x):
        cuts = np.r_[0., cuts0[(cuts0 > 0) & (cuts0 < end)], end]
        for a, b in zip(cuts[:-1], cuts[1:]):
            if b <= a:
                continue
            t = (a+b)/2 + (b-a)*nodes/2
            th = theta.evaluate(t)[:, 0]; rr = rho.evaluate(t)[:, 0]
            e = np.column_stack((np.cos(th), np.sin(th)))
            ep = np.column_stack((-np.sin(th), np.cos(th)))
            factor = length * (b-a)/2
            A[i] += factor*np.einsum('i,ij,ik->jk', weights, e, _basis_knots(t, n, rho.knots))
            K[i] += factor*np.einsum('i,i,ij,ik->jk', weights, rr, ep, _basis_knots(t, m, theta.knots))
    return np.einsum('idn,n->id', A, rho.controls[:, 0]), A, K


def _basis_knots(t, n, knots):
    from ..core.math_tools import quintic_basis
    return quintic_basis(np.asarray(t), n, knots=knots)


def _solve(problem, original=False):
    kwargs = dict(solver='CLARABEL', max_iter=200, tol_gap_abs=1e-10, tol_feas=1e-10)
    if not original:
        kwargs['tol_gap_rel'] = 1e-10
    problem.solve(**kwargs)
    return problem.status


def global_jet_residual(J, d, controls, length, orders):
    """Return rho boundary-jet residuals in global-u units, without scaling away errors."""
    J, d, controls = np.asarray(J, float), np.asarray(d, float), np.asarray(controls, float)
    orders = np.asarray(orders, int)
    if J.ndim != 2 or J.shape[1] != 14 or d.shape != (len(J),) or controls.shape != (14,) or orders.shape != d.shape or np.any((orders < 0) | (orders > 2)) or not np.isfinite(length) or length <= 0:
        raise ValueError('INVALID_GLOBAL_JET_INPUT')
    if not all(np.isfinite(x).all() for x in (J, d, controls)):
        raise ValueError('INVALID_GLOBAL_JET_INPUT')
    return (J @ controls - d) / length**orders


def nonlinear_step_check(moves, beta, controls, delta_theta, delta_rho, fraction):
    """Evaluate actual and first-order candidate reductions; never accepts an H."""
    if fraction not in (1., .5, .25) or len(moves) != len(controls) or len(controls) != len(delta_rho) or len(beta) != len(delta_theta):
        raise ValueError('INVALID_STEP')
    before = after = predicted = 0.
    per_move = []
    for i, move in enumerate(moves):
        k = move['profile']; x = move['x']; h = move['h']
        c = np.asarray(controls[i], float); b = np.asarray(beta[k], float)
        db = np.asarray(delta_theta[k], float); dc = np.asarray(delta_rho[i], float)
        theta = QuinticSpline(b[:,None]); rho = QuinticSpline(c[:,None])
        p, A, K = integral_jacobians(x, theta, rho, h)
        newp, _, _ = integral_jacobians(x, QuinticSpline((b+fraction*db)[:,None]), QuinticSpline((c+fraction*dc)[:,None]), h)
        predicted_p = p+fraction*(np.einsum('idn,n->id',A,dc)+np.einsum('idm,m->id',K,db))
        target = np.asarray(move['p1'])-np.asarray(move['p0']); ref=np.asarray(move['ref'])
        def violation(v):
            return max(float(np.max(np.abs(v[-1]-target))), max(0.,float(np.max(np.abs(v-ref)))-TUBE))/TUBE
        b0, b1, bp = violation(p), violation(newp), violation(predicted_p)
        before=max(before,b0);after=max(after,b1);predicted=max(predicted,bp)
        per_move.append(dict(segment=move['segment'], closure_m=float(np.max(np.abs(newp[-1]-target))), tube_excess_m=max(0.,float(np.max(np.abs(newp-ref)))-TUBE), linearization_error_m=float(np.max(np.abs(newp-predicted_p)))))
    return dict(status='DIAGNOSTIC_STEP', before=before, after=after, predicted=predicted,
                predicted_decrease=before-predicted, actual_decrease=before-after, moves=per_move)


def accepts_diagnostic_step(result, preserved):
    """Only selects an iterative center; it does not certify or construct geometry."""
    return bool(preserved and result['predicted_decrease'] > 1e-8 and result['actual_decrease'] >= .1*result['predicted_decrease'])


def diagnostic_initialize(A, ref, delta, J, d, scale):
    """Lexicographic elastic XY init; J and nonnegativity are never relaxed.

    Returns only a labelled diagnostic center, not an acceptable geometry.
    """
    A, ref, delta, J, d = map(lambda a: np.asarray(a, float), (A, ref, delta, J, d))
    if A.ndim != 3 or A.shape[1:] != (2, 14) or ref.shape != (len(A), 2) or delta.shape != (2,) or J.ndim != 2 or J.shape[1] != 14 or d.shape != (len(J),) or not np.isfinite(scale) or scale <= 0:
        raise ValueError('INVALID_INITIALIZATION_INPUT')
    c = cp.Variable(14); alpha = cp.Variable(nonneg=True)
    pred = cp.vstack([A[:, k, :] @ c for k in range(2)]).T
    constraints = [c >= 0, pred[-1] - delta <= alpha*TUBE,
                   pred[-1] - delta >= -alpha*TUBE,
                   pred-ref <= TUBE*(1+alpha), pred-ref >= -TUBE*(1+alpha)]
    if len(J):
        row = np.linalg.norm(J, axis=1)
        if np.any(row == 0): raise ValueError('ZERO_JET_ROW')
        constraints.append((J/row[:, None])@c == d/row)
    first = cp.Problem(cp.Minimize(alpha), constraints)
    status = _solve(first)
    if status not in ('optimal','optimal_inaccurate') or alpha.value is None:
        return dict(status='JET_OR_SOLVER_FAILURE', solver_status=status)
    best = float(alpha.value)
    # The tie-break reference only chooses a center; it is never published.
    seed = np.full(14, scale)
    if len(J):
        seed += J.T @ np.linalg.lstsq(J@J.T, d-J@seed, rcond=None)[0]
    second = cp.Problem(cp.Minimize(cp.sum_squares((c-seed)/scale)), constraints+[alpha <= best+1e-10])
    status2 = _solve(second)
    if status2 not in ('optimal','optimal_inaccurate') or c.value is None:
        return dict(status='INITIALIZATION_INCONCLUSIVE', solver_status=status2, alpha_min=best)
    value = np.asarray(c.value).copy(); jet = float(np.max(np.abs(J@value-d))) if len(J) else 0.
    p = np.einsum('idn,n->id', A, value)
    xy = float(np.max(np.abs(p[-1]-delta)))
    excess = float(max(0., np.max(np.abs(p-ref))-TUBE))
    if value.min() < -1e-9 or jet > 1e-6:
        return dict(status='INITIALIZATION_INCONCLUSIVE', solver_status=status2, jet_error=jet, min_rho=float(value.min()))
    return dict(status='DIAGNOSTIC_INIT', controls=value, alpha_min=best,
                solver_status=status2, jet_error=jet, min_rho=float(value.min()),
                closure_m=xy, tube_excess_m=excess)


def exact_rho_projection(A, ref, delta, J, d):
    """Original fixed-theta rho QP; every returned success is rechecked."""
    A, ref, delta, J, d = map(lambda a: np.asarray(a, float), (A, ref, delta, J, d))
    c = cp.Variable(14)
    pred = cp.vstack([A[:, k, :] @ c for k in range(2)]).T
    con = [c >= 0, pred[-1] == delta, pred-ref <= TUBE, pred-ref >= -TUBE]
    if len(J): con.append(J@c == d)
    d2 = _basis(np.linspace(0, 1, 41), 14, 2)
    p = cp.Problem(cp.Minimize(cp.sum_squares(pred-ref)/len(A)+1e-6*cp.sum_squares(d2@c)/41),con)
    status = _solve(p, original=True)
    if status not in ('optimal','optimal_inaccurate') or c.value is None:
        return dict(status='NUMERICALLY_INFEASIBLE' if status in ('infeasible','infeasible_inaccurate') else 'SOLVER_INCONCLUSIVE', solver_status=status)
    v = np.asarray(c.value).copy()
    if np.min(v) < -1e-9:
        return dict(status='RECHECK_FAILED', solver_status=status, reason='negative_rho')
    v = np.maximum(v, 0.)
    pxy = np.einsum('idn,n->id', A, v)
    jet = float(np.max(np.abs(J@v-d))) if len(J) else 0.
    closure = float(np.max(np.abs(pxy[-1]-delta)))
    tube = float(np.max(np.abs(pxy-ref)))
    if jet > 1e-6 or closure > 1e-8 or tube > TUBE+1e-9:
        return dict(status='RECHECK_FAILED', solver_status=status, jet_error=jet, closure_m=closure, tube_m=tube)
    return dict(status='PROJECTED', solver_status=status, controls=v,
                jet_error=jet, closure_m=closure, tube_m=tube)
