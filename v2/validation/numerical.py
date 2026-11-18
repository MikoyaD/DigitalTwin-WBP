"""Independent sampled-reference checks. PASS here never certifies collision safety."""
from dataclasses import dataclass
import numpy as np
from ..core.contracts import (Status, PhaseType, ValidationResult, ContractError,
                              TrajectoryRecord, LimitSet, array, require)
from ..core.math_tools import differential_kinematics
from ..core.limits import constraint_enabled


@dataclass(frozen=True)
class Tolerances:
    position: float = 1e-6
    derivative: float = 1e-6
    lateral: float = 1e-8
    relative_limit: float = 1e-4
    absolute_limit: float = 1e-9
    integration_q: float = 1e-3
    integration_v: float = 1e-3
    integration_a: float = 1e-2
    time: float = 1e-9

    def __post_init__(self):
        require(all(np.isfinite(x) and x > 0 for x in vars(self).values()), 'TOLERANCE', 'finite positive tolerances')


def _summary(values, time, unit):
    values = np.asarray(values)
    square_integral = np.trapz(values**2, time, axis=0)
    return dict(peak=np.max(np.abs(values), axis=0).tolist(),
                rms=np.sqrt(square_integral/(time[-1]-time[0])).tolist(),
                integral_squared=square_integral.tolist(), unit=unit)


def jerk_metrics(record):
    return dict(xy=_summary(np.linalg.norm(record.jerk[:, :2], axis=1), record.time_s, 'm/s^3'),
                theta=_summary(record.jerk[:, 2], record.time_s, 'rad/s^3'),
                arm_each=_summary(record.jerk[:, 3:], record.time_s, 'rad/s^3'))


def integration_errors(record):
    """Cumulative trapezoidal integrals on actual timestamps, no angle wrapping.

    Errors include quadrature truncation and do not establish intersample truth.
    """
    dt = np.diff(record.time_s)[:, None]
    output = {}
    for name, state, derivative in [('q_from_v', record.q, record.velocity),
                                    ('v_from_a', record.velocity, record.acceleration),
                                    ('a_from_j', record.acceleration, record.jerk)]:
        integral = np.cumsum(dt*(derivative[:-1]+derivative[1:])/2, axis=0)
        output[name] = np.max(np.abs(state[1:]-state[0]-integral), axis=0)
    return output


def validate(record, limits, expected_start, expected_goal, tolerances=None, *, require_rest=True):
    tol = Tolerances() if tolerances is None else tolerances
    result = ValidationResult(Status.PASS, Status.UNKNOWN)
    # Reconstruct to recheck all supplied fields, rather than trusting a label.
    try:
        require(isinstance(record, TrajectoryRecord) and isinstance(limits, LimitSet), 'INPUT_TYPE', 'record/limits')
        record = TrajectoryRecord(**vars(record))
        limits = LimitSet(**vars(limits))
        start, goal = [array(q, (9,), 'requested endpoint') for q in (expected_start, expected_goal)]
        require(record.schema.reference_point == limits.reference_point, 'REFERENCE_POINT', 'record/limit mismatch')
    except ContractError as e:
        return ValidationResult(Status.FAIL, Status.UNKNOWN, [e.code], {'input': str(e)})

    def check(name, condition, code, detail):
        result.checks[name] = dict(status='PASS' if condition else 'FAIL', detail=detail)
        if not condition:
            result.failure_codes.append(code)
            result.numerical_status = Status.FAIL

    def bound(name, values, maximum, unit):
        maximum = np.asarray(maximum)
        peak = np.max(np.abs(values), axis=0)
        # The opt-in nine-DOF contract is a hard componentwise bound, not the
        # legacy validator's 1e-4 relative allowance. Retain a 1e-12 numerical
        # roundoff allowance; all existing no-profile callers keep old behavior.
        nine_dof = ('xy_component_velocity','xy_component_acceleration',
                    'angular_velocity','angular_acceleration','arm_velocity','arm_acceleration')
        if limits.metadata.get('constraint_profile') and name in nine_dof:
            ok = bool(np.all(peak <= maximum + 1e-12))
        else:
            ok = bool(np.all(peak <= maximum*(1+tol.relative_limit)+tol.absolute_limit))
        detail = dict(peak=np.asarray(peak).tolist(), limit=maximum.tolist(), unit=unit,
                      max_ratio=float(np.max(peak/maximum)))
        if not constraint_enabled(limits, name):
            result.checks[name] = dict(status='DISABLED', detail=detail)
        else:
            check(name, ok, 'LIMIT_'+name.upper(), detail)

    check('input', True, 'INPUT', 'finite SI nine-state arrays; strictly increasing actual time')
    for name, q, expected in [('start', record.q[0], start), ('goal', record.q[-1], goal)]:
        error = q-expected
        error[2] = np.arctan2(np.sin(error[2]), np.cos(error[2]))
        check(name, bool(np.max(np.abs(error)) <= tol.position), 'ENDPOINT_'+name.upper(), error.tolist())
    q, v, a, j, t = record.q, record.velocity, record.acceleration, record.jerk, record.time_s
    if require_rest:
        check('rest_boundaries', bool(np.max(np.abs(v[[0, -1]])) <= tol.derivative and
                                     np.max(np.abs(a[[0, -1]])) <= tol.derivative),
              'BOUNDARY_DERIVATIVE', 'rest-to-rest velocity and acceleration')
    check('arm_position', bool(np.all(q[:, 3:] >= limits.arm_lower.value-tol.position) and
                              np.all(q[:, 3:] <= limits.arm_upper.value+tol.position)),
          'POSITION_LIMIT', 'joint1..6, rad; XY workspace bounds unavailable, not inferred')
    result.checks['xy_position_bounds'] = dict(status='UNKNOWN', detail='no workspace position box supplied')
    bound('xy_component_velocity', v[:, :2], limits.xy_component_velocity.value, 'm/s')
    bound('xy_component_acceleration', a[:, :2], limits.xy_component_acceleration.value, 'm/s^2')
    bound('xy_speed', np.linalg.norm(v[:, :2], axis=1), limits.xy_speed.value, 'm/s')
    bound('xy_acceleration', np.linalg.norm(a[:, :2], axis=1), limits.xy_acceleration.value, 'm/s^2')
    bound('angular_velocity', v[:, 2], limits.angular_velocity.value, 'rad/s')
    bound('angular_acceleration', a[:, 2], limits.angular_acceleration.value, 'rad/s^2')
    bound('arm_velocity', v[:, 3:], limits.arm_velocity.value, 'rad/s')
    bound('arm_acceleration', a[:, 3:], limits.arm_acceleration.value, 'rad/s^2')
    kin = differential_kinematics(q, v, a, float(limits.wheel_radius.value), float(limits.wheel_separation.value))
    bound('tangential_acceleration', kin['forward_acceleration'], limits.tangential_acceleration.value, 'm/s^2')
    bound('wheel_velocity', kin['wheel_velocity'], limits.wheel_velocity.value, 'rad/s; order LEFT,RIGHT')
    if limits.wheel_acceleration is not None:
        bound('wheel_acceleration', kin['wheel_acceleration'], limits.wheel_acceleration.value, 'rad/s^2; LEFT,RIGHT')
    else:
        result.checks['wheel_acceleration'] = dict(status='UNKNOWN', detail='optional bound absent; not verified')
    check('nonholonomic', bool(np.max(np.abs(kin['lateral'])) <= tol.lateral), 'LATERAL_RESIDUAL',
          _summary(kin['lateral'], t, 'm/s'))
    if record.phase_type == PhaseType.TRANSLATE:
        check('forward', bool(np.min(kin['forward']) >= -tol.absolute_limit), 'BACKWARD_MOTION', float(np.min(kin['forward'])))
    fixed = {PhaseType.SPIN: [0, 1, 3, 4, 5, 6, 7, 8],
             PhaseType.ARM_ONLY: [0, 1, 2], PhaseType.HOLD: list(range(9))}.get(record.phase_type, [])
    if fixed:
        ok = np.max(np.abs(q[:, fixed]-q[0, fixed])) <= tol.position
        ok = ok and all(np.max(np.abs(d[:, fixed])) <= tol.derivative for d in (v, a, j))
        check('phase_semantics', bool(ok), 'PHASE_SEMANTICS', record.phase_type.value)
    if record.phase_type == PhaseType.SPIN and constraint_enabled(limits, 'spin_jerk'):
        if limits.spin_jerk is None:
            check('spin_jerk', False, 'MISSING_SPIN_JERK', 'explicit spin jerk bound required')
        else:
            bound('spin_jerk', j[:, 2], limits.spin_jerk.value, 'rad/s^3')
    elif record.phase_type == PhaseType.SPIN:
        result.checks['spin_jerk'] = dict(status='DISABLED', detail=dict(
            peak=float(np.max(np.abs(j[:, 2]))), unit='rad/s^3'))
    errors = integration_errors(record)
    for name, value in errors.items():
        threshold = {'q_from_v': tol.integration_q, 'v_from_a': tol.integration_v,
                     'a_from_j': tol.integration_a}[name]
        check(name, bool(np.max(value) <= threshold), 'INCONSISTENT_'+name.upper(),
              dict(per_coordinate=value.tolist(), tolerance=threshold,
                   note='cumulative actual-time trapezoidal check; finite-resolution only'))
    result.metrics.update(duration_s=float(t[-1]-t[0]),
                          xy_length_m=float(np.trapz(np.linalg.norm(v[:, :2], axis=1), t)),
                          jerk=jerk_metrics(record),
                          spin_angle_rad=float(np.trapz(np.abs(v[:, 2]), t)) if record.phase_type == PhaseType.SPIN else 0.,
                          spin_time_s=float(t[-1]-t[0]) if record.phase_type == PhaseType.SPIN else 0.)
    result.checks['collision'] = dict(status='UNKNOWN', detail='NO_PLANNING_SCENE_BACKEND')
    return result


def validate_junctions(records, tolerances=None):
    tol = Tolerances() if tolerances is None else tolerances
    result = ValidationResult(Status.PASS, Status.UNKNOWN, coverage='one_sided_phase_endpoints')
    require(len(records) >= 2, 'DIMENSION', 'need two or more phases')
    for i, (left, right) in enumerate(zip(records[:-1], records[1:])):
        identity = (left.trial_id == right.trial_id and left.parent_path_hash == right.parent_path_hash
                    and left.planner_id == right.planner_id and left.schema == right.schema)
        time_error = abs(left.time_s[-1]-right.time_s[0])
        errors = [float(np.max(np.abs(getattr(left, field)[-1]-getattr(right, field)[0])))
                  for field in ('q', 'velocity', 'acceleration')]
        # No modulo-2pi at seams: a branch jump must not be hidden.
        ok = identity and time_error <= tol.time and errors[0] <= tol.position and max(errors[1:]) <= tol.derivative
        result.checks['junction_%d' % i] = dict(status='PASS' if ok else 'FAIL', identity=identity,
             time_error_s=time_error, q_v_a_errors=errors,
             jerk_left=left.jerk[-1].tolist(), jerk_right=right.jerk[0].tolist())
        if not ok:
            result.numerical_status = Status.FAIL
            result.failure_codes.append('JUNCTION_DISCONTINUITY')
    return result
