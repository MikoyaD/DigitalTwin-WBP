"""Analytic spline, heading and time-chain derivatives; no optimizers or ROS."""
from dataclasses import dataclass
import numpy as np
from numpy.polynomial import Polynomial
from scipy.interpolate import BSpline, PPoly
from .contracts import PathJet, Status, array, vector, require, ContractError


def quintic_knots(n_control):
    require(isinstance(n_control, int) and n_control >= 6, 'SPLINE_SIZE', 'degree five needs >=6 controls')
    return np.r_[np.zeros(6), np.linspace(0, 1, n_control-4)[1:-1], np.ones(6)]


def checked_knots(knots, n_control):
    t = vector(knots, 'knots')
    require(len(t) == n_control+6 and np.all(np.diff(t) >= 0), 'KNOTS', 'invalid degree-five knot vector')
    require(np.all(t[:6] == 0) and np.all(t[-6:] == 1), 'KNOTS', 'clamped [0,1] endpoints required')
    inner = t[6:-6]
    require(np.all((inner > 0) & (inner < 1)) and np.all(np.diff(inner) > 0), 'KNOTS', 'simple interior knots required for C4')
    return t


def quintic_basis(s, n_control, derivative=0, knots=None):
    require(isinstance(derivative, int) and 0 <= derivative <= 4, 'DERIVATIVE_ORDER', 'supported orders 0..4')
    s = vector(s, 's')
    require(np.all((s >= 0) & (s <= 1)), 'PARAMETER_RANGE', 'no extrapolation')
    t = checked_knots(quintic_knots(n_control) if knots is None else knots, n_control)
    spline = BSpline(t, np.eye(n_control), 5, extrapolate=False)
    return spline.derivative(derivative)(s)


class QuinticSpline:
    """Evaluate explicit coefficients only; does not fit or optimize a geometry."""
    def __init__(self, controls, knots=None):
        c = np.asarray(controls)
        require(c.ndim == 2 and c.shape[0] >= 6, 'SPLINE_SIZE', 'controls need (M>=6,d)')
        self.controls = array(c, c.shape, 'controls')
        self.knots = checked_knots(quintic_knots(len(c)) if knots is None else knots, len(c))

    def evaluate(self, s, derivative=0):
        return quintic_basis(s, len(self.controls), derivative, self.knots).dot(self.controls)


@dataclass(frozen=True)
class Parameterization:
    s: np.ndarray
    retained_indices: np.ndarray
    xy_polyline_length: float


def whole_body_parameter(q, length_scale=1.0, angle_scales=None):
    q = np.asarray(q)
    require(q.ndim == 2 and len(q) >= 2, 'DIMENSION', 'q needs at least two rows')
    q = array(q, (len(q), 9), 'q')
    require(np.isfinite(length_scale) and length_scale > 0, 'PARAMETER_SCALE', 'positive length scale')
    scales = array(np.ones(6) if angle_scales is None else angle_scales, (6,), 'angle_scales')
    require(np.all(scales > 0), 'PARAMETER_SCALE', 'positive angular scales')
    # Only exact full-state duplicates collapse; no arm or theta changes disappear.
    keep = np.r_[0, 1+np.nonzero(np.any(np.diff(q, axis=0) != 0, axis=1))[0]]
    z = q[keep][:, [0, 1, 3, 4, 5, 6, 7, 8]]
    require(len(z) >= 2, 'DEGENERATE_PATH', 'all states duplicate; explicit HOLD required')
    dz = np.diff(z, axis=0) / np.r_[length_scale, length_scale, scales]
    d = np.linalg.norm(dz, axis=1)
    require(np.all(d > 0), 'NEEDS_PHASE_SPLIT', 'theta-only interval: classify SPIN before parameterization')
    cumulative = np.r_[0., np.cumsum(d)]
    return Parameterization(cumulative/cumulative[-1], keep,
                            float(np.sum(np.linalg.norm(np.diff(q[:, :2], axis=0), axis=1))))


@dataclass(frozen=True)
class RegularityResult:
    status: Status
    minimum_speed: float
    parameter_at_minimum: float
    reason: str


def xy_regularity(spline, threshold=1e-9):
    """Numerical polynomial-extrema audit over every span (not a formal certificate)."""
    require(spline.controls.shape[1] >= 2, 'DIMENSION', 'XY columns missing')
    require(np.isfinite(threshold) and threshold > 0, 'TOLERANCE', 'regularity threshold')
    pp = [PPoly.from_spline((spline.knots, spline.controls[:, j], 5)).derivative(1) for j in (0, 1)]
    minimum, location = np.inf, 0.
    for i, (lo, hi) in enumerate(zip(pp[0].x[:-1], pp[0].x[1:])):
        if hi <= lo or lo < 0 or hi > 1:
            continue
        h = hi-lo
        polys = [Polynomial(p.c[:, i][::-1]*h**np.arange(len(p.c))) for p in pp]
        norm2 = polys[0]*polys[0]+polys[1]*polys[1]
        try:
            roots = norm2.deriv().roots()
        except np.linalg.LinAlgError:
            return RegularityResult(Status.UNKNOWN, float('nan'), lo, 'ROOT_SOLVER_FAILED')
        if any(0 < abs(r.imag) < 1e-5 and -1e-7 <= r.real <= 1+1e-7 for r in roots):
            return RegularityResult(Status.UNKNOWN, float('nan'), lo, 'AMBIGUOUS_EXTREMUM_ROOT')
        candidates = [0., 1.] + [float(r.real) for r in roots if abs(r.imag) < 1e-12 and 0 <= r.real <= 1]
        for u in candidates:
            s = lo+h*u
            speed = float(np.linalg.norm(spline.evaluate([s], 1)[0, :2]))
            if speed < minimum:
                minimum, location = speed, s
    require(np.isfinite(minimum), 'KNOTS', 'no valid polynomial span')
    return RegularityResult(Status.PASS if minimum > threshold else Status.FAIL,
                            minimum, location, 'NUMERICAL_EXTREMA' if minimum > threshold else 'DEGENERATE_TANGENT')


def heading_jet(s, p1, p2, p3, p4, anchor, minimum_tangent=1e-9, max_step=np.pi/2):
    """G3 derivatives. Anchor fixes first sample's 2pi branch, NOT a global winding oracle.

    Caller must refine the tangent grid before unwrapping. Sparse samples cannot
    prove that an unobserved full revolution did not occur between them.
    """
    s = vector(s, 'heading parameters')
    require(len(s) == 1 or np.all(np.diff(s) > 0), 'PARAMETER_ORDER', 'heading samples must increase')
    require(np.isfinite(anchor), 'NONFINITE', 'heading anchor')
    require(np.isfinite(minimum_tangent) and minimum_tangent > 0 and 0 < max_step < np.pi,
            'TOLERANCE', 'heading thresholds')
    p1, p2, p3, p4 = [array(p, (len(s), 2), 'XY derivative') for p in (p1, p2, p3, p4)]
    cross = lambda a, b: a[:, 0]*b[:, 1]-a[:, 1]*b[:, 0]
    D = np.sum(p1*p1, axis=1)
    require(np.all(D > minimum_tangent**2), 'DEGENERATE_TANGENT', 'atan2 requires nonzero XY tangent')
    raw = np.arctan2(p1[:, 1], p1[:, 0])
    steps = np.arctan2(np.sin(np.diff(raw)), np.cos(np.diff(raw)))
    require(np.all(np.abs(steps) <= max_step), 'HEADING_GRID_TOO_COARSE', 'subdivide tangent grid')
    theta = np.unwrap(raw)
    theta += 2*np.pi*np.round((anchor-theta[0])/(2*np.pi))
    C, C1 = cross(p1, p2), cross(p1, p3)
    C2 = cross(p2, p3)+cross(p1, p4)
    D1 = 2*np.sum(p1*p2, axis=1)
    D2 = 2*np.sum(p2*p2+p1*p3, axis=1)
    h1 = C/D
    h2 = C1/D-C*D1/D**2
    h3 = C2/D-2*C1*D1/D**2-C*D2/D**2+2*C*D1**2/D**3
    return theta, h1, h2, h3


def translation_jet(s, derivatives, heading_anchor):
    require(len(derivatives) == 5, 'DERIVATIVE_ORDER', 'need z, z_s ... z_ssss')
    z = [array(d, (len(s), 8), 'geometry derivative') for d in derivatives]
    theta = heading_jet(s, *(d[:, :2] for d in z[1:]), anchor=heading_anchor)
    return PathJet(*(np.column_stack((z[k][:, :2], theta[k], z[k][:, 2:])) for k in range(4)))


def spin_jet(s, start, angle):
    s = vector(s, 's')
    require(np.all((s >= 0) & (s <= 1)) and np.isfinite(angle), 'PARAMETER_RANGE', 'spin')
    start = array(start, (9,), 'spin start')
    q = np.tile(start, (len(s), 1)); q[:, 2] += angle*s
    qs = np.zeros_like(q); qs[:, 2] = angle
    return PathJet(q, qs, np.zeros_like(q), np.zeros_like(q))


def arm_only_jet(base_pose, arm_derivatives):
    require(len(arm_derivatives) == 4, 'DERIVATIVE_ORDER', 'arm jet orders 0..3')
    n = len(arm_derivatives[0]); base = array(base_pose, (3,), 'base pose')
    arms = [array(d, (n, 6), 'arm derivative') for d in arm_derivatives]
    return PathJet(np.column_stack((np.tile(base, (n, 1)), arms[0])),
                   *(np.column_stack((np.zeros((n, 3)), a)) for a in arms[1:]))


def hold_jet(q, n):
    require(isinstance(n, int) and n > 0, 'DIMENSION', 'hold sample count')
    q = np.tile(array(q, (9,), 'hold state'), (n, 1))
    return PathJet(q, np.zeros_like(q), np.zeros_like(q), np.zeros_like(q))


def quintic_time_jet(t, duration):
    t = vector(t, 'local times')
    require(np.isfinite(duration) and duration > 0, 'DURATION', 'positive duration required')
    require(np.all((t >= 0) & (t <= duration)), 'TIME_RANGE', 'local times outside phase')
    u = t/duration
    # Symmetry and factored derivatives avoid cancellation/negative speed near 1.
    w = np.minimum(u, 1-u)
    small = w**3*(10-15*w+6*w*w)
    progress = np.where(u <= .5, small, 1-small)
    return (progress, 30*u*u*(1-u)**2/duration,
            60*u*(1-u)*(1-2*u)/duration**2,
            60*(1-6*u+6*u*u)/duration**3)


def chain_rule(jet, speed, acceleration, jerk):
    n = len(jet.q)
    v, a, j = [array(x, (n,), 'time-law derivative')[:, None] for x in (speed, acceleration, jerk)]
    require(np.all(v >= 0), 'REVERSE_PARAMETER', 's(t) must be nondecreasing')
    return (jet.qs*v, jet.qss*v*v+jet.qs*a,
            jet.qsss*v**3+3*jet.qss*v*a+jet.qs*j)


def scale_time(time, velocity, acceleration, jerk, factor):
    t = vector(time, 'time', 2)
    require(np.all(np.diff(t) > 0), 'TIME_NOT_INCREASING', 'scaling time')
    require(np.isfinite(factor) and factor > 0, 'TIME_SCALE', 'positive factor')
    arrays = [array(x, (len(t), 9), 'time derivative') for x in (velocity, acceleration, jerk)]
    return (t[0]+factor*(t-t[0]), arrays[0]/factor, arrays[1]/factor**2, arrays[2]/factor**3)


def differential_kinematics(q, velocity, acceleration, radius, separation):
    n = len(q)
    q, v, a = [array(x, (n, 9), 'kinematic state') for x in (q, velocity, acceleration)]
    require(np.isfinite(radius) and radius > 0 and np.isfinite(separation) and separation > 0,
            'INVALID_LIMIT', 'wheel geometry')
    ct, st = np.cos(q[:, 2]), np.sin(q[:, 2])
    forward = ct*v[:, 0]+st*v[:, 1]
    lateral = -st*v[:, 0]+ct*v[:, 1]
    # Exact derivative of body-projected forward velocity, even on invalid/slipping input.
    forward_acc = ct*a[:, 0]+st*a[:, 1]+v[:, 2]*lateral
    wheels = np.column_stack((forward-separation*v[:, 2]/2,
                              forward+separation*v[:, 2]/2))/radius  # LEFT, RIGHT
    wheel_acc = np.column_stack((forward_acc-separation*a[:, 2]/2,
                                 forward_acc+separation*a[:, 2]/2))/radius
    return dict(forward=forward, lateral=lateral, forward_acceleration=forward_acc,
                wheel_velocity=wheels, wheel_acceleration=wheel_acc)
