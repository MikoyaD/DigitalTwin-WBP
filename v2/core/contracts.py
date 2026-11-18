"""Explicit SI contracts. Malformed input raises a coded error, never zero-fills."""
from dataclasses import dataclass, field
from enum import Enum
from typing import Callable, Mapping, Optional, Tuple
import re
import numpy as np

NAMES = ('world_car/x', 'world_car/y', 'world_car/theta') + tuple('joint%d' % i for i in range(1, 7))
POSITION_UNITS = ('m', 'm') + ('rad',) * 7
VELOCITY_UNITS = ('m/s', 'm/s') + ('rad/s',) * 7
ACCELERATION_UNITS = ('m/s^2', 'm/s^2') + ('rad/s^2',) * 7
JERK_UNITS = ('m/s^3', 'm/s^3') + ('rad/s^3',) * 7


class ContractError(ValueError):
    def __init__(self, code, detail):
        self.code = code
        super().__init__('%s: %s' % (code, detail))


def require(condition, code, detail):
    if not condition:
        raise ContractError(code, detail)


def array(value, shape, name):
    try:
        a = np.array(value, dtype=float, copy=True)
    except (TypeError, ValueError) as e:
        raise ContractError('INVALID_ARRAY', name) from e
    require(a.shape == shape, 'DIMENSION', '%s expected %s, got %s' % (name, shape, a.shape))
    require(np.all(np.isfinite(a)), 'NONFINITE', name)
    a.setflags(write=False)
    return a


def vector(value, name, minimum=1):
    a = np.asarray(value)
    require(a.ndim == 1 and len(a) >= minimum, 'DIMENSION', name)
    return array(a, a.shape, name)


def times(value):
    a = vector(value, 'time_s', 2)
    require(np.all(np.diff(a) > 0), 'TIME_NOT_INCREASING', 'actual timestamps must increase strictly')
    return a


def check_hash(value, name):
    require(isinstance(value, str) and re.fullmatch('[0-9a-f]{64}', value) is not None,
            'MISSING_METADATA', name + ' must be a SHA-256 hex digest')


class PhaseType(str, Enum):
    TRANSLATE = 'TRANSLATE'
    SPIN = 'SPIN'
    ARM_ONLY = 'ARM_ONLY'
    HOLD = 'HOLD'


class Status(str, Enum):
    PASS = 'PASS'
    FAIL = 'FAIL'
    UNKNOWN = 'UNKNOWN'


@dataclass(frozen=True)
class StateSchema:
    joint_names: Tuple[str, ...]
    position_units: Tuple[str, ...]
    frame_id: str
    reference_point: str
    time_unit: str
    velocity_units: Tuple[str, ...] = VELOCITY_UNITS
    acceleration_units: Tuple[str, ...] = ACCELERATION_UNITS
    jerk_units: Tuple[str, ...] = JERK_UNITS

    def __post_init__(self):
        require(tuple(self.joint_names) == NAMES, 'JOINT_ORDER', 'explicitly reorder to canonical nine coordinates')
        for key, expected in [('position_units', POSITION_UNITS), ('velocity_units', VELOCITY_UNITS),
                              ('acceleration_units', ACCELERATION_UNITS), ('jerk_units', JERK_UNITS)]:
            require(tuple(getattr(self, key)) == expected, 'UNITS', key)
        require(self.time_unit == 's', 'UNITS', 'time must be seconds')
        require(isinstance(self.frame_id, str) and bool(self.frame_id.strip()), 'MISSING_METADATA', 'frame_id')
        for name in ('joint_names', 'position_units', 'velocity_units', 'acceleration_units', 'jerk_units'):
            object.__setattr__(self, name, tuple(getattr(self, name)))
        require(self.reference_point == 'car_link', 'REFERENCE_POINT', 'requires axle-centred car_link')


@dataclass(frozen=True)
class PathRecord:
    trial_id: str
    planner_id: str
    parent_path_hash: str
    input_stage: str
    schema: StateSchema
    q: np.ndarray
    actual_time_s: Optional[np.ndarray]
    requested_start: np.ndarray
    requested_goal: np.ndarray
    metadata: Mapping[str, object]

    def __post_init__(self):
        require(isinstance(self.schema, StateSchema), 'MISSING_METADATA', 'explicit StateSchema required')
        require(bool(self.trial_id) and bool(self.planner_id), 'MISSING_METADATA', 'trial/planner')
        check_hash(self.parent_path_hash, 'parent_path_hash')
        require(self.input_stage in ('pre_TOTG', 'post_TOTG', 'synthetic'), 'INPUT_STAGE', self.input_stage)
        q = np.asarray(self.q)
        require(q.ndim == 2 and len(q) >= 2, 'DIMENSION', 'path needs at least two nine-state samples')
        object.__setattr__(self, 'q', array(q, (len(q), 9), 'path.q'))
        for name in ('requested_start', 'requested_goal'):
            object.__setattr__(self, name, array(getattr(self, name), (9,), name))
        if self.actual_time_s is not None:
            t = times(self.actual_time_s)
            require(len(t) == len(q), 'DIMENSION', 'path time/state count')
            object.__setattr__(self, 'actual_time_s', t)


@dataclass(frozen=True)
class PathJet:
    q: np.ndarray
    qs: np.ndarray
    qss: np.ndarray
    qsss: np.ndarray

    def __post_init__(self):
        q = np.asarray(self.q)
        require(q.ndim == 2 and len(q) >= 1, 'DIMENSION', 'jet requires nonempty (N,9) arrays')
        n = len(q)
        for name in ('q', 'qs', 'qss', 'qsss'):
            object.__setattr__(self, name, array(getattr(self, name), (n, 9), name))


@dataclass(frozen=True)
class PhasePath:
    phase_id: str
    phase_type: PhaseType
    schema: StateSchema
    start: np.ndarray
    end: np.ndarray
    evaluator: Callable[[np.ndarray], PathJet]
    provenance: str

    def __post_init__(self):
        require(isinstance(self.schema, StateSchema), 'MISSING_METADATA', 'explicit StateSchema required')
        require(isinstance(self.phase_type, PhaseType), 'PHASE_TYPE', 'use PhaseType')
        require(bool(self.phase_id) and bool(self.provenance), 'MISSING_METADATA', 'phase identity/source')
        for name in ('start', 'end'):
            object.__setattr__(self, name, array(getattr(self, name), (9,), name))

    def evaluate(self, s):
        s = vector(s, 'path parameter')
        require(np.all((s >= 0) & (s <= 1)), 'PARAMETER_RANGE', '[0,1] required')
        jet = self.evaluator(s)
        require(isinstance(jet, PathJet) and len(jet.q) == len(s), 'DIMENSION', 'evaluator must return a matching PathJet')
        fixed = {PhaseType.SPIN: [0, 1, 3, 4, 5, 6, 7, 8],
                 PhaseType.ARM_ONLY: [0, 1, 2], PhaseType.HOLD: list(range(9))}.get(self.phase_type, [])
        if fixed:
            require(np.allclose(jet.q[:, fixed], self.start[fixed], atol=1e-10, rtol=0), 'PHASE_SEMANTICS', 'fixed position changed')
            for a in (jet.qs, jet.qss, jet.qsss):
                require(np.max(np.abs(a[:, fixed])) <= 1e-10, 'PHASE_SEMANTICS', 'fixed coordinate derivative')
        else:
            require(np.all(np.linalg.norm(jet.qs[:, :2], axis=1) > 1e-10), 'DEGENERATE_TANGENT', 'TRANSLATE needs regular XY')
            lateral = -np.sin(jet.q[:, 2])*jet.qs[:, 0] + np.cos(jet.q[:, 2])*jet.qs[:, 1]
            forward = np.cos(jet.q[:, 2])*jet.qs[:, 0] + np.sin(jet.q[:, 2])*jet.qs[:, 1]
            require(np.max(np.abs(lateral)) < 1e-9 and np.all(forward > 0), 'HEADING_MISMATCH', 'tangent must face forward')
        for mask, endpoint in [(s == 0, self.start), (s == 1, self.end)]:
            if np.any(mask):
                require(np.allclose(jet.q[mask], endpoint, atol=1e-9, rtol=0), 'ENDPOINT', 'phase endpoint')
        return jet


@dataclass(frozen=True)
class Quantity:
    value: object
    unit: str
    kind: str
    source: str

    def __post_init__(self):
        require(self.kind in ('model', 'configuration', 'research_limit', 'synthetic', 'hardware_verified'), 'SOURCE_KIND', self.kind)
        require(bool(self.source), 'MISSING_METADATA', 'quantity source')
        a = np.asarray(self.value)
        object.__setattr__(self, 'value', array(a, a.shape, 'quantity'))


@dataclass(frozen=True)
class LimitSet:
    arm_lower: Quantity
    arm_upper: Quantity
    arm_velocity: Quantity
    arm_acceleration: Quantity
    xy_component_velocity: Quantity
    xy_component_acceleration: Quantity
    xy_speed: Quantity
    xy_acceleration: Quantity
    tangential_acceleration: Quantity
    angular_velocity: Quantity
    angular_acceleration: Quantity
    wheel_radius: Quantity
    wheel_separation: Quantity
    wheel_velocity: Quantity
    wheel_acceleration: Optional[Quantity]
    spin_jerk: Optional[Quantity]
    reference_point: str
    metadata: Mapping[str, object]

    def __post_init__(self):
        fields = {
            'arm_lower': ('rad', (6,)), 'arm_upper': ('rad', (6,)),
            'arm_velocity': ('rad/s', (6,)), 'arm_acceleration': ('rad/s^2', (6,)),
            'xy_component_velocity': ('m/s', (2,)), 'xy_component_acceleration': ('m/s^2', (2,)),
            'xy_speed': ('m/s', ()), 'xy_acceleration': ('m/s^2', ()),
            'tangential_acceleration': ('m/s^2', ()), 'angular_velocity': ('rad/s', ()),
            'angular_acceleration': ('rad/s^2', ()), 'wheel_radius': ('m', ()),
            'wheel_separation': ('m', ()), 'wheel_velocity': ('rad/s', (2,)),
            'wheel_acceleration': ('rad/s^2', (2,)), 'spin_jerk': ('rad/s^3', ())}
        for name, (unit, shape) in fields.items():
            item = getattr(self, name)
            if name in ('wheel_acceleration', 'spin_jerk') and item is None:
                continue
            require(isinstance(item, Quantity), 'MISSING_LIMIT', name)
            require(item.unit == unit, 'UNITS', name + ' must use ' + unit)
            require(item.value.shape == shape, 'DIMENSION', name)
            if name not in ('arm_lower', 'arm_upper'):
                require(np.all(item.value > 0), 'INVALID_LIMIT', name + ' must be positive')
        require(np.all(self.arm_lower.value < self.arm_upper.value), 'INVALID_LIMIT', 'position interval')
        require(self.reference_point == 'car_link', 'REFERENCE_POINT', 'limits use axle-centred car_link')


@dataclass(frozen=True)
class TrajectoryRecord:
    """One phase, with global actual time. Keep one-sided endpoint jets at seams."""
    trial_id: str
    phase_id: str
    phase_type: PhaseType
    planner_id: str
    parent_path_hash: str
    schema: StateSchema
    time_s: np.ndarray
    q: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    jerk: np.ndarray
    derivative_source: str

    def __post_init__(self):
        require(isinstance(self.schema, StateSchema), 'MISSING_METADATA', 'explicit StateSchema required')
        require(bool(self.trial_id) and bool(self.phase_id) and bool(self.planner_id), 'MISSING_METADATA', 'trajectory IDs')
        check_hash(self.parent_path_hash, 'parent_path_hash')
        require(isinstance(self.phase_type, PhaseType), 'PHASE_TYPE', 'use enum')
        require(bool(self.derivative_source), 'MISSING_METADATA', 'derivative_source')
        t = times(self.time_s)
        object.__setattr__(self, 'time_s', t)
        for name in ('q', 'velocity', 'acceleration', 'jerk'):
            object.__setattr__(self, name, array(getattr(self, name), (len(t), 9), name))


@dataclass(frozen=True)
class TrialManifest:
    trial_id: str
    planner_id: str
    parent_path_hash: str
    schema: StateSchema
    input_stage: str
    model_hash: str
    scene_hash: Optional[str]
    seed: Optional[int]
    seed_status: str
    parameter_sources: Mapping[str, str]
    code_hashes: Mapping[str, str]
    failure_codes: Tuple[str, ...]
    missing_metadata: Tuple[str, ...]

    def __post_init__(self):
        require(isinstance(self.schema, StateSchema), 'MISSING_METADATA', 'explicit StateSchema required')
        require(bool(self.trial_id) and bool(self.planner_id), 'MISSING_METADATA', 'manifest identity')
        check_hash(self.parent_path_hash, 'parent_path_hash')
        check_hash(self.model_hash, 'model_hash')
        if self.scene_hash is not None:
            check_hash(self.scene_hash, 'scene_hash')
        require(self.input_stage in ('pre_TOTG', 'post_TOTG', 'synthetic'), 'INPUT_STAGE', self.input_stage)
        require(self.seed_status in ('controlled', 'uncontrolled', 'unknown'), 'SEED_STATUS', self.seed_status)
        require(self.seed_status != 'controlled' or isinstance(self.seed, int), 'MISSING_METADATA', 'controlled seed')
        require(bool(self.parameter_sources) and bool(self.code_hashes), 'MISSING_METADATA', 'parameter/code sources')
        for name, digest in self.code_hashes.items():
            check_hash(digest, name)
        if self.scene_hash is None:
            require('scene' in self.missing_metadata, 'MISSING_METADATA', 'explicitly list missing scene')


@dataclass
class ValidationResult:
    numerical_status: Status
    collision_status: Status
    failure_codes: list = field(default_factory=list)
    checks: dict = field(default_factory=dict)
    metrics: dict = field(default_factory=dict)
    coverage: str = 'sampled_reference_only'

    @property
    def overall_status(self):
        if Status.FAIL in (self.numerical_status, self.collision_status):
            return Status.FAIL
        if Status.UNKNOWN in (self.numerical_status, self.collision_status):
            return Status.UNKNOWN
        return Status.PASS


def merge_aligned_streams(base_time, base_q, arm_time, arm_q, arm_names):
    """Strict join only. Resampling/TF transforms belong to a future explicit adapter."""
    bt, at = times(base_time), times(arm_time)
    require(np.array_equal(bt, at), 'TIME_GRID_MISMATCH', 'never zip different joint/multiDOF grids')
    require(tuple(arm_names) == NAMES[3:], 'JOINT_ORDER', 'explicit arm reordering required')
    b = array(base_q, (len(bt), 3), 'base_q')
    a = array(arm_q, (len(at), 6), 'arm_q')
    return np.column_stack((b, a))
