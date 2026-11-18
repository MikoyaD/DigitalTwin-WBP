from pathlib import Path
import numpy as np
from v2.core.contracts import (NAMES, POSITION_UNITS, StateSchema, Quantity,
                               TrajectoryRecord, PhaseType, PathJet)
from v2.core.limits import load_limits
from v2.core.math_tools import quintic_time_jet, chain_rule, spin_jet, arm_only_jet, hold_jet, translation_jet

PACKAGE = Path(__file__).resolve().parents[2]
SCHEMA = StateSchema(NAMES, POSITION_UNITS, 'world', 'car_link', 's')
HASH = 'a'*64


def research_limits():
    make = lambda value, unit: Quantity(value, unit, 'research_limit', 'phase-I explicit test/design setting; not hardware capability')
    return dict(xy_speed=make(.5, 'm/s'), xy_acceleration=make(.5, 'm/s^2'),
                tangential_acceleration=make(.5, 'm/s^2'),
                wheel_velocity=make([.5/.06]*2, 'rad/s'),
                wheel_acceleration=make([.5/.06]*2, 'rad/s^2'), spin_jerk=make(2., 'rad/s^3'))


def limits(**kwargs):
    return load_limits(PACKAGE/'config/gazebo_combined_robot.urdf', PACKAGE/'config/joint_limits.yaml',
                       PACKAGE/'config/combined_robot.srdf', kwargs.get('research', research_limits()),
                       velocity_scaling=kwargs.get('vs', 1.), acceleration_scaling=kwargs.get('acs', 1.))


def line_jet(s):
    q = np.zeros((len(s), 9)); q[:, 0] = s
    qs = np.zeros_like(q); qs[:, 0] = 1
    return PathJet(q, qs, np.zeros_like(q), np.zeros_like(q))


def circle_jet(s):
    z = []
    angle = .4 + 1.2*s
    for k in range(5):
        data = np.zeros((len(s), 8))
        data[:, 0] = 2*1.2**k*np.cos(angle+k*np.pi/2)
        data[:, 1] = 2*1.2**k*np.sin(angle+k*np.pi/2)
        z.append(data)
    return translation_jet(s, z, .4+np.pi/2)


def record(kind=PhaseType.TRANSLATE, n=501, duration=10., offset=0., jet_function=None):
    t = np.linspace(0, duration, n)
    s, v, a, j = quintic_time_jet(t, duration)
    if jet_function is not None:
        jet = jet_function(s)
    elif kind == PhaseType.TRANSLATE:
        jet = line_jet(s)
    elif kind == PhaseType.SPIN:
        jet = spin_jet(s, np.zeros(9), 1.)
    elif kind == PhaseType.ARM_ONLY:
        arm = np.tile(s[:, None], (1, 6))*.2
        jet = arm_only_jet([0, 0, 0], [arm, np.ones_like(arm)*.2, np.zeros_like(arm), np.zeros_like(arm)])
    else:
        jet = hold_jet(np.zeros(9), n)
    derivatives = chain_rule(jet, v, a, j)
    return TrajectoryRecord('synthetic-trial', kind.value, kind, 'synthetic-analytic', HASH,
                            SCHEMA, t+offset, jet.q, *derivatives, 'analytic chain rule')
