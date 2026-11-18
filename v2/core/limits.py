"""Read-only static model extraction. Research limits must be supplied explicitly."""
from pathlib import Path
from hashlib import sha256
import json
import xml.etree.ElementTree as ET
import numpy as np
import yaml
from .contracts import Quantity, LimitSet, require, ContractError, NAMES


def _load_limits(urdf_path, yaml_path, srdf_path, research, *, velocity_scaling, acceleration_scaling):
    for name, scale in [('velocity', velocity_scaling), ('acceleration', acceleration_scaling)]:
        require(np.isfinite(scale) and 0 < scale <= 1, 'SCALING', name)
    paths = [Path(p) for p in (urdf_path, yaml_path, srdf_path)]
    require(all(p.is_file() for p in paths), 'MISSING_MODEL', 'URDF/YAML/SRDF path')
    root = ET.parse(str(paths[0])).getroot()
    srdf = ET.parse(str(paths[2])).getroot()
    with paths[1].open() as stream:
        config = yaml.safe_load(stream)['joint_limits']
    joints = {j.attrib['name']: j for j in root.findall('joint')}
    plugins = [p for p in root.findall('.//plugin') if p.attrib.get('filename') == 'libgazebo_ros_diff_drive.so']
    require(len(plugins) == 1, 'MODEL_SCHEMA', 'need one differential drive plugin')
    plugin = plugins[0]
    radius = float(plugin.findtext('wheelDiameter'))/2
    separation = float(plugin.findtext('wheelSeparation'))
    require(np.isclose(radius, .06) and np.isclose(separation, .3935), 'MODEL_CHANGED', 'review wheel parameters')
    require(plugin.findtext('robotBaseFrame') == 'car_link', 'REFERENCE_POINT', 'plugin base frame')
    virtual = srdf.find("virtual_joint[@name='world_car']")
    require(virtual is not None and virtual.attrib.get('child_link') == 'car_link'
            and virtual.attrib.get('type') == 'planar', 'REFERENCE_POINT', 'SRDF world_car')
    origins = []
    for tag in ('leftJoint', 'rightJoint'):
        joint = joints[plugin.findtext(tag)]
        require(joint.find('parent').attrib['link'] == 'car_link', 'REFERENCE_POINT', 'wheel parent')
        origin = joint.find('origin')
        xyz = np.fromstring(origin.attrib['xyz'], sep=' ')
        rpy = np.fromstring(origin.attrib['rpy'], sep=' ')
        axis = np.fromstring(joint.find('axis').attrib['xyz'], sep=' ')
        require(xyz.shape == (3,) and np.all(np.isfinite(xyz)) and np.allclose(rpy, 0)
                and np.allclose(axis, [0, 1, 0]), 'REFERENCE_POINT', 'wheel origin/axis')
        origins.append(xyz)
    centre = (origins[0]+origins[1])/2
    require(np.allclose(centre[:2], 0) and np.isclose(origins[0][0], origins[1][0])
            and np.isclose(origins[0][2], origins[1][2]), 'REFERENCE_POINT', 'car_link XY must equal axle centre')
    geometric_separation = float(abs(origins[0][1]-origins[1][1]))
    provenance = lambda p, where: str(p.resolve()) + '#' + where
    lower, upper, arm_v, arm_a = [], [], [], []

    def configured(name, kind):
        require(name in config and config[name].get('has_'+kind+'_limits') is True,
                'MISSING_LIMIT', name+'/'+kind)
        value = config[name].get('max_'+kind)
        require(value is not None and np.isfinite(value) and value > 0, 'MISSING_LIMIT', name+'/'+kind)
        return float(value)

    for name in NAMES[3:]:
        require(name in joints and joints[name].attrib['type'] == 'revolute', 'MODEL_SCHEMA', name)
        bound = joints[name].find('limit')
        require(bound is not None, 'MISSING_LIMIT', name)
        lower.append(float(bound.attrib['lower'])); upper.append(float(bound.attrib['upper']))
        arm_v.append(min(float(bound.attrib['velocity']), configured(name, 'velocity'))*velocity_scaling)
        arm_a.append(configured(name, 'acceleration')*acceleration_scaling)
    model = lambda v, unit, location: Quantity(v, unit, 'model', provenance(paths[0], location))
    cfg = lambda v, unit, location: Quantity(v, unit, 'configuration', provenance(paths[1], location))
    needed = ('xy_speed', 'xy_acceleration', 'tangential_acceleration', 'wheel_velocity')
    for key in needed:
        require(key in research, 'MISSING_LIMIT', 'explicit research parameter '+key)
    for key, quantity in research.items():
        require(isinstance(quantity, Quantity) and quantity.kind in ('research_limit', 'synthetic'),
                'SOURCE_KIND', key+' is not a declared research/synthetic limit')
    hashes = {str(p.resolve()): sha256(p.read_bytes()).hexdigest() for p in paths}
    return LimitSet(
        arm_lower=model(lower, 'rad', 'joint1..6/limit/lower'),
        arm_upper=model(upper, 'rad', 'joint1..6/limit/upper'),
        arm_velocity=cfg(arm_v, 'rad/s', 'joint1..6/max_velocity; min(URDF,YAML)*request_scale'),
        arm_acceleration=cfg(arm_a, 'rad/s^2', 'joint1..6/max_acceleration*request_scale'),
        xy_component_velocity=cfg([configured(k, 'velocity')*velocity_scaling for k in NAMES[:2]], 'm/s', 'world_car/x,y'),
        xy_component_acceleration=cfg([configured(k, 'acceleration')*acceleration_scaling for k in NAMES[:2]], 'm/s^2', 'world_car/x,y'),
        xy_speed=research['xy_speed'], xy_acceleration=research['xy_acceleration'],
        tangential_acceleration=research['tangential_acceleration'],
        angular_velocity=cfg(configured(NAMES[2], 'velocity')*velocity_scaling, 'rad/s', 'world_car/theta'),
        angular_acceleration=cfg(configured(NAMES[2], 'acceleration')*acceleration_scaling, 'rad/s^2', 'world_car/theta'),
        wheel_radius=model(radius, 'm', 'differential_drive_controller/wheelDiameter/2'),
        wheel_separation=model(separation, 'm', 'differential_drive_controller/wheelSeparation'),
        wheel_velocity=research['wheel_velocity'], wheel_acceleration=research.get('wheel_acceleration'),
        spin_jerk=research.get('spin_jerk'), reference_point='car_link',
        metadata=dict(input_hashes=hashes, geometric_wheel_separation_m=geometric_separation,
                      wheel_separation_conflict=not np.isclose(geometric_separation, separation),
                      axle_centre_xy_m=centre[:2].tolist(), velocity_scaling=velocity_scaling,
                      acceleration_scaling=acceleration_scaling,
                      scope='STATIC_CONFIG_NOT_RUNTIME_OR_HARDWARE',
                      note='YAML jerk declarations do not prove TOTG jerk enforcement; wheelAcceleration=0 is not a physical bound.'))


OPTIONAL_CONSTRAINTS = ('xy_speed', 'xy_acceleration', 'tangential_acceleration',
                        'wheel_velocity', 'wheel_acceleration', 'spin_jerk')


def constraint_enabled(limits, name):
    """Absent profile means the original contract, including optional None rules."""
    return limits.metadata.get('constraint_profile', {}).get('enabled', {}).get(name, True)


def load_limits(urdf_path, yaml_path, srdf_path, research, *, velocity_scaling, acceleration_scaling, backend_overrides=None, profile=None):
    """Public reader with coded failures for missing/malformed model fields."""
    try:
        limits = _load_limits(urdf_path, yaml_path, srdf_path, research,
                            velocity_scaling=velocity_scaling, acceleration_scaling=acceleration_scaling)
        for key, record in (backend_overrides or {}).items():
            require(key in ('arm_velocity','arm_acceleration','xy_component_velocity','xy_component_acceleration','angular_velocity','angular_acceleration'),'OVERRIDE_KEY',key)
            q=Quantity(**record);old=getattr(limits,key)
            require(q.kind=='research_limit' and q.unit==old.unit,'OVERRIDE_UNIT_SOURCE',key)
            values=np.asarray(q.value,float);bound=np.asarray(old.value,float)
            require(values.shape==bound.shape and np.all(np.isfinite(values)) and np.all(values>0) and np.all(values<=bound),'OVERRIDE_EXCEEDS_MODEL_OR_SHAPE',key)
            from dataclasses import replace
            limits=replace(limits,**{key:q})
        if profile is not None:
            from dataclasses import replace
            require(isinstance(profile, dict) and profile.get('identity') == 'V2_NINE_DOF_VA_ONLY',
                    'PROFILE_IDENTITY', 'explicit supported profile required')
            enabled = profile.get('enabled')
            require(isinstance(enabled, dict) and set(enabled) == set(OPTIONAL_CONSTRAINTS)
                    and all(type(v) is bool for v in enabled.values()),
                    'PROFILE_CONSTRAINTS', 'all six switches must be explicit booleans')
            velocity = np.asarray(profile.get('velocity'), dtype=float)
            acceleration = np.asarray(profile.get('acceleration'), dtype=float)
            require(velocity.shape == (9,) and acceleration.shape == (9,)
                    and np.all(np.isfinite(velocity)) and np.all(velocity > 0)
                    and np.all(np.isfinite(acceleration)) and np.all(acceleration > 0),
                    'PROFILE_NINE_DOF', 'nine positive finite SI velocity and acceleration limits required')
            source = 'user-authorized research profile; not MoveIt or hardware capability'
            def quantity(value, unit): return Quantity(value, unit, 'research_limit', source)
            metadata = dict(limits.metadata)
            metadata['constraint_profile'] = dict(identity=profile['identity'], enabled=dict(enabled),
                velocity=velocity.tolist(), acceleration=acceleration.tolist(),
                note='explicit backend research limits; may exceed MoveIt acceleration configuration; not hardware verified')
            optional_values = {key: dict(value=getattr(limits,key).value.tolist(),unit=getattr(limits,key).unit)
                               for key in OPTIONAL_CONSTRAINTS if enabled[key] and getattr(limits,key) is not None}
            metadata['constraint_profile']['effective_limit_identity'] = sha256(json.dumps(
                dict(enabled=enabled,velocity=velocity.tolist(),acceleration=acceleration.tolist(),
                     optional_values=optional_values),
                sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
            limits = replace(limits, xy_component_velocity=quantity(velocity[:2], 'm/s'),
                angular_velocity=quantity(velocity[2], 'rad/s'), arm_velocity=quantity(velocity[3:], 'rad/s'),
                xy_component_acceleration=quantity(acceleration[:2], 'm/s^2'),
                angular_acceleration=quantity(acceleration[2], 'rad/s^2'),
                arm_acceleration=quantity(acceleration[3:], 'rad/s^2'), metadata=metadata)
        return limits
    except ContractError:
        raise
    except (KeyError, TypeError, ValueError, AttributeError, OSError, ET.ParseError, yaml.YAMLError) as exc:
        raise ContractError('MODEL_SCHEMA', 'missing or malformed model/config field: '+str(exc)) from exc
