"""M4 reference admission; no motion/controller implementation."""
from copy import deepcopy
import math
from contracts.motion import CartesianReference, MotionSessionSpec
from contracts.sessions import payload_sha256


def session_spec(config, robot_id):
    """Bind the effective complete configuration, including limits and stopping."""
    return MotionSessionSpec(robot_id, config['controller'].get('kind', 'cartesian_impedance'),
                             payload_sha256(config), 'fr3_link0', 'fr3_flange',
                             config['controller']['projector'],
                             config['task']['kind'] if config['task']['kind'] in ('z_line','yz_circle') else 'none')


def payload_validator(config, *, manipulator='arm'):
    c = deepcopy(config)
    def validate(payload):
        if not isinstance(payload, dict):
            raise ValueError('M4 reference must be an object')
        allowed = {'manipulator','base_frame','tcp_frame','position_m','quaternion_xyzw',
                   'twist_linear_angular','schema_version','posture_rad'}
        if set(payload)-allowed:
            raise ValueError('Unknown M4 reference fields')
        ref = CartesianReference.from_dict({k:v for k,v in payload.items() if k != 'posture_rad'})
        if (ref.manipulator,ref.base_frame,ref.tcp_frame) != (manipulator,'fr3_link0','fr3_flange'):
            raise ValueError('M4 manipulator/base/TCP mismatch')
        for v,lo,hi in zip(ref.position_m,c['limits']['workspace_lower'],c['limits']['workspace_upper']):
            if not lo <= v <= hi:
                raise ValueError('M4 reference outside workspace')
        if math.sqrt(sum(v*v for v in ref.twist_linear_angular[:3])) > c['limits']['linear_speed_m_s']:
            raise ValueError('M4 linear reference speed exceeds guard')
        if math.sqrt(sum(v*v for v in ref.twist_linear_angular[3:])) > c['limits']['angular_speed_rad_s']:
            raise ValueError('M4 angular reference speed exceeds guard')
        if 'posture_rad' in payload:
            if c['task']['kind'] != 'posture':
                raise ValueError('Posture updates require the configured posture task')
            from .reference_types import finite_vector
            q=finite_vector(payload['posture_rad'],7,'posture_rad')
            if any(not lo<=v<=hi for v,lo,hi in zip(q,c['limits']['joint_lower'],c['limits']['joint_upper'])):
                raise ValueError('Posture target outside joint bounds')
        return payload
    return validate
