"""Bind local authority to the effective immutable M4 backend configuration."""
from .authority import SessionAuthority


def create_m4_session(backend, *, robot_id, clock_id, clock=None, expect_application=True):
    from ..common.motion.payload import session_spec, payload_validator
    config=backend.config
    return SessionAuthority(backend,session_spec(config,robot_id),payload_validator(config),
        clock_id=clock_id,command_timeout_ns=int(config['limits']['reference_timeout_s']*1e9),
        clock=clock,expect_application=expect_application)
