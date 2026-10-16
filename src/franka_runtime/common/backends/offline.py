"""Deterministic offline backends. Neither emulates physical arm dynamics."""
from copy import deepcopy
from types import SimpleNamespace
from pathlib import Path
import numpy as np
from .feedback import Snapshot
from .franky_m4 import FrankyMotionBackend
from ..motion.kinematics import UrdfKinematicsProvider
from ..motion.reference_types import finite_vector


class MockMotionBackend:
    """Static encoder state, logical reference consumption, explicit mock stop."""
    hardware_executed = False
    receipt_source = 'mock_logical_consumption'

    def __init__(self, config, initial_state):
        self.config=deepcopy(config)
        self.initial_state=deepcopy(initial_state)
        self.motion=None
        self.sequence=0
        self.events=[]
        self.active=False
        self._queued=None
        self._tick=0

    def preflight(self):
        state=self.initial_state
        if set(state) != {'schema_version','q','dq'} or state['schema_version'] != 1:
            raise ValueError('Mock state requires schema_version, q and dq only')
        q=finite_vector(state['q'],7,'q');dq=finite_vector(state['dq'],7,'dq')
        c=self.config
        if any(not lo<=v<=hi for v,lo,hi in zip(q,c['limits']['joint_lower'],c['limits']['joint_upper'])):
            raise ValueError('Mock start joints outside bounds')
        if max(abs(v) for v in dq)>c['robot']['max_start_velocity_rad_s']:
            raise ValueError('Mock start is not at rest')
        model=UrdfKinematicsProvider(Path(c['bundle'])/'model/robot_returned_model.urdf')
        p,quat,j=model.evaluate(q)
        if np.any(p<c['limits']['workspace_lower']) or np.any(p>c['limits']['workspace_upper']):
            raise ValueError('Mock start pose outside workspace')
        if np.linalg.svd(j,compute_uv=False).min()<c['limits']['minimum_jacobian_singular_value']:
            raise ValueError('Mock start is singular')
        self.snapshot=Snapshot(0,0.,q,dq,q,(0.,)*7,(0.,)*7,tuple(p),tuple(quat),
             'mock_joint_state_urdf_fk','fr3_link0','fr3_flange',tuple(p),0.,0.,0,
             tuple(map(tuple,np.eye(4))),0.,'Idle',False,'','',False,1.)
        return self.snapshot

    def plan(self, snapshot):
        return FrankyMotionBackend.plan(self,snapshot)

    def posture_target(self, elapsed, snapshot):
        return FrankyMotionBackend.posture_target(self,elapsed,snapshot)

    def start(self, snapshot):
        if self.active:raise RuntimeError('Mock session already active')
        self.active=True;self._queued=None
        self.motion=SimpleNamespace(fault=0,dropped_samples=0)
        self.events.append('start')

    def send_reference(self, payload):
        if not self.active:raise RuntimeError('Mock motion is not active')
        self.sequence+=1
        self._queued=(self.sequence,deepcopy(payload))
        return self.sequence

    def consume(self, now_ns):
        if self._queued is None or not self.active:return None
        sequence,payload=self._queued;self._queued=None;self._tick+=1
        return {'native_sequence':sequence,'native_tick':self._tick,'timestamp_ns':now_ns}

    def request_stop(self, reason='operator_stop'):
        self.events.append(('request_stop',reason));self.active=False;self._queued=None

    def stop_evidence(self):
        return {'joined':not self.active,'stop_finished':not self.active,'stop_at_rest':True,
                'stop_fault':0,'dropped_samples':0,'evidence_source':'mock_only_not_physical'}


class OfflineShadowBackend(MockMotionBackend):
    """Evaluate references against recorded/static state; no application receipts."""
    receipt_source = 'offline_shadow_no_application'

    def consume(self, now_ns):
        self._queued=None
        return None
