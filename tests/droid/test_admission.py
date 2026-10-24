"""Real physical composition with fake devices only; no native SDK import."""
import json
from pathlib import Path
import runpy
from types import SimpleNamespace as NS
import pytest
from contracts.sessions import payload_sha256
from franka_runtime.droid.admission import PhysicalAdmission,service_settings
from franka_runtime.droid.physical_config import PhysicalDroidConfig,pending_template
from franka_runtime.droid.physical_factory import PhysicalDeviceFactory
from franka_runtime.droid.providers import RecordedIdentityReader,BoundCollisionChecker
from franka_runtime.droid.recording import inspect_recording

# Reuse synthetic commissioning and fake native transport fixtures.
FIX=runpy.run_path(str(Path(__file__).with_name('test_physical_config.py')))
NATIVE=runpy.run_path(str(Path(__file__).with_name('test_native_backend.py')))
SPEC=FIX['SPEC']


class Factory(PhysicalDeviceFactory):
    calls=0
    def connect(self,executor,*,spec,authorize_device_setup=False,recorder=None):
        self.calls+=1
        assert authorize_device_setup and recorder.fault is None
        q=(0.,)*7;driver=NATIVE['Driver'](self.clock,q);hand=NATIVE['Hand']()
        from franka_runtime.droid.native import NativeDroidBackend
        backend=NativeDroidBackend(executor,driver,hand,clock=self.clock,startup_timeout_ns=1_000_000_000,
            consumption_timeout_ns=50_000_000,stop_timeout_ns=3_000_000_000,state_timeout_ns=50_000_000)
        backend.configuration_sha256=self.config.sha256
        self.backend=backend
        return backend


def fixture(tmp_path):
    d=FIX['complete'](tmp_path);d['timing'].update(command_timeout_ms=2000,heartbeat_timeout_ms=1000,hold_timeout_ms=100,max_lease_s=30)
    d['contract']['capability_prefix']='droid-jointpos-v1'
    # Match the actual shared wire capability; not a test-only config override.
    from contracts.droid_session import capability_id
    d['contract']['capability_prefix']=capability_id(SPEC).split(':')[0]
    c=PhysicalDroidConfig(d);clock=NATIVE['Clock']()
    checker=NS(artifact_sha256=d['artifacts']['collision_model_sha256'],check_configuration=lambda q:None)
    factory=Factory(c,clock=clock,clock_id='test:monotonic',identity_reader=lambda a:{},collision_checker=checker)
    return PhysicalAdmission(c,SPEC,factory,clock=clock,clock_id='test:monotonic'),factory,clock


def test_inspection_never_connects_or_starts_recording(tmp_path):
    a,f,c=fixture(tmp_path);report=a.inspect()
    assert not report['hardware_accessed'] and f.calls==0
    assert not Path(a.config.data['recording']['directory']).exists()
    settings=service_settings(a.config)
    assert 'mock_start_q' not in settings and 'max_acceleration' not in settings
    assert settings['max_velocity']==tuple(a.config.data['acceptance']['max_consecutive_row_speed_rad_s'])


@pytest.mark.parametrize('setup,motion',[(False,False),(True,False),(False,True)])
def test_separate_authorizations_are_required(tmp_path,setup,motion):
    a,f,c=fixture(tmp_path)
    with pytest.raises(ValueError,match='authorization'):a.prepare(authorize_device_setup=setup,authorize_motion=motion,token='secret')
    assert f.calls==0 and not Path(a.config.data['recording']['directory']).exists()


def test_pending_and_config_binding_fail_before_devices(tmp_path):
    a,f,c=fixture(tmp_path);a.config.data['workspace']['joint_names'].reverse()
    with pytest.raises(ValueError,match='mismatch'):a.prepare(authorize_device_setup=True,authorize_motion=True,token='secret')
    assert f.calls==0
    with pytest.raises(ValueError,match='Uncommissioned'):service_settings(PhysicalDroidConfig(pending_template()))


def test_complete_physical_composition_cancels_without_fabricating_native_stop(tmp_path):
    a,f,c=fixture(tmp_path)
    prepared=a.prepare(authorize_device_setup=True,authorize_motion=True,token='do-not-record-token')
    svc=prepared.service
    assert svc.backend.hardware_executed and svc.listener is None and not svc.backend._started
    from contracts.sessions import LeaseRequest
    lease=svc.authority.acquire(LeaseRequest('r',SPEC.robot.robot_id,'test',svc.clock_id,c(),1_000_000_000),episode_id='test')
    svc.shutdown();report=prepared.close(timeout_s=.1)
    assert report['ok'] and not report['native_control_started']
    assert report['stop_evidence'] is None and report['cancellation_evidence']['control_never_started']
    assert not svc.backend.driver.motion.sent and not svc.backend.driver.motion.stop
    with pytest.raises(ValueError,match='single-session'):svc.backend.check_session_admission()
    audit=inspect_recording(Path(a.config.data['recording']['directory']))
    assert audit['sealed'] and audit['terminal']['cancellation_evidence']['workers_never_started']
    assert 'do-not-record-token' not in (Path(a.config.data['recording']['directory'])/'events.jsonl').read_text()


def test_factory_failure_is_recorded_without_control(tmp_path):
    a,f,c=fixture(tmp_path)
    def fail(*a,**kw):raise RuntimeError('fake_constructor_failure')
    f.connect=fail
    with pytest.raises(RuntimeError,match='constructor_failure'):
        a.prepare(authorize_device_setup=True,authorize_motion=True,token='secret')
    audit=inspect_recording(Path(a.config.data['recording']['directory']))
    assert audit['sealed'] and audit['terminal']['admission_failed'] and not audit['terminal']['control_started']


def test_recorded_identity_is_hash_bound_and_not_restamped(tmp_path):
    from franka_runtime.droid.physical_config import digest
    path=tmp_path/'identity.json'
    v=dict(schema_version=1,source='synthetic',address='test.invalid',model='FR3',firmware_version='5.9.2',
        fci_version=10,arm_serial='test',control_serial='test',clock_id='test:monotonic',observed_ns=123)
    path.write_text(json.dumps(v));reader=RecordedIdentityReader(path,sha256=digest(path))
    assert reader('test.invalid')['observed_ns']==123
    with pytest.raises(ValueError,match='address'):reader('wrong')
    path.write_text('{}')
    with pytest.raises(ValueError,match='hash'):reader('test.invalid')


def test_collision_engine_requires_bound_model_tool_and_explicit_clearance(tmp_path):
    c=PhysicalDroidConfig(FIX['complete'](tmp_path));d=c.data
    engine=NS(model_sha256=d['artifacts']['collision_model_sha256'],urdf_sha256=d['workspace']['model_sha256'],
        joint_names=tuple(SPEC.robot.joint_names),tool_sha256=payload_sha256(d['tool']),provenance='synthetic test engine',
        check_configuration=lambda q:True)
    checker=BoundCollisionChecker(c,engine);checker.check_configuration((0.,)*7)
    engine.check_configuration=lambda q:None
    with pytest.raises(ValueError,match='clearance'):checker.check_configuration((0.,)*7)
    engine.tool_sha256='wrong'
    with pytest.raises(ValueError,match='tool_sha256'):BoundCollisionChecker(c,engine)


def test_admitted_physical_service_runs_the_shared_protocol_with_fake_native_receipts(tmp_path):
    a,f,c=fixture(tmp_path)
    prepared=a.prepare(authorize_device_setup=True,authorize_motion=True,token='test')
    svc=prepared.service;b=f.backend
    estimate=NATIVE['start'](svc,b,c);NATIVE['submit'](svc,b,c,estimate)
    c.advance();svc.control_step(c(),.001)
    c.advance();b.driver.motion.callback();svc.control_step(c(),.001)
    receipts=svc.authority.drain_receipts()
    assert [r.status for r in receipts]==['accepted','applied']
    svc.shutdown();c.advance();b.driver.motion.callback(finished=True,rest=True,consume=False);b.driver.joined=True
    report=prepared.close(timeout_s=.1)
    assert report['ok'] and report['native_control_started'] and report['stop_evidence']['stop_at_rest']
    audit=inspect_recording(Path(a.config.data['recording']['directory']))
    assert audit['kinds']['receipt']==2 and audit['terminal']['stop_confirmed']
