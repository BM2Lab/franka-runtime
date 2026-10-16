import json
from pathlib import Path
from types import SimpleNamespace
import pytest
np=pytest.importorskip("numpy")
pytest.importorskip("yaml")
from franka_runtime.common.motion import legacy
from franka_runtime.common.motion.impedance import JointImpedanceController, TorqueCommandProcessor, dynamic_torque_projector
from franka_runtime.common.motion.kinematics import UrdfKinematicsProvider, nullspace_projector
from franka_runtime.common.motion.virtual_fixture import CurveVirtualFixture
from franka_runtime.common.backends.feedback import EncoderForwardKinematics, _encoder_pose_feedback
from franka_runtime.common.backends.franky_m4 import command_success_threshold, FrankyMotionBackend, make_curve_fixture

ROOT=Path(__file__).parents[2]
FIXTURES=ROOT/'tests/parity/fixtures'
D=json.loads((FIXTURES/'legacy-parity-v1.json').read_text())


def close(actual,expected):np.testing.assert_allclose(actual,expected,rtol=0,atol=D['tolerance']['atol'])


def test_extracted_reference_history_and_joint_pd_against_golden():
    r=D['reference']
    close(legacy.action_to_target(r['position_mapping'],r['action'],r['prior']),r['position_target'])
    close(legacy.action_to_velocity(r['velocity_mapping'],r['action']),r['velocity_target'])
    close(legacy.assemble_policy_observation(**D['observation']['inputs']),D['observation']['values'])
    c=D['joint_controller'];controller=JointImpedanceController(c['stiffness'],c['damping'])
    actual=controller.compute(**c['inputs'])
    for key,value in c['output'].items():close(actual[key],value)


def test_extracted_torque_order_and_selective_reset_against_golden():
    a,b=TorqueCommandProcessor(),TorqueCommandProcessor()
    for row in D['torque_processing']:
        result=a.compute(row['requested'],row['gravity'])
        for key,value in row['output'].items():close(result[key],value)
    b.compute([100]*7,[0]*7);a.reset()
    reset=a.compute([100]*7,[0]*7);untouched=b.compute([100]*7,[0]*7)
    for label,result in [('reset_instance',reset),('untouched_instance',untouched)]:
        for key,value in D['selective_reset'][label].items():close(result[key],value)


def test_projectors_curve_and_urdf_fk_against_golden():
    p=D['projectors'];close(nullspace_projector(p['jacobian']),p['kinematic'])
    close(dynamic_torque_projector(p['jacobian'],p['mass']),p['dynamic'])
    for c in D['curve_fixtures']:
        f=CurveVirtualFixture(c['kind'],c['origin'],c['radius'])
        projected,tangent=f.project(c['position']);close(projected,c['projected']);close(tangent,c['tangent'])
        close(f.force(c['position'],c['velocity'],*c['gains']),c['force'])
    f=D['urdf_fk'];model=UrdfKinematicsProvider(FIXTURES/'robot_returned_model.urdf')
    p,q,j=model.evaluate(f['q']);close(p,f['position']);close(q,f['quaternion_xyzw']);close(j,f['jacobian'])
    assert model.joint_names==f['joint_names']


def test_encoder_model_arguments_and_tool_audit_conversion_against_golden():
    d=D['encoder_fk_adapter'];called={}
    class Affine:
        def __init__(self):self.matrix=np.eye(4)
    class Model:
        def pose(self,frame,q,a,b):
            called.update(frame=frame,q=list(q),F_T_EE=a.matrix.tolist(),EE_T_K=b.matrix.tolist())
            m=np.eye(4);m[:3,3]=d['position']
            return SimpleNamespace(matrix=m,quaternion=d['quaternion_xyzw'])
    f=SimpleNamespace(Affine=Affine,Frame=SimpleNamespace(Flange='Frame.Flange'))
    fk=EncoderForwardKinematics(SimpleNamespace(model=Model()),f)
    reported=np.eye(4);reported[:3,3]=[.4,.1,.6];tool=np.eye(4);tool[2,3]=.1
    result=_encoder_pose_feedback(SimpleNamespace(q=D['urdf_fk']['q'],O_T_EE=reported,F_T_EE=tool),fk)
    assert called==d['model_call'];close(result.position_m,d['position'])
    assert result.reported_position_error_m==pytest.approx(d['reported_position_error_m'],abs=1e-12)


def test_native_adapter_never_connects_on_construction():
    backend=FrankyMotionBackend({})
    assert backend.robot is None and backend.motion is None
    with pytest.raises(RuntimeError,match='NOT QUALIFIED'):backend.connect()
    with pytest.raises(legacy.ConfigError,match='No explicit'):backend.preflight()


def test_wire_success_threshold_and_fixture_anchor():
    assert command_success_threshold(.95)==float(np.float32(.95))
    assert command_success_threshold(.95)>float(np.nextafter(np.float32(.95),np.float32(0)))
    f=SimpleNamespace(CurveFixtureParams=SimpleNamespace,CurveFixtureType=SimpleNamespace(ZLine='z',YZCircle='circle'))
    c={'task':{'kind':'yz_circle','fixture_anchor_to_start':True},'fixture':{'origin':[0,0,0],'radius':.1}}
    s=SimpleNamespace(flange_position_m=[.4,0,.5],flange_orientation_xyzw=[1,0,0,0])
    result=make_curve_fixture(c,f,s)
    close(result.origin,[.4,0,.4]);assert result.orientation==s.flange_orientation_xyzw


def test_native_stop_exception_keeps_failure_and_never_invents_smooth_stop():
    b=FrankyMotionBackend({'stop':{'max_duration':0}})
    b.f=SimpleNamespace(M4RuntimeMotion=SimpleNamespace(stop_behavior_version=2))
    b.motion=SimpleNamespace(request_stop=lambda:None,stop_fault=18)
    def join(timeout):raise RuntimeError('not at sustained rest')
    b.robot=SimpleNamespace(is_in_control=False,join_motion=join,stop=lambda:pytest.fail('No redundant abort'))
    b._last_sample=SimpleNamespace(stop_finished=False,stop_at_rest=False);b.drain=lambda stream:[]
    assert b.stop(None) is False
    assert b.stop_evidence['stop_fault']==18 and 'rest' in b.stop_evidence['exception']


def test_trajectory_entry_twist_and_phase_timing_against_legacy_source():
    from dataclasses import asdict
    from franka_runtime.common.motion.config import TrajectorySpec
    data=json.loads((FIXTURES/'trajectory-parity-v1.json').read_text())
    for case in data['cases']:
        path=TrajectorySpec(case['name'],case['spec']).build(case['angular_speed_rad_s'],case['initial_position_m'],case['initial_quaternion_xyzw'])
        assert path.duration_s==pytest.approx(case['duration_s'],abs=1e-12)
        for sample in case['samples']:
            result=asdict(path.evaluate(sample['time_s']));expected=sample['expected']
            assert result['phase']==expected['phase']
            for key in ('position_m','quaternion_xyzw','linear_velocity_m_s','angular_velocity_rad_s'):
                close(result['reference'][key],expected['reference'][key])
            close(result['acceleration_m_s2'],expected['acceleration_m_s2'])
            close(result['jerk_m_s3'],expected['jerk_m_s3'])
