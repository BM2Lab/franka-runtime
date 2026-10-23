import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import hashlib
import pytest
# PPO/M4 depend on contracts.motion and contracts.franka_policy, which no released
# contracts version provides yet (see docs/droid-runtime.md). Skip, do not hide.
pytest.importorskip("contracts.motion", reason="contracts.motion is not in contracts 0.0.2")
np=pytest.importorskip("numpy")
pytest.importorskip("yaml")
yaml=pytest.importorskip("yaml")
from franka_runtime.common.artifacts.bundle import verify_motion_bundle,sha256
from franka_runtime.common.backends.franky_m4 import load_hardware_config
from franka_runtime.common.backends.offline import MockMotionBackend
from franka_runtime.common.motion.payload import payload_validator
from franka_runtime.service.cli import main
from franka_runtime.service.replay import replay_servo

ROOT=Path(__file__).parents[2]
FIXTURES=ROOT/'tests/parity/fixtures'


@pytest.fixture
def bundle(tmp_path):
    b=tmp_path/'bundle';(b/'model').mkdir(parents=True);(b/'config').mkdir()
    (b/'model/robot_returned_model.urdf').write_bytes((FIXTURES/'robot_returned_model.urdf').read_bytes())
    (b/'config/trajectories.yaml').write_text('schema_version: 1\ntrajectories: {}\n')
    manifest={'files':{p.relative_to(b).as_posix():sha256(p) for p in b.rglob('*') if p.is_file()}}
    (b/'manifest.json').write_text(json.dumps(manifest))
    return b


@pytest.fixture
def config(tmp_path,bundle):
    p=tmp_path/'config.yaml'
    c=yaml.safe_load((ROOT/'configs/fr3/m4.hold.pending.yaml').read_text())
    c.update(bundle=str(bundle),bundle_manifest_sha256=sha256(bundle/'manifest.json'))
    c['task']['duration_s']=.06
    p.write_text(yaml.safe_dump(c));return p


def test_data_only_bundle_integrity_inventory_and_anchor(tmp_path,bundle):
    anchor=sha256(bundle/'manifest.json');assert verify_motion_bundle(bundle,anchor)
    with pytest.raises(ValueError,match='hash'):verify_motion_bundle(bundle,'0'*64)
    (bundle/'extra').write_text('unexpected')
    with pytest.raises(ValueError,match='inventory'):verify_motion_bundle(bundle,anchor)
    (bundle/'extra').unlink();(bundle/'link').symlink_to('/tmp')
    with pytest.raises(ValueError,match='symlinks'):verify_motion_bundle(bundle,anchor)


def test_installed_mock_and_shadow_commands(config,tmp_path,capsys):
    state=ROOT/'configs/fr3/m4.mock-start.json'
    assert main(['preflight','--config',str(config),'--state',str(state)])==0
    for shadow in (False,True):
        out=tmp_path/('shadow' if shadow else 'mock')
        args=['mock','--config',str(config),'--state',str(state),'--output',str(out)]
        if shadow:args.append('--shadow')
        assert main(args)==0
        report=json.loads((out/'report.json').read_text())
        assert report['terminal_state']=='complete' and report['hardware_executed'] is False
        assert report['submitted']==3 and report['applied_receipts']==(0 if shadow else 3)
        assert report['stop_ok']
        assert main(args)==1 # exclusive artifacts


def test_six_pending_profiles_validate_without_bundle_or_native_modules():
    for name in ('hold','tracking','dls','posture','z_line','yz_circle'):
        c=load_hardware_config(ROOT/f'configs/fr3/m4.{name}.pending.yaml')
        assert c['motion_authorized'] is False and not c['executable']
        assert '/home/' not in json.dumps(c)
        with pytest.raises(ValueError):load_hardware_config(ROOT/f'configs/fr3/m4.{name}.pending.yaml',execute=True)


def test_payload_frames_bounds_and_task_admission(config):
    from contracts.motion import CartesianReference
    c=load_hardware_config(config);b=MockMotionBackend(c,json.loads((ROOT/'configs/fr3/m4.mock-start.json').read_text()))
    s=b.preflight();v=payload_validator(c)
    p=CartesianReference('arm','fr3_link0','fr3_flange',s.flange_position_m,s.flange_orientation_xyzw,(0,)*6).to_dict()
    assert v(p)==p
    for invalid in (dict(p,base_frame='world'),dict(p,position_m=[9,9,9]),dict(p,posture_rad=[0]*7),dict(p,twist_linear_angular=[9]*6)):
        with pytest.raises(ValueError):v(invalid)


def test_replay_stream_checks_actual_record_fields_and_processing(tmp_path,bundle):
    from franka_runtime.common.motion.kinematics import UrdfKinematicsProvider
    q=json.loads((ROOT/'configs/fr3/m4.mock-start.json').read_text())['q']
    p,quat,_=UrdfKinematicsProvider(bundle/'model/robot_returned_model.urdf').evaluate(q)
    pose=np.eye(4);pose[:3,3]=p
    alpha=.001/(.001+1/(2*np.pi*100))
    row={'tick':1,'monotonic_ns':1000,'fault':0,'q':q,'pose':pose.tolist(),'ee_feedback_valid':True,
         'torque_previous':[0]*7,'slew':[1]*7,'filtered_prediction':[float(alpha)]*7,'stop_finished':True}
    log=tmp_path/'servo.jsonl';log.write_text(json.dumps(row)+'\n')
    result=replay_servo(log,bundle/'model/robot_returned_model.urdf')
    assert result['numerical_checks_passed'] and result['valid_fk_rows']==1 and result['stop_finished_seen']
    log.write_text(json.dumps(row)+'\n'+json.dumps(row)+'\n')
    with pytest.raises(ValueError,match='Nonmonotonic'):replay_servo(log,bundle/'model/robot_returned_model.urdf')


def test_import_isolation_includes_absent_legacy_and_vendor_packages():
    code='''
import sys,importlib.abc
class Block(importlib.abc.MetaPathFinder):
 def find_spec(self,fullname,path=None,target=None):
  if fullname.split('.')[0] in ('franka_motion','franky','franky_experiment','learning','isaaclab','torch','rclpy'):
   raise AssertionError('Forbidden import '+fullname)
sys.meta_path.insert(0,Block())
import franka_runtime
import franka_runtime.service.authority
import franka_runtime.service.cli
import franka_runtime.common.backends.franky_m4
import franka_runtime.common.backends.offline
from franka_runtime.common.backends.franky_m4 import FrankyMotionBackend
assert FrankyMotionBackend({}).robot is None
'''
    subprocess.run([sys.executable,'-c',code],check=True)


@pytest.mark.parametrize('change', ['boolean_gain','manifest_identity','reference_acceleration'])
def test_config_admission_rejects_bad_numeric_or_artifact_data(tmp_path,change):
    c=yaml.safe_load((ROOT/'configs/fr3/m4.hold.pending.yaml').read_text())
    if change=='boolean_gain':c['controller']['stiffness'][0]=True
    if change=='manifest_identity':c['bundle_manifest_sha256']='not-a-hash'
    if change=='reference_acceleration':c['reference_limits']['acceleration_m_s2']=-1
    p=tmp_path/'invalid.yaml';p.write_text(yaml.safe_dump(c))
    with pytest.raises(ValueError):load_hardware_config(p)
