"""Stage authorization and physical CLI composition with fake devices; no SDK or socket."""
import json
from pathlib import Path
import runpy
import pytest
from franka_runtime.droid.physical_config import PhysicalDroidConfig, digest

ROOT=Path(__file__).resolve().parents[2]
POPS=runpy.run_path(str(Path(__file__).with_name('test_population_sources.py')))
ADM=runpy.run_path(str(Path(__file__).with_name('test_admission.py')))
AUTH=runpy.run_path(str(ROOT/'scripts/droid/authorize_stage.py'))['authorize']
HOME=(0.005415,-0.783251,-0.000904,-2.353230,-0.013001,1.562240,-0.001859)


class Factory(ADM['Factory']):
    def connect(self,executor,*,spec,authorize_device_setup=False,recorder=None):
        assert authorize_device_setup and recorder.fault is None
        from franka_runtime.droid.native import NativeDroidBackend
        n=ADM['NATIVE'];backend=NativeDroidBackend(executor,n['Driver'](self.clock,HOME),n['Hand'](),clock=self.clock,
            startup_timeout_ns=1_000_000_000,consumption_timeout_ns=50_000_000,stop_timeout_ns=3_000_000_000,state_timeout_ns=50_000_000)
        backend.configuration_sha256=self.config.sha256;self.backend=backend
        return backend


def approved(tmp_path):
    POPS['POP'](**POPS['inputs'](tmp_path),fresh_state=POPS['FRESH'],collision_bundle=POPS['BUNDLE'],
                unit_observation=POPS['full_unit_observation'](tmp_path))
    out=tmp_path/'out';c=PhysicalDroidConfig.load(out/'physical.review.pending.json')
    receipt=POPS['APPROVE'](candidate_dir=out,approval=POPS['approval'](tmp_path),expected_sha256=c.sha256)
    return out,receipt['approved_candidate_sha256']


def stage(tmp_path,**changes):
    out,sha=approved(tmp_path)
    args=dict(candidate_dir=out,run_dir=tmp_path/'run',stage='D8 hold',authorized_by='tester',statement='synthetic',
              expected_candidate_sha256=sha);args.update(changes)
    return out,AUTH(**args)


def test_stage_run_config_is_executable_only_in_its_own_directory(tmp_path):
    out,record=stage(tmp_path)
    run=PhysicalDroidConfig.load(tmp_path/'run/physical.json');d=run.require_approved()
    assert record['run_config_sha256']==run.sha256 and d['recording']['directory']==str((tmp_path/'run/recording').resolve())
    assert digest(run.path(d['artifacts']['collision_model']))==d['artifacts']['collision_model_sha256']
    original=PhysicalDroidConfig.load(out/'physical.review.pending.json').data
    assert not original['executable'] and not original['motion_authorized']
    with pytest.raises(FileExistsError):AUTH(candidate_dir=out,run_dir=tmp_path/'run',stage='x',authorized_by='x',
                                             statement='x',expected_candidate_sha256=record['approved_candidate_sha256'])


def test_stage_refuses_a_different_or_unapproved_candidate(tmp_path):
    with pytest.raises(ValueError,match='approved hash'):stage(tmp_path,expected_candidate_sha256='0'*64)
    assert not (tmp_path/'run').exists()


def test_physical_cli_composes_admission_with_bound_identity_and_collision(tmp_path):
    pytest.importorskip('pinocchio')
    from franka_runtime.droid.physical_cli import prepare_physical
    stage(tmp_path);run=PhysicalDroidConfig.load(tmp_path/'run/physical.json')
    clock=ADM['NATIVE']['Clock']();identity=tmp_path/'identity.json';identity.write_text('{}')
    common=dict(identity=identity,identity_sha256=digest(identity),clock=clock,clock_id='test:monotonic',token='t',
                factory_class=Factory)
    spec=ADM['SPEC']
    with pytest.raises(ValueError,match='authorization'):prepare_physical(run,spec,**common)
    prepared=prepare_physical(run,spec,authorize_device_setup=True,authorize_motion=True,**common)
    assert prepared.configuration_sha256==run.sha256 and prepared.service.backend.hardware_executed
    prepared.service.shutdown();assert prepared.close(timeout_s=.1)['ok']
    (tmp_path/'run/stage-authorization.json').write_text(json.dumps(dict(run_config_sha256='0'*64)))
    with pytest.raises(ValueError,match='Stage authorization'):
        prepare_physical(run,spec,authorize_device_setup=True,authorize_motion=True,**common)
