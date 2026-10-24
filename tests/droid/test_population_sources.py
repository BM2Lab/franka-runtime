"""Offline population regression using archived-value subsets and synthetic receipts."""
import json
from pathlib import Path
import runpy
import pytest
from franka_runtime.droid.commissioning import digest
from franka_runtime.droid.physical_config import PhysicalDroidConfig
from franka_runtime.droid.workspace import SerialChain

ROOT=Path(__file__).resolve().parents[2]
POP=runpy.run_path(str(ROOT/'scripts/droid/populate_review.py'))['populate']
DATA=json.loads((ROOT/'tests/fixtures/droid/population-inputs.json').read_text())


def inputs(tmp_path):
    m4=tmp_path/'m4';m4.mkdir();(m4/'config.yaml').write_text(json.dumps(DATA['m4']))
    report=dict(hardware_executed=True,terminal_state='complete',smooth_stop_ok=True,config_sha256=digest(m4/'config.yaml'))
    (m4/'report.json').write_text(json.dumps(report))
    legacy=tmp_path/'legacy.yaml';legacy.write_text(json.dumps(DATA['legacy']))
    src=tmp_path/'sources';(src/'droid/config/fr3').mkdir(parents=True);(src/'droid/droid/franka').mkdir(parents=True)
    (src/'droid/config/fr3/franka_hardware.yaml').write_text(json.dumps(DATA['droid']))
    (src/'droid/droid/franka/robot.py').write_text('def update_gripper(self):\n    self._gripper.goto(width=0, speed=0.05, force=0.1)\n')
    manifest=[dict(file=str(p.relative_to(src)),sha256=digest(p)) for p in sorted(src.rglob('*')) if p.is_file()]
    (src/'sources.json').write_text(json.dumps(manifest))
    state=tmp_path/'state.json';state.write_text(json.dumps(dict(q=[0.]*7,F_T_EE=[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1],
        m_ee=0,I_ee=[0]*9,F_x_Cee=[0]*3,m_load=0,I_load=[0]*9,F_x_Cload=[0]*3)))
    return dict(repo=ROOT,m4_run=m4,legacy_reference=legacy,droid_sources=src,
        mock_link=ROOT/'configs/fr3/droid.jointpos.mock-link.json',observed_state=state,
        output_dir=tmp_path/'out',recording_dir=tmp_path/'sessions/run')


def test_populated_draft_keeps_tool_identity_and_approval_unknown(tmp_path):
    args=inputs(tmp_path);r=POP(**args)
    c=PhysicalDroidConfig.load(tmp_path/'out/physical.review.pending.json');d=c.data
    assert r['prefilled']==86 and r['remaining']==18
    assert d['controller']['joint_stiffness']==[40,30,50,25,35,25,10]
    assert d['acceptance']['max_consecutive_row_speed_rad_s']==[2.075]*4+[2.51]*3
    assert d['guards']['max_measured_joint_velocity_rad_s']==[.435]*4+[.522]*3
    assert d['robot']['arm_serial'] is None and d['tool']['m_ee'] is None and d['tool']['F_T_EE'] is None
    assert d['gripper']['grasp_force_n']==20 and d['gripper']['speed_m_s']==.05
    assert d['stop']['max_duration_s']==2 and d['timing']['stop_timeout_ns']==2_500_000_000
    assert not d['executable'] and not d['motion_authorized'] and all(v is None for v in d['approval'].values())
    with pytest.raises(ValueError,match='Uncommissioned'):c.require_approved()
    for key in ('native_pin','model_urdf'):
        group='artifacts' if key=='native_pin' else 'workspace'
        h='native_pin_sha256' if key=='native_pin' else 'model_sha256'
        assert digest(c.path(d[group][key]))==d[group][h]
    raw=SerialChain(ROOT/'tests/parity/fixtures/robot_returned_model.urdf','link0','link8')
    alias=SerialChain(c.path(d['workspace']['model_urdf']),'link0','link8')
    assert alias.joint_names==d['workspace']['joint_names']
    q=[.1,-.7,.05,-2.2,.04,1.5,.1]
    assert raw.frames(q)==alias.frames(q)
    receipt=json.loads((tmp_path/'out/provenance.json').read_text())
    assert Path(receipt['sources']['field_evidence']['path']).is_file()


@pytest.mark.parametrize('kind',['report','source','margin'])
def test_bad_archival_evidence_fails_without_output(tmp_path,kind):
    args=inputs(tmp_path)
    if kind=='report':(args['m4_run']/'report.json').write_text(json.dumps(dict(hardware_executed=True,terminal_state='fault',smooth_stop_ok=False)))
    elif kind=='source':(args['droid_sources']/'droid/config/fr3/franka_hardware.yaml').write_text('changed')
    else:
        d=json.loads(args['legacy_reference'].read_text());d['safety']['position_margin_rad'][0]=0
        args['legacy_reference'].write_text(json.dumps(d))
    with pytest.raises(ValueError):POP(**args)
    assert not args['output_dir'].exists()


def test_populated_numeric_settings_validate_with_synthetic_missing_inputs(tmp_path):
    args=inputs(tmp_path);POP(**args)
    c=PhysicalDroidConfig.load(tmp_path/'out/physical.review.pending.json')
    # These stand-ins only exercise schema validation; never write them to a candidate.
    d=c.data;d['robot'].update(arm_serial='synthetic',control_serial='synthetic',identity_max_age_ns=1_000_000_000)
    d['tool'].update(F_T_EE=[[float(i==j) for j in range(4)] for i in range(4)],m_ee=1.,I_ee=[0.]*9,
                     F_x_Cee=[0.]*3,m_load=0.,I_load=[0.]*9,F_x_Cload=[0.]*3)
    model=tmp_path/'synthetic-model.json';model.write_text('{}')
    d['artifacts'].update(collision_model=str(model),collision_model_sha256=digest(model))
    assert c.validated()['controller']['lowpass_cutoff_hz']==100
    with pytest.raises(ValueError,match='not approved'):c.require_approved()


def hand_timing(tmp_path,**changes):
    summary=dict(captured='2026-10-24',read_only=True,hand=dict(samples=50,median_ms=100.18,max_ms=100.74),
                 arm=dict(samples=241,median_ms=20.89,max_ms=21.07),hand_counter_per_s=20.05)
    summary.update(changes);path=tmp_path/'hand-timing.json';path.write_text(json.dumps(summary));return path


def test_measured_hand_timing_sets_freshness_with_evidence(tmp_path):
    args=inputs(tmp_path);POP(**args,hand_timing=hand_timing(tmp_path))
    out=tmp_path/'out';d=PhysicalDroidConfig.load(out/'physical.review.pending.json').data
    assert d['gripper']['max_state_age_ns']==250_000_000
    row=json.loads((out/'provenance.json').read_text())['field_provenance']['gripper.max_state_age_ns']
    assert 'Measured read-only Hand state period' in row['basis'] and 'PROPOSAL' not in row['basis']
    assert row['source']['sha256']==digest(out/'source-evidence/hand_timing.json')


@pytest.mark.parametrize('changes',[dict(hand=dict(samples=10,median_ms=130.,max_ms=130.)),dict(read_only=False)])
def test_hand_timing_without_margin_or_read_only_basis_is_refused(tmp_path,changes):
    with pytest.raises(ValueError,match='Hand timing'):POP(**inputs(tmp_path),hand_timing=hand_timing(tmp_path,**changes))


def test_without_hand_timing_the_proposal_baseline_remains(tmp_path):
    POP(**inputs(tmp_path))
    assert PhysicalDroidConfig.load(tmp_path/'out/physical.review.pending.json').data['gripper']['max_state_age_ns']==100_000_000


FRESH=ROOT/'tests/fixtures/droid/robot-state-with-hand-2026-10-24.json'
BUNDLE=ROOT/'configs/fr3/droid-source-review/assets/collision'


def unit_observation(tmp_path):
    path=tmp_path/'unit.json'
    path.write_text(json.dumps(dict(schema_version=1,address='172.16.0.6',basis='Read-only FCI connection to the current unit')))
    return path


def test_fresh_tool_collision_bundle_and_unit_address_fill_their_fields(tmp_path):
    r=POP(**inputs(tmp_path),fresh_state=FRESH,collision_bundle=BUNDLE,unit_observation=unit_observation(tmp_path))
    out=tmp_path/'out';c=PhysicalDroidConfig.load(out/'physical.review.pending.json');d=c.data
    assert r['remaining']==9 and all(f.startswith(('approval.','robot.')) for f in r['missing'])
    assert d['robot']['address']=='172.16.0.6' and d['tool']['m_ee']==.73
    assert d['tool']['F_T_EE'][2][3]==.1034 and d['tool']['F_T_EE'][1][:2]==[-.707107,.707107]
    assert d['artifacts']['collision_model_sha256']==digest(c.path(d['artifacts']['collision_model']))
    prov=json.loads((out/'provenance.json').read_text())['field_provenance']
    assert 'not a per-unit calibration' in prov['tool.m_ee']['basis']
    pytest.importorskip('pinocchio')
    from franka_runtime.droid.collision import engine_from_physical_config
    d['robot'].update(arm_serial='synthetic',control_serial='synthetic',identity_max_age_ns=1)  # schema stand-ins only
    engine=engine_from_physical_config(c)
    assert engine.check_configuration((0.005415,-0.783251,-0.000904,-2.353230,-0.013001,1.562240,-0.001859))


def test_collision_bundle_with_a_different_hand_mount_is_refused(tmp_path):
    state=json.loads(FRESH.read_text());state['F_T_EE'][14]=.2
    fresh=tmp_path/'fresh.json';fresh.write_text(json.dumps(state))
    with pytest.raises(ValueError,match='Hand mount'):POP(**inputs(tmp_path),fresh_state=fresh,collision_bundle=BUNDLE)
    assert not (tmp_path/'out').exists()
