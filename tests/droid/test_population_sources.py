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
