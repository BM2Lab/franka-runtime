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
    assert r['prefilled']==87 and r['remaining']==18
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


APPROVE=runpy.run_path(str(ROOT/'scripts/droid/record_approval.py'))['record']
NEVER=2**63-1


def full_unit_observation(tmp_path):
    path=tmp_path/'unit.json'
    path.write_text(json.dumps(dict(schema_version=1,address='172.16.0.6',basis='Read-only FCI connection',
        arm_serial='synthetic-arm',basis_arm_serial='Operator-read label',control_serial='synthetic-control',
        basis_control_serial='Operator-read label',identity_max_age_ns=NEVER,basis_identity_max_age_ns='Operator decision')))
    return path


def approval(tmp_path,**changes):
    a=dict(schema_version=1,statement='synthetic',approved_by='tester',approved_at='2026-10-24T00:00:00Z',
           physical_stop_procedure='synthetic',workspace_review='synthetic',hand_review='synthetic',collision_review='synthetic')
    a.update(changes);path=tmp_path/'approval.json';path.write_text(json.dumps(a));return path


def test_unit_identity_then_explicit_approval_marks_reviewed_but_not_executable(tmp_path):
    r=POP(**inputs(tmp_path),fresh_state=FRESH,collision_bundle=BUNDLE,unit_observation=full_unit_observation(tmp_path))
    out=tmp_path/'out';assert r['remaining']==6 and all(f.startswith('approval.') for f in r['missing'])
    c=PhysicalDroidConfig.load(out/'physical.review.pending.json')
    assert c.data['robot']['identity_max_age_ns']==NEVER and c.data['robot']['arm_serial']=='synthetic-arm'
    with pytest.raises(ValueError,match='reviewed hash'):APPROVE(candidate_dir=out,approval=approval(tmp_path),expected_sha256='0'*64)
    receipt=APPROVE(candidate_dir=out,approval=approval(tmp_path),expected_sha256=c.sha256)
    d=PhysicalDroidConfig.load(out/'physical.review.pending.json')
    assert d.data['status']=='reviewed' and d.data['approval']['approved_by']=='tester' and not d.missing()
    assert receipt['approved_candidate_sha256']==d.sha256 and not receipt['motion_authorized']
    with pytest.raises(ValueError,match='not approved/executable'):d.require_approved()
    with pytest.raises(ValueError,match='unapproved review draft'):
        APPROVE(candidate_dir=out,approval=approval(tmp_path),expected_sha256=d.sha256)


def test_approval_refuses_incomplete_candidate_or_record(tmp_path):
    POP(**inputs(tmp_path));out=tmp_path/'out';c=PhysicalDroidConfig.load(out/'physical.review.pending.json')
    with pytest.raises(ValueError,match='Uncommissioned'):APPROVE(candidate_dir=out,approval=approval(tmp_path),expected_sha256=c.sha256)
    full=tmp_path/'full';full.mkdir()
    POP(**{**inputs(full),'output_dir':full/'out'},fresh_state=FRESH,collision_bundle=BUNDLE,unit_observation=full_unit_observation(full))
    c=PhysicalDroidConfig.load(full/'out/physical.review.pending.json')
    with pytest.raises(ValueError,match='nonempty'):
        APPROVE(candidate_dir=full/'out',approval=approval(tmp_path,hand_review=' '),expected_sha256=c.sha256)
    assert PhysicalDroidConfig.load(full/'out/physical.review.pending.json').data['status']=='pending_lab_review'


BUILD=runpy.run_path(str(ROOT/'scripts/droid/build_payload.py'))['build']


def payload(tmp_path,**changes):
    v=BUILD(name='wrist-d435i',components=[('D435i',.072),('mount',.051)],com_tcp_m=(-.1,-.04,-.07),state=FRESH,
            box_m=(.09,.025,.025),box_component='D435i',basis='weighed; COM measured from TCP')
    v.update(changes);path=tmp_path/'payload.json';path.write_text(json.dumps(v));return path


def test_payload_converts_tcp_com_to_flange_and_fills_load_fields(tmp_path):
    v=json.loads(payload(tmp_path).read_text())
    assert v['m_load']==.123 and [round(x,4) for x in v['F_x_Cload']]==[-.099,.0424,.0334]
    assert v['I_load'][0]>0 and v['I_load'][1]==0  # column-major diagonal box inertia
    r=POP(**inputs(tmp_path),fresh_state=FRESH,collision_bundle=BUNDLE,unit_observation=full_unit_observation(tmp_path),
          payload=payload(tmp_path))
    d=PhysicalDroidConfig.load(tmp_path/'out/physical.review.pending.json').data
    assert r['remaining']==6 and d['tool']['m_load']==.123 and d['tool']['F_x_Cload']==v['F_x_Cload']
    assert d['tool']['m_ee']==.73  # Desk Hand profile unchanged
    prov=json.loads((tmp_path/'out/provenance.json').read_text())['field_provenance']
    assert 'Operator-measured payload wrist-d435i' in prov['tool.m_load']['basis']


def test_implausible_payload_is_refused(tmp_path):
    with pytest.raises(ValueError,match='Payload'):
        POP(**inputs(tmp_path),fresh_state=FRESH,payload=payload(tmp_path,m_load=5.))


def test_rig_box_and_true_fingertips_become_workspace_tool_points(tmp_path):
    path=payload(tmp_path,bounding_box=runpy.run_path(str(ROOT/'scripts/droid/build_payload.py'))['rig_box'](
        [[.707107,.707107,0],[-.707107,.707107,0],[0,0,1]],[0,0,.1034],(-.12,-.075,-.10),(0.,.075,-.05)))
    POP(**inputs(tmp_path),fresh_state=FRESH,collision_bundle=BUNDLE,unit_observation=full_unit_observation(tmp_path),payload=path)
    w=PhysicalDroidConfig.load(tmp_path/'out/physical.review.pending.json').data['workspace'];pts,body=w['tool_points_m'],w['body_points_m']
    assert len(pts)==3 and pts[0]==[0.,0.,.1034] and len(body)==8
    assert [round(v,4) for v in pts[1]]==[.0283,.0283,.1034]  # fingertip along the TCP y axis, not the flange y axis
    assert min(p[2] for p in body)==pytest.approx(.0034,abs=1e-6)


def test_operator_workspace_box_replaces_the_historical_box(tmp_path):
    box=tmp_path/'box.json'
    box.write_text(json.dumps(dict(schema_version=1,box_min_m=[.27,-.38,.03],box_max_m=[.88,.38,.53],basis='operator-measured')))
    POP(**inputs(tmp_path),workspace_observation=box)
    w=PhysicalDroidConfig.load(tmp_path/'out/physical.review.pending.json').data['workspace']
    assert w['box_min_m']==[.27,-.38,.03] and w['box_max_m']==[.88,.38,.53] and w['clearance_m']==.05  # mock table kept
    table=tmp_path/'table';table.mkdir();box2=table/'box.json'
    box2.write_text(json.dumps(dict(schema_version=1,box_min_m=[.27,-.38,.03],box_max_m=[.88,.38,.53],table_z_m=.03,clearance_m=.02,basis='operator')))
    POP(**inputs(table),workspace_observation=box2)
    w=PhysicalDroidConfig.load(table/'out/physical.review.pending.json').data['workspace']
    assert (w['table_z_m'],w['clearance_m'])==(.03,.02)
    box.write_text(json.dumps(dict(schema_version=1,box_min_m=[.9,-.38,.03],box_max_m=[.88,.38,.53],basis='x')))
    bad=tmp_path/'bad';bad.mkdir()
    with pytest.raises(ValueError,match='Workspace observation'):POP(**inputs(bad),workspace_observation=box)
