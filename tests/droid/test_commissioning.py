import json
from pathlib import Path
import pytest
from contracts.droid import DroidIOSpec
from franka_runtime.droid.commissioning import prepare_review,digest
from franka_runtime.droid.physical_config import PhysicalDroidConfig

ROOT=Path(__file__).resolve().parents[2]
CONTRACT=ROOT/'configs/fr3/droid_fr3_jointpos_v1.contract.json'
PIN=ROOT/'src/franka_runtime/droid/droid-server10.candidate.json'
AUDIT=ROOT/'tests/fixtures/droid/hybrid-feedback.json'


def make(tmp_path,**extra):
    return prepare_review(contract_path=CONTRACT,native_pin_path=PIN,audit_fixture_path=AUDIT,
                          output_dir=tmp_path/'review',**extra)


def test_review_has_bound_software_and_no_approvals(tmp_path):
    r=make(tmp_path);out=tmp_path/'review';c=PhysicalDroidConfig.load(out/'physical.review.pending.json')
    assert not c.data['executable'] and not c.data['motion_authorized'] and not r['settings_validated']
    assert c.data['approval']['approved_by'] is None and c.data['robot']['arm_serial'] is None
    assert c.data['robot']['firmware_version'] is None
    assert c.data['artifacts']['native_pin_sha256']==digest(out/'native-pin.json')
    assert c.data['controller']['joint_stiffness'] is None
    ref=json.loads((out/'controller.reference.json').read_text())
    assert ref['controller']['joint_stiffness']==[40,30,50,25,35,25,10]
    assert r['cross_host_mock_requires_physical_approval'] is False
    with pytest.raises(ValueError,match='Uncommissioned'):c.require_approved()
    with pytest.raises(ValueError,match='disabled'):c.require_runnable()
    with pytest.raises(FileExistsError):make(tmp_path)


def evidence(tmp_path,field,value,**override):
    source=tmp_path/'operator-notes.md';source.write_text('Synthetic decision, not approval.')
    entry=dict(field=field,value=value,source=source.name,sha256=digest(source),basis='synthetic operator decision')
    entry.update(override)
    p=tmp_path/'evidence.json';p.write_text(json.dumps(dict(schema_version=1,entries=[entry])))
    return p


def test_sourced_decision_populates_only_the_draft(tmp_path):
    r=make(tmp_path,evidence_path=evidence(tmp_path,'controller.joint_stiffness',[40.]*7))
    c=PhysicalDroidConfig.load(tmp_path/'review/physical.review.pending.json')
    assert c.data['controller']['joint_stiffness']==[40.]*7 and not c.data['executable']
    assert r['field_provenance']['controller.joint_stiffness']['source']['sha256']


@pytest.mark.parametrize('field,value', [('approval.approved_by','auto'),('executable',True),
 ('robot.fci_version',6),('contract.capability_prefix','other'),('robot.unknown',1)])
def test_no_approval_bypass_or_wrong_family(tmp_path,field,value):
    with pytest.raises(ValueError):make(tmp_path,evidence_path=evidence(tmp_path,field,value))
    assert not (tmp_path/'review').exists()


def test_changed_evidence_source_fails_before_output(tmp_path):
    p=evidence(tmp_path,'controller.joint_stiffness',[40.]*7)
    (tmp_path/'operator-notes.md').write_text('changed')
    with pytest.raises(ValueError,match='hash'):make(tmp_path,evidence_path=p)
    assert not (tmp_path/'review').exists()


def test_differently_formatted_pin_hash_matches_the_copy(tmp_path):
    pin=tmp_path/'compact-pin.json';pin.write_text(json.dumps(json.loads(PIN.read_text()),separators=(',',':')))
    prepare_review(contract_path=CONTRACT,native_pin_path=pin,audit_fixture_path=AUDIT,output_dir=tmp_path/'review')
    c=PhysicalDroidConfig.load(tmp_path/'review/physical.review.pending.json')
    assert c.data['artifacts']['native_pin_sha256']==digest(tmp_path/'review/native-pin.json')


def test_inconsistent_audit_gains_fail(tmp_path):
    audit=json.loads(AUDIT.read_text());audit['cases'][1]['Kq'][0]+=1
    p=tmp_path/'audit.json';p.write_text(json.dumps(audit))
    with pytest.raises(ValueError,match='gains'):
        prepare_review(contract_path=CONTRACT,native_pin_path=PIN,audit_fixture_path=p,output_dir=tmp_path/'review')
