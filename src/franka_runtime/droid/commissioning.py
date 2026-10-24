"""Offline commissioning review pack. Never approves, contacts or controls a robot."""
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
from contracts.droid import DroidIOSpec
from contracts.droid_session import capability_id
from contracts.sessions import payload_sha256
from .physical_config import PhysicalDroidConfig, FIELDS, pending_template
from .providers import IDENTITY_FIELDS


def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_json(path):
    with Path(path).open('rb') as f:data=f.read(8_388_609)
    if len(data)>8_388_608:raise ValueError('Review input exceeds 8 MiB')
    obj=json.loads(data)
    json.dumps(obj,allow_nan=False) # Reject nonfinite JSON even in partial drafts.
    return obj


def sha(value):
    if not isinstance(value,str) or len(value)!=64 or any(c not in '0123456789abcdef' for c in value):
        raise ValueError('Expected SHA256')
    return value


def gain_reference(fixture):
    cases=fixture.get('cases',[])
    if not cases:raise ValueError('Audit fixture has no numerical cases')
    mapping={'Kq':('joint_stiffness',7),'Dq':('joint_damping',7),
             'Kx':('cartesian_stiffness',6),'Dx':('cartesian_damping',6)}
    out={}
    for key,(name,n) in mapping.items():
        v=cases[0].get(key)
        if (not isinstance(v,list) or len(v)!=n or any(isinstance(x,bool) or not isinstance(x,(int,float))
                or not math.isfinite(x) or x<0 for x in v) or any(case.get(key)!=v for case in cases)):
            raise ValueError('Inconsistent/invalid audited gains: '+key)
        out[name]=v
    return dict(status='reference_only_not_commissioned',controller=out,scope=fixture.get('scope'))


def prepare_review(*,contract_path,native_pin_path,audit_fixture_path,output_dir,evidence_path=None):
    contract_path,native_pin_path,audit_fixture_path=map(Path,(contract_path,native_pin_path,audit_fixture_path))
    contract_data=read_json(contract_path);spec=DroidIOSpec.from_dict(contract_data)
    pin=read_json(native_pin_path);audit=read_json(audit_fixture_path)
    if pin.get('family')!='server10' or not isinstance(pin.get('files'),dict) or not pin['files']:
        raise ValueError('Expected a pinned server10 native candidate')
    sha(pin['wheel_sha256'])
    for h in pin['files'].values():sha(h)
    reference=gain_reference(audit)
    config=pending_template();provenance={}
    sources={}
    for label,path in [('contract',contract_path),('native_pin',native_pin_path),('controller_audit',audit_fixture_path)]:
        sources[label]=dict(path=str(path.resolve()),sha256=digest(path))
    def put(field,value,source,basis):
        section,key=field.split('.')
        config[section][key]=deepcopy(value)
        provenance[field]=dict(source=source,basis=basis)
    put('robot.robot_id',spec.robot.robot_id,sources['contract'],'logical contract label; not serial identity')
    put('contract.allowed_sha256',[payload_sha256(spec)],sources['contract'],'contract binding')
    put('contract.capability_prefix',capability_id(spec).split(':')[0],sources['contract'],'wire capability binding')
    put('gripper.open_width_m',spec.gripper.open_width_m,sources['contract'],'closure normalization; physical width still needs review')
    put('controller.kind','droid_hybrid_joint_impedance','franka_runtime.droid.physical_factory.build_motion','implemented controller type')
    put('artifacts.native_pin','native-pin.json',sources['native_pin'],'copied candidate pin; not hardware qualification')
    put('artifacts.native_pin_sha256',hashlib.sha256((json.dumps(pin,indent=2,allow_nan=False)+'\n').encode()).hexdigest(),
        sources['native_pin'],'normalized copied file hash; original input hash retained in provenance')
    protected=set(provenance)
    if evidence_path is not None:
        evidence_path=Path(evidence_path);evidence=read_json(evidence_path)
        if set(evidence)!={'schema_version','entries'} or evidence['schema_version']!=1 or not isinstance(evidence['entries'],list):
            raise ValueError('Expected field evidence schema 1')
        seen=set()
        for entry in evidence['entries']:
            if not isinstance(entry,dict) or set(entry)!={'field','value','source','sha256','basis'}:
                raise ValueError('Evidence requires field/value/source/sha256/basis')
            field=entry['field'];parts=field.split('.') if isinstance(field,str) else []
            if len(parts)!=2 or parts[0]=='approval' or parts[0] not in FIELDS or parts[1] not in FIELDS[parts[0]]:
                raise ValueError('Unknown field or approval mutation forbidden')
            if field in seen:raise ValueError('Duplicate field evidence')
            seen.add(field)
            if not isinstance(entry['basis'],str) or not entry['basis'].strip():raise ValueError('Evidence basis required')
            source=Path(entry['source'])
            if not source.is_absolute():source=evidence_path.parent/source
            if digest(source)!=sha(entry['sha256']):raise ValueError('Evidence source hash mismatch')
            if field in protected and config[parts[0]][parts[1]]!=entry['value']:
                raise ValueError('Evidence conflicts with software binding: '+field)
            put(field,entry['value'],dict(path=str(source.resolve()),sha256=entry['sha256']),entry['basis'])
        sources['field_evidence']=dict(path=str(evidence_path.resolve()),sha256=digest(evidence_path))
    config.update(executable=False,motion_authorized=False,status='pending_lab_review',
                  provenance='Offline review draft; see provenance.json. No unit approval, fresh identity, or geometry qualification implied.')
    c=PhysicalDroidConfig(config)
    if config['robot']['fci_version'] is not None and config['robot']['fci_version']!=10:
        raise ValueError('This candidate is server10; firmware/protocol must not be bypassed')
    missing=c.missing()
    groups={section:[k for k in fields if f'{section}.{k}' in missing] for section,fields in FIELDS.items()}
    report=dict(schema_version=1,hardware_accessed=False,approved=False,executable=False,motion_authorized=False,
                config_sha256=c.sha256,software_prefilled=sorted(protected),field_provenance=provenance,
                missing=missing,missing_by_section={k:v for k,v in groups.items() if v},
                sources=sources,settings_validated=False,
                cross_host_mock_requires_physical_approval=False,
                timing_note='Select physical timing margins after cross-host mock measurements; they need not block that mock test.')
    out=Path(output_dir)
    if not out.parent.is_dir():raise ValueError('Review parent directory must already exist')
    out.mkdir(mode=0o700) # Never overwrite an existing review/approval.
    def write(name,value):
        (out/name).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')
    write('physical.review.pending.json',config)
    write('contract.json',contract_data);write('native-pin.json',pin);write('controller-audit.json',audit)
    write('controller.reference.json',reference);write('provenance.json',report)
    write('field-evidence.pending.json',dict(schema_version=1,entries=[]))
    write('identity.pending.json',{k:(1 if k=='schema_version' else None) for k in sorted(IDENTITY_FIELDS)})
    rows=['# DROID commissioning review','',
          'This is a non-executable review draft. No robot was accessed and no approval was generated.','',
          f'{len(provenance)} fields have sourced candidate values; {len(missing)} fields remain unset.','',
          'Upstream gains are in `controller.reference.json` and were not applied to the draft.', '',
          '| Section | Unset fields | Required evidence / decision |','|---|---|---|']
    notes={'robot':'Current unit identity/Desk + trusted FCI evidence; operator collision thresholds',
           'controller':'Choose controller gains and processing explicitly; upstream values are suggestions',
           'joint_bounds':'This unit/model limits and chosen margins',
           'guards':'Reviewed measured-state/command guards', 'acceptance':'Reviewed target lead and row-speed bounds',
           'tool':'Measured/configured Hand, payload, COM, inertia and TCP',
           'workspace':'Exported model, measured base-frame workspace/table and reviewed tool geometry',
           'artifacts':'Vetted self-collision model/engine; software pin alone does not supply one',
           'timing':'Timing measurements; physical margins may follow cross-host mock testing',
           'clock':'Cross-host clock probe measurements', 'gripper':'Hand behavior and cancellation/freshness qualification',
           'stop':'Hold/stop qualification and operator procedure', 'approval':'Operator review, never auto-generated',
           'recording':'Session output/storage budget and recording cadence', 'network':'Explicit dedicated-link addresses and peer policy'}
    for section,keys in report['missing_by_section'].items():
        rows.append(f"| {section} | {', '.join(keys)} | {notes.get(section,'Contract review')} |")
    rows+=['','A partial draft is not a validated physical configuration. Unknown values stay null.',
           'Evidence import checks source hashes and field provenance; it never writes approval flags.',
           'Old archived robot state must not be restamped as a fresh identity observation.',
           'The cross-host mock test can proceed independently of physical commissioning approval.','']
    (out/'REVIEW.md').write_text('\n'.join(rows))
    return report


def main():
    import argparse
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('contract','native-pin','audit-fixture','output-dir'):p.add_argument('--'+key,type=Path,required=True)
    p.add_argument('--evidence',type=Path)
    a=p.parse_args()
    r=prepare_review(contract_path=a.contract,native_pin_path=a.native_pin,audit_fixture_path=a.audit_fixture,
                     output_dir=a.output_dir,evidence_path=a.evidence)
    print(json.dumps(dict(output_dir=str(a.output_dir),prefilled=len(r['field_provenance']),
                         unset=len(r['missing']),executable=False,hardware_accessed=False),indent=2))


if __name__=='__main__':main()
