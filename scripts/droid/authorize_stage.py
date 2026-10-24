"""Create a single-use run config for one operator-authorized commissioning stage.

Copies the approved (reviewed, not executable) candidate into a new run directory,
pins its artifacts by absolute path, gives the run its own recording directory and
sets executable/motion_authorized only there, with a stage authorization record
bound to the approved candidate hash. The approved candidate itself is unchanged.
"""
import argparse
import json
from copy import deepcopy
from pathlib import Path
from franka_runtime.droid.physical_config import PhysicalDroidConfig, digest


def authorize(*,candidate_dir,run_dir,stage,authorized_by,statement,expected_candidate_sha256):
    candidate_dir,run_dir=Path(candidate_dir).resolve(),Path(run_dir)
    c=PhysicalDroidConfig.load(candidate_dir/'physical.review.pending.json');d=deepcopy(c.data)
    if c.sha256!=expected_candidate_sha256:raise ValueError('Candidate differs from the approved hash')
    if d['status']!='reviewed' or d['executable'] or d['motion_authorized'] or not all(d['approval'].values()):
        raise ValueError('Approved, non-executable reviewed candidate required')
    receipt=json.loads((candidate_dir/'approval-receipt.json').read_text())
    if receipt['approved_candidate_sha256']!=c.sha256:raise ValueError('Approval receipt does not match the candidate')
    for text in (stage,authorized_by,statement):
        if not isinstance(text,str) or not text.strip():raise ValueError('Stage, authorizer and statement required')
    run_dir.mkdir(parents=True)  # never reuse a run directory
    run_dir=run_dir.resolve()
    for group,key in (('workspace','model_urdf'),('artifacts','native_pin'),('artifacts','collision_model')):
        d[group][key]=str(c.path(d[group][key]).resolve())
    d['recording']['directory']=str(run_dir/'recording')
    d.update(executable=True,motion_authorized=True,
             provenance=f'Stage {stage} run of approved candidate {c.sha256}; see stage-authorization.json')
    run=PhysicalDroidConfig(d,str(run_dir/'physical.json'),run_dir);run.require_approved()
    (run_dir/'physical.json').write_text(json.dumps(d,indent=2,allow_nan=False)+'\n')
    record=dict(schema_version=1,stage=stage,authorized_by=authorized_by,statement=statement,
                approved_candidate_sha256=c.sha256,approval_sha256=digest(candidate_dir/'approval.json'),
                run_config_sha256=run.sha256,changed_fields=['executable','motion_authorized','provenance','recording.directory',
                'workspace.model_urdf (absolute)','artifacts.native_pin (absolute)','artifacts.collision_model (absolute)'])
    (run_dir/'stage-authorization.json').write_text(json.dumps(record,indent=2)+'\n')
    return record


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--candidate-dir',type=Path,required=True);p.add_argument('--run-dir',type=Path,required=True)
    p.add_argument('--stage',required=True);p.add_argument('--authorized-by',required=True)
    p.add_argument('--statement',required=True);p.add_argument('--expected-candidate-sha256',required=True)
    print(json.dumps(authorize(**vars(p.parse_args())),indent=2))


if __name__=='__main__':main()
